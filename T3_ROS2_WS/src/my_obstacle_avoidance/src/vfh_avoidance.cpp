
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/bool.hpp"

#include <cmath>
#include <vector>
#include <algorithm>

// Include additional headers for VFH or any other algorithms/tools you're using.
#include "interfaces/msg/commands.hpp"
#include "interfaces/msg/t.hpp"

// Define a simple 3D point struct for clarity
struct Point3D
{
    double x, y, z;
};

class VFHAvoidance : public rclcpp::Node
{
public:
    VFHAvoidance() : Node("vfh_avoidance_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points", 10,
            std::bind(&VFHAvoidance::pointcloud_callback, this, std::placeholders::_1));
        velocity_command_sub_ = this->create_subscription<interfaces::msg::Commands>(
            "/PublishCommand",
            1,
            std::bind(&VFHAvoidance::velocityCommandArrayCallback, this, std::placeholders::_1));
        timestep_sub_ = this->create_subscription<interfaces::msg::T>(
            "/Timestep",
            1,
            std::bind(&VFHAvoidance::TimestepCallback, this, std::placeholders::_1));
        // Initialize other members or services as required.
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_pointcloud", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("histogram_marker", 10);
        trajectory_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        collision_pub_ = this->create_publisher<std_msgs::msg::Bool>("obstacle", 10);
    }

private:
    // datatypes for storing information in class
    float max_distance = 10.0f; // Default range for detecting obstacles
    std::vector<std::vector<double>> velocity_data_;
    std::vector<Point3D> trajectory_;
    std::vector<std::vector<float>> histogram;
    const int num_sectors = 45;
    const float hFoV = 1.5009831567;           // Horizontal field of view in radians
    const float vFoV = 46.74 * (M_PI / 180.0); // Vertical field of view in radians
    const float MAX_DETECTION_RANGE = 10.0;
    const float SAFETY_RADIUS = 0.5; // in meters
    // Callback function to handle the Timestep message
    void TimestepCallback(const interfaces::msg::T::SharedPtr msg)
    {
        bool collision_detected = false;
        int index = msg->t;

        if (index < 0 || index >= static_cast<int>(velocity_data_.size()))
        {
            RCLCPP_ERROR(this->get_logger(), "Received timestep index is out of bounds.");
            return;
        }

        trajectory_.clear();
        Point3D current_position{0.0, 0.0, 0.0};
        trajectory_.push_back(current_position);

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_link"; // Replace with your frame ID
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05; // Set the width of the line
        marker.color.r = 1.0;  // Red
        marker.color.g = 0.0;  // Green
        marker.color.b = 0.0;  // Blue
        marker.color.a = 1.0;  // Alpha

        // Assuming histogram, hFoV, vFoV, num_sectors, and safety_radius are class members or available globally
        for (size_t i = index; i < velocity_data_.size(); ++i)
        {
            const auto &velocity_vector = velocity_data_[i];
            Point3D next_position = {
                current_position.x + velocity_vector[0],
                current_position.y + velocity_vector[1],
                current_position.z + velocity_vector[2]};
            // Check for potential collisions
            if (checkCollision(current_position, next_position, histogram, hFoV, vFoV, num_sectors, SAFETY_RADIUS))
            {
                // If a collision is detected, log an error and stop processing further
                // RCLCPP_ERROR(this->get_logger(), "Collision detected at timestep %zu between points [%f, %f, %f] and [%f, %f, %f].", i, current_position.x, current_position.y, current_position.z, next_position.x, next_position.y, next_position.z);
                // i is the timestep in the future which collision will occur
                // You can use this information to stop the robot or take other actions
                // For now, we will just stop processing further
                // publish collision boolean
                // publish
                collision_detected = true;
                break; // Stop processing as a potential collision is detected
            }
            trajectory_.push_back(current_position);
            // Convert the next_position to a geometry_msgs Point and add to the marker points
            geometry_msgs::msg::Point p;
            p.x = next_position.x;
            p.y = next_position.y;
            p.z = next_position.z;
            marker.points.push_back(p);
            // If no collision, update the current position and add it to the trajectory
            current_position = next_position;
        }
        // Here you can publish the trajectory or perform further processing
        // Publish the entire trajectory as a single marker
        trajectory_pub_->publish(marker);
        std_msgs::msg::Bool collision_msg;
        collision_msg.data = collision_detected;
        collision_pub_->publish(collision_msg);
        // std::cout << "Collision detected: " << collision_detected << std::endl;
    }

    // A function to get the sector indices for a given 3D point
    // Function to calculate the horizontal and vertical indices of the point in the histogram
    std::pair<int, int> getSectorIndices(const Point3D &point,
                                         const float hFoV,
                                         const float vFoV,
                                         const int num_sectors)
    {
        float angle_horizontal = atan2(point.x, point.z);
        float angle_vertical = atan2(point.y, point.z);

        int sector_index_horizontal = round((angle_horizontal + hFoV / 2) * (num_sectors / hFoV));
        int sector_index_vertical = round((angle_vertical + vFoV / 2) * (num_sectors / vFoV));

        return {sector_index_horizontal, sector_index_vertical};
    }

    // Function to check for collision along a path segment with safety radius
    bool checkCollision(const Point3D &start,
                        const Point3D &end,
                        const std::vector<std::vector<float>> &histogram,
                        const float hFoV,
                        const float vFoV,
                        const int num_sectors,
                        const float SAFETY_RADIUS)
    {
        Point3D direction = {end.x - start.x, end.y - start.y, end.z - start.z};
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);

        // std::cout << "Length: " << length << std::endl;

        // Normalize direction
        direction.x /= length;
        direction.y /= length;
        direction.z /= length;

        // Step through the line segment in increments of the safety radius
        for (float t = 0; t <= length; t += SAFETY_RADIUS)
        {
            Point3D point = {start.x + direction.x * t, start.y + direction.y * t, start.z + direction.z * t};

            // std::cout << "Point: " << point.x << ", " << point.y << ", " << point.z << std::endl;
            // Check if point is within the maximum detection range
            if (std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z) > 10.0f)
            {
                // The point is outside the detection range, skip further checks, this is the latest point
                return false;
            }

            std::pair<int, int> sector_indices = getSectorIndices(point, hFoV, vFoV, num_sectors);
            int sector_horizontal = sector_indices.first;
            int sector_vertical = sector_indices.second;

            // std::cout << "Sector indices: " << sector_horizontal << ", " << sector_vertical << std::endl;

            // Check bounds and then collision
            if (sector_horizontal >= 0 && sector_horizontal < num_sectors &&
                sector_vertical >= 0 && sector_vertical < num_sectors)
            {
                float obstacle_distance = histogram[sector_horizontal][sector_vertical];

                // std::cout << "Obstacle distance: " << obstacle_distance << std::endl;

                float point_distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

                // std::cout << "Point distance: " << point_distance << std::endl;

                // take the absolute distance between obstacle distance and point distance
                if (abs(obstacle_distance - point_distance) < SAFETY_RADIUS)
                {
                    // Collision detected
                    return true;
                }
            }
        }

        // No collision detected
        return false;
    }

    void velocityCommandArrayCallback(const interfaces::msg::Commands::SharedPtr msg)
    {
        velocity_data_.clear(); // Clear old data if only the latest is wanted

        // Assuming that the cmd array is properly formatted as 6 x n
        // Check if the cmd array size is a multiple of 6
        if (msg->cmd.size() % 6 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "The cmd array size is not a multiple of 6.");
            return;
        }

        // Process the array in chunks of six to extract each velocity vector
        for (size_t i = 0; i < msg->cmd.size(); i += 6)
        {
            std::vector<double> single_command_data{
                msg->cmd[i],     // cmd1
                msg->cmd[i + 1], // cmd2
                msg->cmd[i + 2], // cmd3
                msg->cmd[i + 3], // cmd4
                msg->cmd[i + 4], // cmd5
                msg->cmd[i + 5]  // cmd6
            };

            velocity_data_.push_back(single_command_data);
        }

        // Print data for debugging with clearer formatting
        for (const auto &data : velocity_data_)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Velocity Vector: [%f, %f, %f, %f, %f, %f]",
                        data[0], data[1], data[2], data[3], data[4], data[5]);
        }
        RCLCPP_INFO(this->get_logger(), "-------------------------------------------------");
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloud);

        // Preprocess the point cloud: remove points farther than 10 meters
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, MAX_DETECTION_RANGE);
        pass.filter(*cloud);

        // 1. Reduce Point Cloud Density, voxelization
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        vg.setInputCloud(cloud);
        vg.setLeafSize(0.05f, 0.05f, 0.05f);
        vg.filter(*cloud_filtered);
        cloud = cloud_filtered;

        // segment ground using surface normals
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles = segmentGround(cloud);

        // visualization section
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_obstacles, output);
        output.header.stamp = this->now();
        output.header.frame_id = "camera_link"; // Adjust this to your frame
        point_cloud_pub_->publish(output);

        // Convert the point cloud to a histogram
        histogram = convertPointCloudToHistogram(cloud);

        // for (size_t i = 0; i < 45; ++i)
        // {
        //     for (size_t j = 0; j < 45; ++j)
        //     {
        //         // Check if the current row is within the bounds of the histogram
        //         if (i < histogram.size() && j < histogram[i].size())
        //         {
        //             std::cout << std::fixed << std::setprecision(2) << std::setw(6) << histogram[i][j] << " ";
        //         }
        //         else
        //         {
        //             std::cout << "      "; // Print empty spaces if the row or column is out of bounds
        //         }
        //     }
        //     std::cout << std::endl;
        // }
        // Publish the histogram as a marker
        publishHistogramAsMarker(histogram);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr segmentGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>(true));

        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.3);
        ne.compute(*cloud_normals);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacles(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t i = 0; i < cloud->points.size(); i++)
        {
            // Keep points where the normal is not too vertical
            if (std::abs(cloud_normals->points[i].normal_y) < 0.7)
            {
                cloud_obstacles->push_back(cloud->points[i]);
            }
        }

        return cloud_obstacles;
    }

    std::vector<std::vector<float>> convertPointCloudToHistogram(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        const int values_to_average = 10;
        // 2D vector to collect all distances for each horizontal and vertical sector
        std::vector<std::vector<std::vector<float>>> sectors(num_sectors, std::vector<std::vector<float>>(num_sectors));

        for (const auto &point : cloud->points)
        {
            float angle_horizontal = atan2(point.x, point.z);
            float angle_vertical = atan2(point.y, point.z);

            if (angle_horizontal >= -hFoV / 2 && angle_horizontal <= hFoV / 2 &&
                angle_vertical >= -vFoV / 2 && angle_vertical <= vFoV / 2)
            {
                int sector_index_horizontal = round((angle_horizontal + hFoV / 2) * (num_sectors / hFoV));
                int sector_index_vertical = round((angle_vertical + vFoV / 2) * (num_sectors / vFoV));

                // Boundary check for sector indices
                if (sector_index_horizontal >= 0 && sector_index_horizontal < num_sectors &&
                    sector_index_vertical >= 0 && sector_index_vertical < num_sectors)
                {
                    sectors[sector_index_horizontal][sector_index_vertical].push_back(sqrt(point.x * point.x + point.y * point.y + point.z * point.z));
                }
            }
        }

        std::vector<std::vector<float>> histogram(num_sectors, std::vector<float>(num_sectors, std::numeric_limits<float>::max()));

        for (int i = 0; i < num_sectors; i++)
        {
            for (int j = 0; j < num_sectors; j++)
            {
                std::sort(sectors[i][j].begin(), sectors[i][j].end()); // Sort in ascending order

                // Compute average of first 10 minimum values
                int count = 0;
                float sum = 0.0f;
                for (const float value : sectors[i][j])
                {
                    if (count >= values_to_average)
                        break;
                    sum += value;
                    count++;
                }

                if (count > 0) // To avoid division by zero
                    histogram[i][j] = sum / static_cast<float>(count);
            }
        }

        return histogram;
    }

    void publishHistogramAsMarker(const std::vector<std::vector<float>> &histogram)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_link"; // Change to your desired frame
        marker.header.stamp = this->now();
        marker.ns = "histogram";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS; // Changed to POINTS
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.1; // Size of POINTS
        marker.scale.y = 0.1; // Size of POINTS, make sure this is non-zero to see the points

        float hFoV = 1.5009831567;
        float vFoV = 46.74 * (M_PI / 180.0); // converting 46.74 degrees to radians

        float angle_step_horizontal = hFoV / histogram.size();
        float angle_step_vertical = vFoV / histogram[0].size(); // assuming a square matrix

        for (size_t i = 0; i < histogram.size(); i++)
        {
            for (size_t j = 0; j < histogram[i].size(); j++)
            {
                // Check if the histogram value indicates an obstacle
                if (histogram[i][j] < max_distance)
                {
                    float angle_horizontal = -hFoV / 2 + i * angle_step_horizontal;
                    float angle_vertical = -vFoV / 2 + j * angle_step_vertical;

                    geometry_msgs::msg::Point p_end;

                    p_end.x = histogram[i][j] * sin(angle_horizontal);
                    p_end.y = histogram[i][j] * sin(angle_vertical);
                    p_end.z = histogram[i][j] * cos(angle_vertical); // distance is now represented in z

                    marker.points.push_back(p_end); // Only add the end point

                    std_msgs::msg::ColorRGBA color;
                    color.r = 0.0;
                    color.g = 1.0;
                    color.b = 0.0;
                    color.a = 1.0;

                    marker.colors.push_back(color); // Add color for each point
                }
            }
        }

        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<interfaces::msg::Commands>::SharedPtr velocity_command_sub_;
    rclcpp::Subscription<interfaces::msg::T>::SharedPtr timestep_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr collision_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VFHAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
