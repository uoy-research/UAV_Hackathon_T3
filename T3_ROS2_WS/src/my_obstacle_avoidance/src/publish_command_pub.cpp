#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/commands.hpp"
#include "interfaces/msg/t.hpp"

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher() : Node("velocity_publisher"), count_(0)
    {
        command_publisher_ = this->create_publisher<interfaces::msg::Commands>("PublishCommand", 10);
        timestep_publisher_ = this->create_publisher<interfaces::msg::T>("Timestep", 10);

        // Initially publish velocity commands
        velocity_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms for initial rapid publishing
            std::bind(&VelocityPublisher::publishVelocityCommands, this));
    }

private:
    void publishVelocityCommands()
    {
        if (count_ < 10)
        {
            auto message = interfaces::msg::Commands();
            std::vector<double> cmds = {
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398,// Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
                0.0, -0.1, 1.0, 0.0, 0.0, -0.785398, // Angular velocity in radians for a 45-degree turn
            };

            message.cmd = cmds;
            command_publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published velocity command #%d", count_);
            ++count_;
        }
        else
        {
            velocity_timer_->cancel(); // Stop the velocity timer after 10 messages
            startTimestepPublisher();  // Start the timestep publisher
        }
    }

    void startTimestepPublisher()
    {
        // Now start publishing timestep at 1-second intervals
        timestep_timer_ = this->create_wall_timer(
            std::chrono::seconds(20),
            std::bind(&VelocityPublisher::publishTimestep, this));
    }

    void publishTimestep()
    {
        auto message = interfaces::msg::T();
        message.t = 0; // For now, the timestep is always 0 for debugging
        timestep_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published timestep");
    }

    rclcpp::Publisher<interfaces::msg::Commands>::SharedPtr command_publisher_;
    rclcpp::Publisher<interfaces::msg::T>::SharedPtr timestep_publisher_; // Placeholder for the custom timestep publisher
    rclcpp::TimerBase::SharedPtr velocity_timer_;
    rclcpp::TimerBase::SharedPtr timestep_timer_;
    int count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
