/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */
#include "geometry_msgs/msg/twist.hpp"
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <interfaces/msg/move_out.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
		subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&OffboardControl::topic_callback, this, std::placeholders::_1));
		subscription2_ = this->create_subscription<interfaces::msg::MoveOut>("/cmd_vel", 10, std::bind(&OffboardControl::topic_callback2, this, std::placeholders::_1));
		  

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
		
		  
		
	}

	void arm();
	void disarm();
	
	int getch(void)
	{
	  int ch;
	  struct termios oldt;
	  struct termios newt;

	  // Store old settings, and copy to new settings
	  tcgetattr(STDIN_FILENO, &oldt);
	  newt = oldt;

	  // Make required changes and apply the settings
	  newt.c_lflag &= ~(ICANON | ECHO);
	  newt.c_iflag |= IGNBRK;
	  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	  newt.c_cc[VMIN] = 1;
	  newt.c_cc[VTIME] = 0;
	  tcsetattr(fileno(stdin), TCSANOW, &newt);

	  // Get the current character
	  ch = getchar();

	  // Reapply old settings
	  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	  return ch;
	}

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
	rclcpp::Subscription<interfaces::msg::MoveOut>::SharedPtr subscription2_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
	void topic_callback2(const interfaces::msg::MoveOut::SharedPtr msg) const;
  	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{

	TrajectorySetpoint msg{};
	//msg.position = {1.0, 20.0, -1.0};
	//msg.yaw = -3.14; // [-PI:PI]
	msg.position = {NAN, NAN, NAN};
	msg.velocity[0] =0;
	msg.velocity[1] =0;
	msg.velocity[2] =-1;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	//velocity_publisher_->publish(msg);
	//RCLCPP_INFO(this->get_logger(), "1");
}

void OffboardControl::topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear.z);
      TrajectorySetpoint msg2{};
      if (msg->linear.z == 100){
        msg2.position = {0.0, 0.0, -1.2};
        msg2.velocity = {NAN, NAN, NAN};
      }
      else if (msg->linear.z ==-100){
      	msg2.position[0] =   0.0;
        msg2.position[1] =  0.0;
        msg2.position[2] =  0.0;
        msg2.velocity = {NAN, NAN, NAN};
      }

      else if (msg->angular.z == 100){
      	msg2.yaw = 3.14;
    		msg2.yawspeed = NAN;
      }
      
      else{
        msg2.velocity[0] =   msg->linear.x;
        msg2.velocity[1] =  msg->linear.y;
        msg2.velocity[2] =  msg->linear.z;
        //msg2.yaw = msg->angular.z;
        msg2.yaw = NAN;
    		msg2.yawspeed = msg->angular.z;
    		//msg2.yawspeed = 10*msg2.yawspeed;
    		//msg2.yawspeed = NAN;
        msg2.position = {NAN, NAN, NAN};
        
      }
	//msg2.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	//RCLCPP_INFO(this->get_logger(), "I published: '%f'", msg2.yawspeed);
	trajectory_setpoint_publisher_->publish(msg2);
    }
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

 

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
 
void OffboardControl::topic_callback2(const interfaces::msg::MoveOut::SharedPtr msg) const
  
    {
      //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->cmd3);
      TrajectorySetpoint msg2{};
      if (msg->cmd3 == 100){
        msg2.position = {0.0, 0.0, -1.2};
        msg2.velocity = {NAN, NAN, NAN};
      }
      else if (msg->cmd3 ==-100){
      	msg2.position[0] =   0.0;
        msg2.position[1] =  -1.0;
        msg2.position[2] =  -1.2;

        
      }

      else{
        msg2.velocity[0] =   msg->cmd1;
        msg2.velocity[1] =  msg->cmd2;
        msg2.velocity[2] =  msg->cmd3;
        msg2.yawspeed = msg->cmd6;
        msg2.position = {NAN, NAN, NAN};
      }
	//msg2.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	//RCLCPP_INFO(this->get_logger(), "I published: '%f'", msg2.position[2]);
	trajectory_setpoint_publisher_->publish(msg2);
    }
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

 

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
	//RCLCPP_INFO(this->get_logger(), "3");
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	

	rclcpp::shutdown();
	return 0;
}
