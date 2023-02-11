#include "mini_ros2/spot.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <mini_ros2/teleop.hpp>
#include "mini_interfaces/msg/mini_cmd.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <functional>  // To use std::bind

using std::placeholders::_1;
using std::placeholders::_2;

namespace spot
{

	// Spot Constructor
	Spot::Spot(uint64_t threshold_ns) : Node("mini_sm_node"), timeout_ns_(threshold_ns)
	{
		RCLCPP_INFO(this->get_logger(), "STARTING NODE: spot_mini State Machine");

		this->declare_parameter("frequency", 5.0);
		double frequency = this->get_parameter("frequency").get_parameter_value().get<double>();
	

		// TODO: params for frequency and timeout
		// see: http://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html

		// pubs and subs
		teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("teleop", 10, std::bind(&Spot::teleopCallback, this, _1));
		estop_sub_ = this->create_subscription<std_msgs::msg::Bool>("estop", 10, std::bind(&Spot::estopCallback, this, _1));
		mini_pub_ = this->create_publisher<mini_interfaces::msg::MiniCmd>("mini_cmd", 10);
        
		switch_mvmnt_service_ = this->create_service<std_srvs::srv::Empty>("switch_movement", std::bind(&Spot::swmCallback, this, _1, _2));

		// Init MiniCmd - what gets acted upon
		// Placeholder
		mini_cmd_.x_velocity = 0.0;
		mini_cmd_.y_velocity = 0.0;
		mini_cmd_.rate = 0.0;
		mini_cmd_.roll = 0.0;
		mini_cmd_.pitch = 0.0;
		mini_cmd_.yaw = 0.0;
		mini_cmd_.z = 0.0;
		mini_cmd_.faster = 0.0;
		mini_cmd_.slower = 0.0;
		mini_cmd_.motion = "Stop";
		mini_cmd_.movement = "Stepping";

		// Command used to track updates from controllers
		cmd.x_velocity = 0.0;
		cmd.y_velocity = 0.0;
		cmd.rate = 0.0;
        cmd.roll = 0.0;
        cmd.pitch = 0.0;
        cmd.yaw = 0.0;
        cmd.z = 0.0;
		cmd.motion = Stop;
		cmd.movement = Viewing;

		teleop_flag_ = false;
		motion_flag_ = false;
		ESTOP_ = false;

		// init time, timer, and callback
		current_time_ = this->get_clock()->now();
    	last_time_ = this->get_clock()->now();
		timer_ = this->create_wall_timer(
    		std::chrono::duration<double>(1.0/frequency), std::bind(&Spot::timerCallback, this));
	}

	void Spot::teleopCallback(const geometry_msgs::msg::Twist &tw)
	{	
		this->update_command(tw.linear.x, tw.linear.y, tw.linear.z, tw.angular.z, tw.angular.x, tw.angular.y);
	}

	void Spot::estopCallback(const std_msgs::msg::Bool &estop)
	{ 
		if (estop.data)
		{
			this->update_command(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
			motion_flag_ = true;
			if (!ESTOP_)
			{
				RCLCPP_ERROR(this->get_logger(), "ENGAGING MANUAL E-STOP!");
				ESTOP_ = true;
			} else
			{
				RCLCPP_WARN(this->get_logger(), "DIS-ENGAGING MANUAL E-STOP!");
				ESTOP_ = false;
			}
		}

		last_time_ = this->get_clock()->now();
	}

	bool Spot::swmCallback(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> rsp)
	{
		this->switch_movement();
		motion_flag_ = true;
		return true;
	}

	void Spot::timerCallback() {
		current_time_ = this->get_clock()->now();

        spot::SpotCommand cmd = return_command();

        // Condition for sending non-stop command
        if (!motion_flag_ && !(current_time_.nanoseconds() - last_time_.nanoseconds() > timeout_ns_) && !ESTOP_)
        {
          mini_cmd_.x_velocity = cmd.x_velocity;
          mini_cmd_.y_velocity = cmd.y_velocity;
          mini_cmd_.rate = cmd.rate;
          mini_cmd_.roll = cmd.roll;
          mini_cmd_.pitch = cmd.pitch;
          mini_cmd_.yaw = cmd.yaw;
          mini_cmd_.z = cmd.z;
          mini_cmd_.faster = cmd.faster;
          mini_cmd_.slower = cmd.slower;
          // Now convert enum to string
          // Motion
          if (cmd.motion == spot::Go)
          {
            mini_cmd_.motion = "Go";
          } else
          {
            mini_cmd_.motion = "Stop";
          }
          // Movement
          if (cmd.movement == spot::Stepping)
          {
            mini_cmd_.movement = "Stepping";
          } else
          {
            mini_cmd_.movement = "Viewing";
          }

        } else
        {
          mini_cmd_.x_velocity = 0.0;
          mini_cmd_.y_velocity = 0.0;
          mini_cmd_.rate = 0.0;
          mini_cmd_.roll = 0.0;
          mini_cmd_.pitch = 0.0;
          mini_cmd_.yaw = 0.0;
          mini_cmd_.z = 0.0;
          mini_cmd_.faster = 0.0;
          mini_cmd_.slower = 0.0;
          mini_cmd_.motion = "Stop";
        }

        if (current_time_.nanoseconds() - last_time_.nanoseconds() > timeout_ns_)
        {
          RCLCPP_ERROR(this->get_logger(), "TIMEOUT...ENGAGING E-STOP!");
        }

        // Now publish
        mini_pub_->publish(mini_cmd_);
        motion_flag_ = false;		
	}

	void Spot::update_command(const double & vx, const double & vy, const double & z,
							  const double & w, const double & wx, const double & wy)
	{
		// If Command is nearly zero, just give zero
		if (almost_equal(vx, 0.0) and almost_equal(vy, 0.0) and almost_equal(z, 0.0) and almost_equal(w, 0.0))
		{
			cmd.motion = Stop;
			cmd.x_velocity = 0.0;
			cmd.y_velocity = 0.0;
			cmd.rate = 0.0;
			cmd.roll = 0.0;
	        cmd.pitch = 0.0;
	        cmd.yaw = 0.0;
	        cmd.z = 0.0;
	        cmd.faster = 0.0;
	        cmd.slower = 0.0;
		} else
		{
			cmd.motion = Go;
			if (cmd.movement == Stepping)
			{
				// Stepping Mode, use commands as vx, vy, rate, Z
				cmd.x_velocity = vx;
				cmd.y_velocity = vy;
				cmd.rate = w;
				cmd.z = z;
				cmd.roll = 0.0;
		        cmd.pitch = 0.0;
		        cmd.yaw = 0.0;
		        // change clearance height from +- 0-2 * scaling
		        cmd.faster = 1.0 - wx;
		        cmd.slower = -(1.0 - wy);
			} else
			{
				// Viewing Mode, use commands as RPY, Z
				cmd.x_velocity = 0.0;
				cmd.y_velocity = 0.0;
				cmd.rate = 0.0;
				cmd.roll = vy;
		        cmd.pitch = vx;
		        cmd.yaw = w;
		        cmd.z = z;
		        cmd.faster = 0.0;
		        cmd.slower = 0.0;
			}
		}
		
	}

	void Spot::switch_movement()
	{
		if (!almost_equal(cmd.x_velocity, 0.0) and !almost_equal(cmd.y_velocity, 0.0) and !almost_equal(cmd.rate, 0.0))
		{
			RCLCPP_WARN(this->get_logger(), "MAKE SURE BOTH LINEAR [%.2f, %.2f] AND ANGULAR VELOCITY [%.2f] ARE AT 0.0 BEFORE SWITCHING!", cmd.x_velocity, cmd.y_velocity, cmd.rate);

			RCLCPP_WARN(this->get_logger(), "STOPPING ROBOT...");

			cmd.motion = Stop;
			cmd.x_velocity = 0.0;
			cmd.y_velocity = 0.0;
			cmd.rate = 0.0;
			cmd.roll = 0.0;
	        cmd.pitch = 0.0;
	        cmd.yaw = 0.0;
	        cmd.z = 0.0;
	        cmd.faster = 0.0;
	        cmd.slower = 0.0;
		} else
		{
			cmd.x_velocity = 0.0;
			cmd.y_velocity = 0.0;
			cmd.rate = 0.0;
			cmd.roll = 0.0;
	        cmd.pitch = 0.0;
	        cmd.yaw = 0.0;
	        cmd.z = 0.0;
	        cmd.faster = 0.0;
	        cmd.slower = 0.0;
			if (cmd.movement == Viewing)
			{
				RCLCPP_INFO(this->get_logger(), "SWITCHING TO STEPPING MOTION, COMMANDS NOW MAPPED TO VX|VY|W|Z.");

				cmd.movement = Stepping;
				cmd.motion = Stop;
			} else
			{
				RCLCPP_INFO(this->get_logger(), "SWITCHING TO VIEWING MOTION, COMMANDS NOW MAPPED TO R|P|Y|Z.");

				cmd.movement = Viewing;
				cmd.motion = Stop;
			}
		}
	}

	SpotCommand Spot::return_command()
	{
		return cmd;
	}
}