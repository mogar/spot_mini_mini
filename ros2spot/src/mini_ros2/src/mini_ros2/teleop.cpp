

#include "rclcpp/rclcpp.hpp"

#include "mini_ros2/teleop.hpp"
#include <mini_ros2/spot.hpp>

#include "mini_interfaces/msg/mini_cmd.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "mini_interfaces/msg/joy_buttons.hpp"

#include <chrono>
#include <functional>  // To use std::bind

using std::placeholders::_1;

namespace tele
{
	Teleop::Teleop() : Node("teleop_node")
	{
	    RCLCPP_INFO(this->get_logger(), "STARTING NODE: Teleoperation");

		// Parameters
		this->declare_parameter("frequency", 60);
		double frequency = this->get_parameter("frequency").get_parameter_value().get<double>();
		this->declare_parameter("axis_linear_x", 4);
		linear_x_ = this->get_parameter("axis_linear_x").get_parameter_value().get<int>();
		this->declare_parameter("axis_linear_y", 3);
		linear_y_ = this->get_parameter("axis_linear_y").get_parameter_value().get<int>();
		this->declare_parameter("axis_linear_z", 1);
		linear_z_ = this->get_parameter("axis_linear_z").get_parameter_value().get<int>();
		this->declare_parameter("axis_angular", 0);
		angular_ = this->get_parameter("axis_linear_x").get_parameter_value().get<int>();
		this->declare_parameter("scale_linear", 1.0);
		l_scale_ = this->get_parameter("scale_linear").get_parameter_value().get<double>();
		this->declare_parameter("scale_angular", 1.0);
		a_scale_ = this->get_parameter("scale_angular").get_parameter_value().get<double>();
		this->declare_parameter("scale_bumper", 1.0);
		B_scale_ = this->get_parameter("scale_bumper").get_parameter_value().get<double>();
		this->declare_parameter("button_switch", 0);
		sw_ = this->get_parameter("button_switch").get_parameter_value().get<int>();
		this->declare_parameter("button_estop", 1);
		es_ = this->get_parameter("button_estop").get_parameter_value().get<int>();
		this->declare_parameter("rb", 5);
		RB_ = this->get_parameter("rb").get_parameter_value().get<int>();;
		this->declare_parameter("lb", 2);
		LB_ = this->get_parameter("lb").get_parameter_value().get<int>();
		this->declare_parameter("rt", 5);
		RT_ = this->get_parameter("rt").get_parameter_value().get<int>();
		this->declare_parameter("lt", 4);
		LT_ = this->get_parameter("lt").get_parameter_value().get<int>();
		this->declare_parameter("updown", 7);
		UD_ = this->get_parameter("updown").get_parameter_value().get<int>();
		this->declare_parameter("leftright", 6);
		LR_ = this->get_parameter("leftright").get_parameter_value().get<int>();
		this->declare_parameter("debounce_thresh", 0.15);
		debounce_thresh_ = static_cast<uint64_t>(1E9*this->get_parameter("debounce_thresh").get_parameter_value().get<double>());

		// Set up publishers
		// TODO: what are these last numbers in the call?
		estop_pub_ = this->create_publisher<std_msgs::msg::Bool>("estop", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("teleop", 10);
		jb_pub_ = this->create_publisher<mini_interfaces::msg::JoyButtons>("joybuttons", 10);

		// init subscribers (also handles pub)
		// note: history depth of 10 as per tutorial
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Teleop::joyCallback, this, _1));

		// Client
		switch_mvmnt_client_ = this->create_client<std_srvs::srv::Empty>("switch_movement");
		switch_mvmnt_client_->wait_for_service();

		switch_trigger_ = false;
		ESTOP_ = false;
		updown_ = 0;
        leftright_ = 0;

		left_bump_ = false;
		right_bump_ = false;

		// init time, timer, and callback
		current_time_ = this->get_clock()->now();
    	last_time_ = this->get_clock()->now();
		timer_ = this->create_wall_timer(
    		std::chrono::duration<double>(1.0/frequency), std::bind(&Teleop::timerCallback, this));
	}

	void Teleop::timerCallback() {
		current_time_ = this->get_clock()->now();

        std_msgs::msg::Bool estop;
        estop.data = this->return_estop();

        if (estop.data && (current_time_.nanoseconds() - last_time_.nanoseconds() >= debounce_thresh_))
        {
            RCLCPP_INFO(this->get_logger(), "SENDING E-STOP COMMAND!");
            last_time_ = this->get_clock()->now();
        } else if (!this->return_trigger())
        {
          // Send Twist
          vel_pub_->publish(this->return_twist());
          estop.data = 0;
        } else if (current_time_.nanoseconds() - last_time_.nanoseconds() >= debounce_thresh_)
        {
          // Call Switch Service
          auto e = std::make_shared<std_srvs::srv::Empty::Request>();

          auto result = switch_mvmnt_client_->async_send_request(e);
          // Wait for the result.
          if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_ERROR(this->get_logger(), "Failed to call swithc mvmnt service");
          }

          estop.data = 0;
          last_time_ = this->get_clock()->now();
        }
        // pub buttons
        jb_pub_->publish(this->return_buttons());

        estop_pub_->publish(estop);
        
	}

	void Teleop::joyCallback(const sensor_msgs::msg::Joy::ConstPtr& joy)
	{
		twist_.linear.x = l_scale_*joy->axes[linear_x_];
		twist_.linear.y = l_scale_*joy->axes[linear_y_];
		// NOTE: used to control robot height
		twist_.linear.z = -l_scale_*joy->axes[linear_z_];
		twist_.angular.z = a_scale_*joy->axes[angular_];
		// NOTE: bottom bumpers used for changing step velocity
		twist_.angular.x = B_scale_*joy->axes[RB_];
		twist_.angular.y = B_scale_*joy->axes[LB_];
		

		// Switch Trigger: Button A
		switch_trigger_ = joy->buttons[sw_];

		// ESTOP: Button B
		ESTOP_ = joy->buttons[es_];

		// Arrow Pad
		updown_ = joy->axes[UD_];
		leftright_ = -joy->axes[LR_];

		// Top Bumpers
		left_bump_ = joy->buttons[LT_];
		right_bump_ = joy->buttons[RT_];
	}

	geometry_msgs::msg::Twist Teleop::return_twist()
	{
		return twist_;
	}

	bool Teleop::return_trigger()
	{
		return switch_trigger_;
	}

	bool Teleop::return_estop()
	{
		return ESTOP_;
	}

	mini_interfaces::msg::JoyButtons Teleop::return_buttons()
	{
		mini_interfaces::msg::JoyButtons jb;
		jb.updown = updown_;
		jb.leftright = leftright_;
		jb.left_bump = left_bump_;
		jb.right_bump = right_bump_;

		return jb;
	}	
}