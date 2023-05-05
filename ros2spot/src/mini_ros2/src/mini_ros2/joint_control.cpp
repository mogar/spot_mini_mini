

#include "rclcpp/rclcpp.hpp"

#include "mini_ros2/joint_control.hpp"
#include <mini_ros2/spot.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "mini_interfaces/msg/joint_angles.hpp"

#include <chrono>
#include <functional>  // To use std::bind

using std::placeholders::_1;

namespace joint_control
{
	JointControl::JointControl() : Node("joint_control_node")
	{
	    RCLCPP_INFO(this->get_logger(), "STARTING NODE: Joint Control");

		// Parameters
		// joint to pin IDs
		// limits for each joint

		// init subscribers
		// note: history depth of 10 as per tutorial
        joint_sub_ = this->create_subscription<mini_interfaces::msg::joint_angles>("spot/joints", 10, std::bind(&JointControl::joint_callback, this, _1));

		// init time, timer, and callback
		current_time_ = this->get_clock()->now();
    	last_time_ = this->get_clock()->now();
	}

	void JointControl::joint_callback(const mini_interfaces::msg::joint_angles::ConstPtr& joints)
	{
		// TODO: get joint angles

		// map joint angles into servo commands

		// send servo commands
	}
}