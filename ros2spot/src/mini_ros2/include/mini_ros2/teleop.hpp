#ifndef TELEOP_INCLUDE_GUARD_HPP
#define TELEOP_INCLUDE_GUARD_HPP
/// \file
/// \brief Teleoperation Library that converts Joystick commands to motion
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "mini_interfaces/msg/joy_buttons.hpp"

namespace tele
{

    // \brief Teleop class responsible for convertick Joystick commands into linear and angular velocity
    class Teleop : public rclcpp::Node
    {
    public:
        // \brief Teleop constructor that defines the axes used for control and sets their scaling factor
        Teleop();

        // \brief Handles periodic operations of Teleop, including debounce and twist publishing
        void timerCallback();

        // \brief Takes a Joy messages and converts it to linear and angular velocity (Twist)
        // \param joy: sensor_msgs describing Joystick inputs
        void joyCallback(const sensor_msgs::msg::Joy::ConstPtr& joy);

        // \brief returns the  most recently commanded Twist
        // \returns: Twist
        geometry_msgs::msg::Twist return_twist();

        // \brief returns a boolean indicating whether the movement switch trigger has been pressed
        // \returns: switch_trigger(bool)
        bool return_trigger();

        // \brief returns whether the E-STOP has been pressed
        // \returns: ESTOP(bool)
        bool return_estop();

        /// \brief returns other joystick buttons triggers, arrow pad etc)
        mini_interfaces::msg::JoyButtons return_buttons();

    private:
        // publishers
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::Publisher<mini_interfaces::msg::JoyButtons>::SharedPtr jb_pub_;

        // subscribers
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        // Clients
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr switch_mvmnt_client_;

        // AXES ON JOYSTICK
        int linear_x_ = 0;
        int linear_y_ = 0;
        int linear_z_ = 0;
        int angular_= 0;
        int RB_ = 0;
        int LB_ = 0;
        // BUTTONS ON JOYSTICK
        int sw_ = 0;
        int es_ = 0;
        int RT_ = 0;
        int LT_ = 0;
        int UD_ = 0;
        int LR_ = 0;
        // DEBOUNCE THRESHOLD
        uint64_t debounce_thresh_;
        // AXIS SCALES
        double l_scale_, a_scale_, B_scale_;
        // TWIST
        geometry_msgs::msg::Twist twist_;
        // TRIGGERS
        bool switch_trigger_ = false;
        bool ESTOP_ = false;
        int updown_ = 0;
        int leftright_ = 0;
        bool left_bump_ = false;
        bool right_bump_ = false;

        // timer and timekeeping
        rclcpp::Time current_time_;
        rclcpp::Time last_time_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
    
}

#endif
