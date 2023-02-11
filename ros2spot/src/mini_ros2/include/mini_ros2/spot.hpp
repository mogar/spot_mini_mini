#ifndef SPOT_INCLUDE_GUARD_HPP
#define SPOT_INCLUDE_GUARD_HPP
/// \file
/// \brief Spots library which contains control functionality for Spot Mini Mini.
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "mini_interfaces/msg/mini_cmd.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/bool.hpp"

namespace spot
{

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// Note: the fabs function in <cmath> (c++ equivalent of math.h) will
    /// be useful here
    // constexpr are all define in .hpp
    // constexpr allows fcn to be run at compile time and interface with 
    // static_assert tests.
    // Note high default epsilon since using controller
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-1)
    {
        if (fabs(d1 - d2) < epsilon)
        {
            return true;
        } else {
            return false;
        }
    }

    enum Motion {Go, Stop};
    enum Movement {Stepping, Viewing};

    // \brief Struct to store the commanded type of motion, velocity and rate
    struct SpotCommand
    {
        Motion motion = Stop;
        Movement movement = Viewing;
        double x_velocity = 0.0;
        double y_velocity = 0.0;
        double rate = 0.0;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double z = 0.0;
        double faster = 0.0;
        double slower = 0.0;
    };

    // \brief Spot class responsible for high-level motion commands
    class Spot : public rclcpp::Node
    {

    public:
        // \brief Constructor for Spot class
        Spot(uint64_t threshold_ns);

		/// \brief cmd_vel subscriber callback. Records commanded twist
		///
		/// \param tw (geometry_msgs::Twist): the commanded linear and angular velocity
		/** 
		 * This function runs every time we get a geometry_msgs::Twist message on the "cmd_vel" topic.
		 * We generally use the const <message>ConstPtr &msg syntax to prevent our node from accidentally
		 * changing the message, in the case that another node is also listening to it.
		 */
        void teleopCallback(const geometry_msgs::msg::Twist &tw);

	    void estopCallback(const std_msgs::msg::Bool &estop);

        /// Switches the Movement mode from FB (Forward/Backward) to LR (Left/Right)
	    /// and vice versa
	    bool swmCallback(std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> rsp);

	    void timerCallback();

        // \brief updates the type and velocity of motion to be commanded to the Spot
        // \param vx: linear velocity (x)
        // \param vy: linear velocity (y)
        // \param z: robot height
        // \param w: angular velocity
        // \param wx: step height increase
        // \param wy: step height decrease
        void update_command(const double & vx, const double & vy, const double & z,
                            const double & w, const double & wx, const double & wy);

        // \brief changes the commanded motion from Forward/Backward to Left/Right or vice-versa
        void switch_movement();

        // \brief returns the Spot's current command (Motion, v,w) for external use
        // \returns SpotCommand
        SpotCommand return_command();

    private:
        mini_interfaces::msg::MiniCmd mini_cmd_;

        SpotCommand cmd;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
        rclcpp::Publisher<mini_interfaces::msg::MiniCmd>::SharedPtr mini_pub_;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr switch_mvmnt_service_;
        
        // state
        bool teleop_flag_;
        bool motion_flag_;
        bool ESTOP_;

        // timer and timekeeping
        rclcpp::Time current_time_;
        rclcpp::Time last_time_;
        uint64_t timeout_ns_;
        rclcpp::TimerBase::SharedPtr timer_;

    };
    
}

#endif
