/// \file
/// \brief Draws Each Obstacle in RViz using MarkerArrays
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:

#include "rclcpp/rclcpp.hpp"

#include <math.h>
#include <string>
#include <vector>

#include <mini_ros2/joint_control.hpp>


int main(int argc, char** argv)
/// The Main Function ///
{
    rclcpp::init(argc, argv);

    std::shared_ptr<joint_control::JointControl> joint_control = std::make_shared<joint_control::JointControl>();

    rclcpp::spin(joint_control);
    rclcpp::shutdown();
    return 0;
}