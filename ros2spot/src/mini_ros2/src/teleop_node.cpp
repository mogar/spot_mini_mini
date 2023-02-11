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

#include <mini_ros2/teleop.hpp>


int main(int argc, char** argv)
/// The Main Function ///
{
    rclcpp::init(argc, argv);

    std::shared_ptr<tele::Teleop> teleop = std::make_shared<tele::Teleop>();

    rclcpp::spin(teleop);
    rclcpp::shutdown();
    return 0;
}