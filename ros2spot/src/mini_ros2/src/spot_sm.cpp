/// \file
/// \brief Draws Each Obstacle in RViz using MarkerArrays
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:

#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <string>
#include <vector>

#include <mini_ros2/spot.hpp>

int main(int argc, char** argv)
/// The Main Function ///
{
    // Seconds for timeout
    double timeout = 1.0;

    rclcpp::init(argc, argv);

    std::shared_ptr<spot::Spot> spot_mini = std::make_shared<spot::Spot>(static_cast<uint64_t>(timeout*1e9));

    rclcpp::spin(spot_mini);
    rclcpp::shutdown();

    return 0;
}