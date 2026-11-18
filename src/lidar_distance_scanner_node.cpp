#include "lidar_distance_scanner/distance_coloriser.hpp"

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DistanceColorizer>());
	rclcpp::shutdown();
	return 0;
}
