#ifndef DISTANCE_COLORISER
#define DISTANCE_COLORISER

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <functional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class DistanceColorizer : public rclcpp::Node
{
	public:
		DistanceColorizer();
		sensor_msgs::msg::PointCloud2::ConstUniquePtr get_point_cloud();

	private:
		void update_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud);
		void update_reference_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose);
		void colorise_point_cloud();

		bool isReferenceInitialised_;
		double reference_x_, reference_y_, reference_z_, max_distance_;
		sensor_msgs::msg::PointCloud2::ConstSharedPtr point_cloud_;
		sensor_msgs::msg::PointCloud2::SharedPtr colorised_point_cloud_;
		
		rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr target_subscription_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
};

#endif