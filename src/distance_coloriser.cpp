#include "lidar_distance_scanner/distance_coloriser.hpp"

DistanceColorizer::DistanceColorizer() : Node("lidar_distance_scanner_node")
{
    // Parameters:
    std::string points_topic = this->declare_parameter<std::string>("points_topic");
    std::string coloured_points_topic = this->declare_parameter<std::string>("coloured_points_topic");
    std::string target_point_topic = this->declare_parameter<std::string>("target_point_topic");
    max_distance_ = this->declare_parameter<double>("max_distance", 100.0); // in meters?
    int64_t publisher_interval = this->declare_parameter<int64_t>("publisher_interval", 5); // in milliseconds

    if (!std::isfinite(max_distance_) || max_distance_ <= 0.0) 
    {
        throw std::invalid_argument("max_distance must be positive and finite");
    }

    // If there's no reference point yet, we want all the point's color to be white
    this->isReferenceInitialised_ = false;

    // Initialising so that it is not a null.
    this->point_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    this->colorised_point_cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // Declare publishers and subscribers
    this->publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(coloured_points_topic, rclcpp::SensorDataQoS());
    this->subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        points_topic, 
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_pointcloud_map_data), rmw_qos_profile_pointcloud_map_data), 
        std::bind(&DistanceColorizer::update_point_cloud, this, std::placeholders::_1)
    );
    this->target_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        target_point_topic, 
        rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default), rmw_qos_profile_default),
        std::bind(&DistanceColorizer::update_reference_pose, this, std::placeholders::_1)
    );

    // Create a timer wall for publisher
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(publisher_interval),
        [this]() { 
            if (this->point_cloud_->header.stamp.nanosec != 0.0)
            {
                this->publisher_->publish(*(this->colorised_point_cloud_.get())); 
            }
            else
            {
                RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *(this->get_clock()), 3000, 
                    "No point cloud data is received.");
            }
        }
    );
}

void DistanceColorizer::update_point_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud)
{
    this->point_cloud_ = cloud;
    this->colorise_point_cloud();
}

void DistanceColorizer::update_reference_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& pose)
{
    this->reference_x_ = pose->pose.pose.position.x;
    this->reference_y_ = pose->pose.pose.position.y;
    this->reference_z_ = pose->pose.pose.position.z;
    
    this->isReferenceInitialised_ = false;
    this->colorise_point_cloud();
}

void DistanceColorizer::colorise_point_cloud()
{
    auto output = this->colorised_point_cloud_;

    output->header = this->point_cloud_->header;
    output->width = this->point_cloud_->width;
    output->height = this->point_cloud_->height;
    output->is_dense = this->point_cloud_->is_dense;

    sensor_msgs::PointCloud2Modifier modifier(*output);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    const auto count = static_cast<size_t>(this->point_cloud_->width) * this->point_cloud_->height;
    if (count == 0) 
    {
        return;
    }

    try 
    {
        sensor_msgs::PointCloud2ConstIterator<float> in_x(*(this->point_cloud_), "x");
        sensor_msgs::PointCloud2ConstIterator<float> in_y(*(this->point_cloud_), "y");
        sensor_msgs::PointCloud2ConstIterator<float> in_z(*(this->point_cloud_), "z");

        sensor_msgs::PointCloud2Iterator<float> out_x(*output, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(*output, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(*output, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(*output, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(*output, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(*output, "b");

        for (size_t i = 0; i < count; i++) 
        {
            const float x = *in_x;
            const float y = *in_y;
            const float z = *in_z;

            *out_x = x; *out_y = y; *out_z = z;

            // Blue -> cyan -> green -> yellow -> red across [0, max_distance].
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) 
            {
                const double dx = static_cast<double>(x) - reference_x_;
                const double dy = static_cast<double>(y) - reference_y_;
                const double dz = static_cast<double>(z) - reference_z_;

                const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                const double t = std::clamp(distance / max_distance_, 0.0, 1.0);
                const double s = 4.0 * t;

                *out_r = static_cast<uint8_t>(255.0 * std::clamp(s - 2.0, 0.0, 1.0));
                *out_g = static_cast<uint8_t>(255.0 * std::clamp(std::min(s, 4.0 - s), 0.0, 1.0));
                *out_b = static_cast<uint8_t>(255.0 * std::clamp(2.0 - s, 0.0, 1.0));
            }
            else if (!this->isReferenceInitialised_)
            {
                *out_r = 255;
                *out_g = 255;
                *out_b = 255;
            }
            else {
                *out_r = 0;
                *out_g = 0;
                *out_b = 0;
            }

            ++in_x; ++in_y; ++in_z;
            ++out_x; ++out_y; ++out_z;
            ++out_r; ++out_g; ++out_b;
        }
    } 
    catch (const std::runtime_error& error) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *(this->get_clock()), 5000,
            "Cannot colorise input point cloud: %s", error.what()
        );
    }
}