#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class LiDAR3DNode : public rclcpp::Node {
public:
    LiDAR3DNode() : Node("lidar_3d_node"), current_angle(0.0), rocking_started(false), rocking_complete(false), min_angle(std::numeric_limits<double>::max()), max_angle(std::numeric_limits<double>::lowest()) {
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&LiDAR3DNode::lidar_scan_callback, this, std::placeholders::_1));
        angle_sub = this->create_subscription<std_msgs::msg::Float64>("servo_angle_feedback", 10, std::bind(&LiDAR3DNode::servo_angle_callback, this, std::placeholders::_1));
        cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_cloud", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        if (!rocking_complete) {
            RCLCPP_WARN(this->get_logger(), "Rocking motion not complete, skipping point cloud publication");
            return;
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = scan_msg->header;
        cloud.header.frame_id = "lidar_frame";

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(scan_msg->ranges.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float distance = scan_msg->ranges[i];
            if (std::isinf(distance) || distance == 0.0) continue;

            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

            *iter_x = distance * cos(angle);
            *iter_y = distance * sin(angle);
            *iter_z = 0;
            *iter_intensity = scan_msg->intensities[i];

            ++iter_x; ++iter_y; ++iter_z; ++iter_intensity;
        }

        cloud_pub->publish(cloud);
    }

    void servo_angle_callback(const std_msgs::msg::Float64::SharedPtr angle_msg) {
        double angle_degrees = angle_msg->data - 60 - 90;
        current_angle = angle_degrees * M_PI / 180.0;

        if (!rocking_started) {
            rocking_started = true;
            min_angle = current_angle;
            max_angle = current_angle;
        } else {
            min_angle = std::min(min_angle, current_angle);
            max_angle = std::max(max_angle, current_angle);
            if (current_angle == min_angle || current_angle == max_angle) {
                rocking_complete = true;
            }
        }

        broadcast_transform(current_angle, this->get_clock()->now());
    }

    void broadcast_transform(double angle_rad, const rclcpp::Time &timestamp) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = timestamp;
        t.header.frame_id = "base_link";
        t.child_frame_id = "lidar_frame";

        double pivot_offset_y = 0.038;
        double lidar_height_z = 0.018;

        t.transform.translation.x = -pivot_offset_y * sin(angle_rad);
        t.transform.translation.y = -pivot_offset_y * cos(angle_rad);
        t.transform.translation.z = lidar_height_z;

        tf2::Quaternion q;
        q.setRPY(0, -angle_rad, 0);

        t.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double current_angle;
    bool rocking_started;
    bool rocking_complete;
    double min_angle;
    double max_angle;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDAR3DNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

