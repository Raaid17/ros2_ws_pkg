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
    LiDAR3DNode() : Node("lidar_3d_node") {
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LiDAR3DNode::lidar_scan_callback, this, std::placeholders::_1));
        angle_sub = this->create_subscription<std_msgs::msg::Float64>(
            "servo_angle_feedback", 10, std::bind(&LiDAR3DNode::servo_angle_callback, this, std::placeholders::_1));
        cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_cloud", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        current_angle = 0.0;
    }

private:
    void lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = scan_msg->header;
    cloud.header.frame_id = "lidar_frame";  // This ensures the point cloud is associated with the lidar_frame

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, 
                                     "y", 1, sensor_msgs::msg::PointField::FLOAT32, 
                                     "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(scan_msg->ranges.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float distance = scan_msg->ranges[i];
        if (std::isinf(distance) || distance == 0.0) continue;  // Ignore infinite or zero distances

        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        *iter_x = distance * cos(angle);  // x coordinate in the lidar_frame
        *iter_y = distance * sin(angle);  // y coordinate in the lidar_frame
        *iter_z = 0;                      // z coordinate is zero as this is a 2D scan

        ++iter_x; ++iter_y; ++iter_z;
    }

    cloud_pub->publish(cloud);  // Publish the modified point cloud
}


    void servo_angle_callback(const std_msgs::msg::Float64::SharedPtr angle_msg) 
    {
    // Convert angle_msg->data from servo's range to actual angle in radians
    // Servo range: 0-300, Offset: -60 degrees at 150 servo value
    double angle_degrees = (angle_msg->data - 60 -90);
    current_angle = angle_degrees * M_PI / 180.0; // Convert degrees to radians

    broadcast_transform(current_angle);
    }


	void broadcast_transform(double angle_rad) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "lidar_frame";

    // Define the pivot point's offset from the base_link origin
    double pivot_offset_y = 0.038; // Horizontal offset of the pivot along the base_link's negative y-axis
    double lidar_height_z = 0.018; // Height of the LiDAR from the base of the base_link

    // Calculate new positions based on rotation around the negative Y-axis
    t.transform.translation.x = -pivot_offset_y * sin(angle_rad); // Moving along a circular path in XZ-plane, mirrored across Y-axis
    t.transform.translation.y = -pivot_offset_y * cos(angle_rad); // Negative to invert the direction along the y-axis
    t.transform.translation.z = lidar_height_z; // Height remains constant

    // Initial rotation around the negative Y-axis
    tf2::Quaternion q;
    q.setRPY(0, -angle_rad, 0); // Rotation around the negative Y-axis

    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);
}






    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    double current_angle;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LiDAR3DNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

