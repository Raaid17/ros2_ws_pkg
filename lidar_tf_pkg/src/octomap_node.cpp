#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

class OctoMapNode : public rclcpp::Node {
public:
    OctoMapNode() : Node("octomap_node"), octree(0.1) {
        subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "lidar_cloud", 10, std::bind(&OctoMapNode::pointCloudCallback, this, std::placeholders::_1));
        publisher = this->create_publisher<octomap_msgs::msg::Octomap>("octomap", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        octomap::Pointcloud cloud;
        octomap::pointCloud2ToOctomap(*msg, cloud);
        octree.insertPointCloud(cloud, octomap::point3d(0, 0, 0));
        
        octomap_msgs::msg::Octomap octomap_msg;
        octomap_msgs::binaryMapToMsg(octree, octomap_msg);
        publisher->publish(octomap_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr publisher;
    octomap::OcTree octree;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OctoMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

