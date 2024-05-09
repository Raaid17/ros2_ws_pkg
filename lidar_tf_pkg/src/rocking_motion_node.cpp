#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

// Control table address
#define ADDR_AX_TORQUE_ENABLE 24
#define ADDR_AX_GOAL_POSITION 30
#define ADDR_AX_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION 1.0  // See the Dynamixel protocol version

// Default setting
#define DXL_ID 3  // Dynamixel ID: 1
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB1"  // Check which port is being used

// Dynamixel AX-12A specific settings
#define MAX_POSITION 1023  // Maximum position for a 300-degree rotation
#define MIN_POSITION 0     // Minimum position
#define ROCKING_AREA 341   // Rocking FOV
#define CENTER_POSITION (MAX_POSITION / 2) // Middle position

class ServoRockingNode : public rclcpp::Node {
public:
    ServoRockingNode() : Node("servo_rocking_node") {
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // Open port
        if (portHandler->openPort()) {
            RCLCPP_INFO(this->get_logger(), "Succeeded to open the port!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
            rclcpp::shutdown();
            return;
        }

        // Set port baud rate
        if (portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_INFO(this->get_logger(), "Succeeded to change the baudrate!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to change the baudrate!");
            rclcpp::shutdown();
            return;
        }

        // Enable Dynamixel Torque
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;  // Communication result
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for Dynamixel!");
        }

        // Start the rocking motion and servo feedback publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ServoRockingNode::rocking_motion, this));
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("servo_angle_feedback", 10);
    }

    ~ServoRockingNode() {
        // Disable Dynamixel Torque
        packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_AX_TORQUE_ENABLE, 0, nullptr);
        portHandler->closePort();
    }

private:
    void rocking_motion() {
        static bool direction = false; // false for increasing, true for decreasing
        static int position = CENTER_POSITION;
        const int step_size = 10; // Change this to adjust the speed of the rocking motion

        if (!direction) {
            position += step_size;
            if (position >= MAX_POSITION-ROCKING_AREA) {
                direction = !direction;
            }
        } else {
            position -= step_size;
            if (position <= MIN_POSITION+ROCKING_AREA) {
                direction = !direction;
            }
        }

        // Write goal position
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_AX_GOAL_POSITION, position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to write goal position!");
        }

        // Read present position
        uint16_t present_position = 0;
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_AX_PRESENT_POSITION, &present_position, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read present position!");
        } else {
            // Convert to angle and publish
            auto angle_msg = std_msgs::msg::Float64();
            angle_msg.data = static_cast<double>(present_position) / MAX_POSITION * 300; // Convert to degrees
            angle_publisher_->publish(angle_msg);
        }
    }

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoRockingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
