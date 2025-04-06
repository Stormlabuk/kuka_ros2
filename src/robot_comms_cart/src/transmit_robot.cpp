#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class CartesianUdpSender : public rclcpp::Node
{
public:
    CartesianUdpSender()
    : Node("transmit_cartesian")
    {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            rclcpp::shutdown();
            return;
        }

        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(30300); 
        inet_pton(AF_INET, "172.31.1.147", &dest_addr_.sin_addr); // Robot IP

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "cartesian_command", 10,
            std::bind(&CartesianUdpSender::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Cartesian UDP sender initialized");
    }

    ~CartesianUdpSender()
    {
        close(udp_socket_);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        double x = msg->position.x * 1000.0;  // Convert to mm
        double y = msg->position.y * 1000.0;
        double z = msg->position.z * 1000.0;

        // Convert quaternion to Euler angles in degrees
        double roll, pitch, yaw;
        quaternion_to_euler(msg->orientation.x, msg->orientation.y,
                            msg->orientation.z, msg->orientation.w,
                            roll, pitch, yaw);

        roll *= 180.0 / M_PI;
        pitch *= 180.0 / M_PI;
        yaw *= 180.0 / M_PI;

        std::ostringstream oss;
        oss << "C:" << x << "," << y << "," << z << ","
            << roll << "," << pitch << "," << yaw;

        std::string data_str = oss.str();

        ssize_t sent_bytes = sendto(udp_socket_, data_str.c_str(), data_str.size(), 0,
                                    (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));

        if (sent_bytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send Cartesian command");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent Cartesian command: %s", data_str.c_str());
        }
    }

    void quaternion_to_euler(double x, double y, double z, double w,
                             double& roll, double& pitch, double& yaw)
    {
        // Roll (X-axis rotation)
        double sinr_cosp = 2.0 * (w * x + y * z);
        double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (Y-axis rotation)
        double sinp = 2.0 * (w * y - z * x);
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp);
        else
            pitch = std::asin(sinp);

        // Yaw (Z-axis rotation)
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    int udp_socket_;
    struct sockaddr_in dest_addr_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianUdpSender>());
    rclcpp::shutdown();
    return 0;
}
