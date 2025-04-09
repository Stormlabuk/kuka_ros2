#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class CartesianRawSender : public rclcpp::Node
{
public:
    CartesianRawSender()
    : Node("transmit_cartesian_raw")
    {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            rclcpp::shutdown();
            return;
        }

        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(30300);
        inet_pton(AF_INET, "172.31.1.147", &dest_addr_.sin_addr); // KUKA IP

        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "cartesian_command", 10,
            std::bind(&CartesianRawSender::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Raw Cartesian UDP sender initialized");
    }

    ~CartesianRawSender()
    {
        close(udp_socket_);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Assume: position in mm, orientation is Euler A (Z), B (Y), C (X) in degrees
        double x = msg->position.x;
        double y = msg->position.y;
        double z = msg->position.z;
        double A = msg->orientation.x;
        double B = msg->orientation.y;
        double C = msg->orientation.z;

        std::ostringstream oss;
        oss << "C:" << x << "," << y << "," << z << ","
            << A << "," << B << "," << C;

        std::string data_str = oss.str();

        ssize_t sent = sendto(udp_socket_, data_str.c_str(), data_str.size(), 0,
                              (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));

        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send raw Cartesian command");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent: %s", data_str.c_str());
        }
    }

    int udp_socket_;
    struct sockaddr_in dest_addr_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CartesianRawSender>());
    rclcpp::shutdown();
    return 0;
}
