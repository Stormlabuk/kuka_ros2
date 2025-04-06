#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <cstring>      // For memset
#include <sys/socket.h> // For socket functions
#include <netinet/in.h> // For sockaddr_in
#include <arpa/inet.h>  // For inet_pton
#include <unistd.h>     // For close

using namespace std::chrono_literals;

class Heartbeat : public rclcpp::Node
{
public:
    Heartbeat()
        : Node("heartbeat"), heartbeat_value_(false)
    {
        // Initialize UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            rclcpp::shutdown();
            return;
        }

        // Configure the destination address (KUKA robot)
        memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(30301); // UDP_PORT_SEND

        if (inet_pton(AF_INET, "172.31.1.147", &dest_addr_.sin_addr) <= 0) { // UDP_IP
            RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address");
            close(udp_socket_);
            rclcpp::shutdown();
            return;
        }

        // Create a timer that fires every 50ms
        timer_ = this->create_wall_timer(50ms, std::bind(&Heartbeat::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Heartbeat Started");
    }

    ~Heartbeat()
    {
        close(udp_socket_);
    }

private:
    void timer_callback()
    {
        // Toggle the heartbeat value
        heartbeat_value_ = !heartbeat_value_;

        // Convert the boolean to string ("true" or "false")
        std::string heartbeat_str = heartbeat_value_ ? "true" : "false";

        // Send the heartbeat over UDP
        ssize_t bytes_sent = sendto(udp_socket_, heartbeat_str.c_str(), heartbeat_str.size(), 0,
                                    (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
        if (bytes_sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send heartbeat");
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Sent heartbeat: %s", heartbeat_str.c_str());
        }
    }

    int udp_socket_;
    struct sockaddr_in dest_addr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool heartbeat_value_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Heartbeat>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
