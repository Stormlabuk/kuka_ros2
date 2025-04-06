#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>

// Correct socket headers
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring> // For memset

#include "kuka_messages/msg/moving_status.hpp" // Include custom message for robot moving status

class receive_robotNode : public rclcpp::Node {
public:
    receive_robotNode() : Node("receive_robot") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        moving_status_publisher_ = this->create_publisher<kuka_messages::msg::MovingStatus>("moving_status", 10);

        // Initialize JointState message with joint names
        joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        joint_state_msg_.position.resize(7, 0.0);
        joint_state_msg_.velocity.resize(7, 0.0);
        joint_state_msg_.effort.resize(7, 0.0);

        // Set up UDP socket for receiving
        setup_udp_socket();

        // Start the UDP receiving thread
        receiving_thread_ = std::thread(&receive_robotNode::receive_udp_data, this);
    }

    ~receive_robotNode() {
        // Signal the receiving thread to stop
        stop_flag_ = true;
        if (receiving_thread_.joinable()) {
            receiving_thread_.join();
        }

        if (sockfd_ >= 0) {
            close(sockfd_);
        }
    }

private:
    void setup_udp_socket() {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            throw std::runtime_error("Socket creation failed");
        }

        // Allow address reuse
        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket options");
            close(sockfd_);
            throw std::runtime_error("Setting socket options failed");
        }

        memset(&servaddr_, 0, sizeof(servaddr_));
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_port = htons(5005); // Port to listen on
        servaddr_.sin_addr.s_addr = inet_addr("172.31.1.150"); // Bind to specific IP address

        if (bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(sockfd_);
            throw std::runtime_error("Socket bind failed");
        }

        RCLCPP_INFO(this->get_logger(), "UDP socket set up and listening on 172.31.1.150:5005");
    }

    void receive_udp_data() {
        const int buffer_size = 1024;
        char buffer[buffer_size];
        struct sockaddr_in src_addr;
        socklen_t addrlen = sizeof(src_addr);

        while (!stop_flag_) {
            ssize_t n = recvfrom(sockfd_, buffer, buffer_size - 1, 0, (struct sockaddr *)&src_addr, &addrlen);
            if (n < 0) {
                if (stop_flag_) {
                    break; // Exit if stopping
                }
                RCLCPP_ERROR(this->get_logger(), "Failed to receive UDP data");
                continue;
            }

            buffer[n] = '\0'; // Null-terminate the received data
            std::string data(buffer);
            RCLCPP_DEBUG(this->get_logger(), "Received UDP data: %s", data.c_str());

            // Parse the received data and publish JointState and MovingStatus
            parse_and_publish(data);
        }
    }

    void parse_and_publish(const std::string &data) {
        // Expected format:
        // pos1,pos2,...,pos7;vel1,vel2,...,vel7;eff1,eff2,...,eff7;moving_status

        std::stringstream ss(data);
        std::string token;
        std::vector<std::string> parts;

        // Split the data by ';'
        while (std::getline(ss, token, ';')) {
            parts.push_back(token);
        }

        if (parts.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Received data does not have enough parts: %s", data.c_str());
            return;
        }

        // Parse positions
        if (!parse_string_to_vector(parts[0], joint_state_msg_.position)) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse positions");
            return;
        }

        // Parse velocities
        if (!parse_string_to_vector(parts[1], joint_state_msg_.velocity)) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse velocities");
            return;
        }

        // Parse efforts
        if (!parse_string_to_vector(parts[2], joint_state_msg_.effort)) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse efforts");
            return;
        }

        // Update the timestamp
        joint_state_msg_.header.stamp = this->now();

        // Publish the JointState message
        publisher_->publish(joint_state_msg_);

        // Parse the moving status
        std::string moving_status_str = parts[3];
        int moving_status_value = 0;
        try {
            moving_status_value = std::stoi(moving_status_str);
        } catch (const std::exception &e) {
            RCLCPP_WARN(this->get_logger(), "Invalid moving status value: %s", moving_status_str.c_str());
            return;
        }

        // Create and publish the MovingStatus message
        kuka_messages::msg::MovingStatus moving_status_msg;
        moving_status_msg.timestamp = this->now();
        moving_status_msg.moving_status = (moving_status_value != 0);
        moving_status_publisher_->publish(moving_status_msg);
    }

    bool parse_string_to_vector(const std::string &str, std::vector<double> &vec) {
        std::stringstream ss(str);
        std::string num_str;
        std::vector<double> temp_vec;

        while (std::getline(ss, num_str, ',')) {
            try {
                double num = std::stod(num_str);
                temp_vec.push_back(num);
            } catch (const std::invalid_argument &e) {
                RCLCPP_WARN(this->get_logger(), "Invalid number in data: %s", num_str.c_str());
                return false;
            } catch (const std::out_of_range &e) {
                RCLCPP_WARN(this->get_logger(), "Number out of range in data: %s", num_str.c_str());
                return false;
            }
        }

        if (temp_vec.size() != 7) {
            RCLCPP_WARN(this->get_logger(), "Expected 7 elements, got %zu", temp_vec.size());
            return false;
        }

        vec = temp_vec;
        return true;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;  // joint data publisher
    rclcpp::Publisher<kuka_messages::msg::MovingStatus>::SharedPtr moving_status_publisher_; // Moving status publiosher
    sensor_msgs::msg::JointState joint_state_msg_;
    int sockfd_ = -1;
    struct sockaddr_in servaddr_;

    std::thread receiving_thread_;
    bool stop_flag_ = false;
};

int main(int argc, char *argv[]) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    try {
        // Create and spin the node
        auto node = std::make_shared<receive_robotNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        std::cerr << "Exception in receive_robotNode: " << e.what() << std::endl;
    }

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
