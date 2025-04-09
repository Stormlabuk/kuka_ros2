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

#include "kuka_messages/msg/moving_status.hpp"
#include "kuka_messages/msg/cartesian_pose.hpp"  // Correct message include for Cartesian pose

class receive_robotNode : public rclcpp::Node {
public:
    receive_robotNode() : Node("receive_robot") {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        moving_status_publisher_ = this->create_publisher<kuka_messages::msg::MovingStatus>("moving_status", 10);
        cartesian_pose_publisher_ = this->create_publisher<kuka_messages::msg::CartesianPose>("cartesian_pose", 10);

        joint_state_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
        joint_state_msg_.position.resize(7, 0.0);
        joint_state_msg_.velocity.resize(7, 0.0);
        joint_state_msg_.effort.resize(7, 0.0);

        setup_udp_socket();
        receiving_thread_ = std::thread(&receive_robotNode::receive_udp_data, this);
    }

    ~receive_robotNode() {
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

        int opt = 1;
        if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set socket options");
            close(sockfd_);
            throw std::runtime_error("Setting socket options failed");
        }

        memset(&servaddr_, 0, sizeof(servaddr_));
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_port = htons(5005);
        servaddr_.sin_addr.s_addr = inet_addr("172.31.1.150");

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
                if (stop_flag_) break;
                RCLCPP_ERROR(this->get_logger(), "Failed to receive UDP data");
                continue;
            }

            buffer[n] = '\0';
            std::string data(buffer);
            RCLCPP_DEBUG(this->get_logger(), "Received UDP data: %s", data.c_str());

            parse_and_publish(data);
        }
    }

    void parse_and_publish(const std::string &data) {
        std::stringstream ss(data);
        std::string token;
        std::vector<std::string> parts;

        while (std::getline(ss, token, ';')) {
            parts.push_back(token);
        }

        if (parts.size() < 5) {
            RCLCPP_WARN(this->get_logger(), "Received data does not have enough parts: %s", data.c_str());
            return;
        }

        if (!parse_string_to_vector(parts[0], joint_state_msg_.position) ||
            !parse_string_to_vector(parts[1], joint_state_msg_.velocity) ||
            !parse_string_to_vector(parts[2], joint_state_msg_.effort)) {
            RCLCPP_WARN(this->get_logger(), "Failed to parse joint data");
            return;
        }

        joint_state_msg_.header.stamp = this->now();
        publisher_->publish(joint_state_msg_);

        try {
            int moving_status_value = std::stoi(parts[3]);
            kuka_messages::msg::MovingStatus moving_status_msg;
            moving_status_msg.timestamp = this->now();
            moving_status_msg.moving_status = (moving_status_value != 0);
            moving_status_publisher_->publish(moving_status_msg);
        } catch (...) {
            RCLCPP_WARN(this->get_logger(), "Invalid moving status value");
            return;
        }

        std::stringstream pose_ss(parts[4]);
        std::vector<double> pose_vals;
        while (std::getline(pose_ss, token, ',')) {
            try {
                pose_vals.push_back(std::stod(token));
            } catch (...) {
                RCLCPP_WARN(this->get_logger(), "Invalid Cartesian pose value");
                return;
            }
        }

        if (pose_vals.size() != 6) {
            RCLCPP_WARN(this->get_logger(), "Expected 6 Cartesian values, got %zu", pose_vals.size());
            return;
        }

        kuka_messages::msg::CartesianPose pose_msg;
        pose_msg.timestamp = this->now();
        pose_msg.x = pose_vals[0];
        pose_msg.y = pose_vals[1];
        pose_msg.z = pose_vals[2];
        pose_msg.alpha = pose_vals[3];
        pose_msg.beta = pose_vals[4];
        pose_msg.gamma = pose_vals[5];

        cartesian_pose_publisher_->publish(pose_msg);
    }

    bool parse_string_to_vector(const std::string &str, std::vector<double> &vec) {
        std::stringstream ss(str);
        std::string num_str;
        std::vector<double> temp_vec;

        while (std::getline(ss, num_str, ',')) {
            try {
                double num = std::stod(num_str);
                temp_vec.push_back(num);
            } catch (...) {
                return false;
            }
        }

        if (temp_vec.size() != 7) {
            return false;
        }

        vec = temp_vec;
        return true;
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Publisher<kuka_messages::msg::MovingStatus>::SharedPtr moving_status_publisher_;
    rclcpp::Publisher<kuka_messages::msg::CartesianPose>::SharedPtr cartesian_pose_publisher_;

    sensor_msgs::msg::JointState joint_state_msg_;
    int sockfd_ = -1;
    struct sockaddr_in servaddr_;

    std::thread receiving_thread_;
    bool stop_flag_ = false;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<receive_robotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
