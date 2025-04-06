#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <queue>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class KukaUdpSender : public rclcpp::Node
{
public:
  KukaUdpSender()
  : Node("transmit_robot")
  {
    // Create UDP socket
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create UDP socket");
      rclcpp::shutdown();
    }

    // Destination address
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(30300);
    inet_pton(AF_INET, "172.31.1.147", &dest_addr_.sin_addr);

    // Subscription to Float64MultiArray
    sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "control_array", 10,
      std::bind(&KukaUdpSender::array_callback, this, std::placeholders::_1));

    // Timer to transmit commands every 1 ms
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&KukaUdpSender::process_queue, this));
  }

  ~KukaUdpSender()
  {
    close(udp_socket_);
  }

private:
  void array_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // Expect rows of 7
    const size_t row_size = 7;
    if (msg->data.size() % row_size != 0) {
      RCLCPP_ERROR(get_logger(), "Received data not multiple of 7.");
      return;
    }
    // Push each row into the queue
    std::lock_guard<std::mutex> lock(queue_mutex_);
    for (size_t i = 0; i < msg->data.size(); i += row_size) {
      std::vector<double> row(row_size);
      for (size_t j = 0; j < row_size; ++j) {
        row[j] = msg->data[i + j];
      }
      command_queue_.push(row);
    }
  }

  void process_queue()
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!command_queue_.empty()) {
      auto row = command_queue_.front();
      command_queue_.pop();
      send_udp(row);
    }
  }

  void send_udp(const std::vector<double>& joint_positions)
  {
    std::ostringstream oss;
    for (size_t i = 0; i < joint_positions.size(); ++i) {
      oss << joint_positions[i] << ((i < joint_positions.size() - 1) ? "," : "");
    }
    std::string data_str = oss.str();
    ssize_t sent_bytes = sendto(
      udp_socket_, data_str.c_str(), data_str.size(), 0,
      (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));
    if (sent_bytes < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to send UDP message");
    } else {
      RCLCPP_INFO(get_logger(), "Sent: %s", data_str.c_str());
    }
  }

  int udp_socket_;
  struct sockaddr_in dest_addr_;

  std::queue<std::vector<double>> command_queue_;
  std::mutex queue_mutex_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KukaUdpSender>());
  rclcpp::shutdown();
  return 0;
}
