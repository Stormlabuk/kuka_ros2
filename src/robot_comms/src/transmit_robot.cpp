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
#include "kuka_messages/msg/control_message.hpp"
#include "kuka_messages/msg/moving_status.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

class KukaUdpSender : public rclcpp::Node
{
public:
  KukaUdpSender()
  : Node("transmit_robot"), is_moving_(true)
  {
    // Initialize UDP socket
    udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_socket_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
      rclcpp::shutdown();
    }

    // Configure the destination address (KUKA robot)
    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(30300); // UDP_PORT_SEND
    inet_pton(AF_INET, "172.31.1.147", &dest_addr_.sin_addr); // UDP_IP

    // Create a subscription to the ControlMessage topic with a queue size of 200
    control_subscription_ = this->create_subscription<kuka_messages::msg::ControlMessage>(
      "control_command", 200,
      std::bind(&KukaUdpSender::control_callback, this, std::placeholders::_1));

    // Create a subscription to the MovingStatus topic with a queue size of 200
    moving_status_subscription_ = this->create_subscription<kuka_messages::msg::MovingStatus>(
      "moving_status", 200,
      std::bind(&KukaUdpSender::moving_status_callback, this, std::placeholders::_1));

    // Timer to process the queue
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&KukaUdpSender::process_queue, this));

    // Initialize last_command_time_ to the current time minus 7 seconds
    last_command_time_ = std::chrono::steady_clock::now() - std::chrono::milliseconds(1);
  }

  ~KukaUdpSender()
  {
    close(udp_socket_);
  }

private:
  void control_callback(const kuka_messages::msg::ControlMessage::SharedPtr msg)
  {
    if (msg->commands.size() != 7) {
      RCLCPP_ERROR(this->get_logger(), "Received command with incorrect number of joint positions.");
      return;
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (command_queue_.size() >= MAX_QUEUE_SIZE) {
      RCLCPP_WARN(this->get_logger(), "Command queue is full (size: %lu). Discarding new command.", command_queue_.size());
      return;
    }

    command_queue_.push(msg->commands);
    RCLCPP_INFO(this->get_logger(), "Command added to queue. Queue size: %lu", command_queue_.size());
  }

  void moving_status_callback(const kuka_messages::msg::MovingStatus::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    is_moving_ = msg->moving_status;
  }

  void process_queue()
  {
    std::lock_guard<std::mutex> status_lock(status_mutex_);
    auto now = std::chrono::steady_clock::now();

    // Check if robot is not moving and 7 seconds have passed since the last command was sent
    if (!is_moving_ && (now - last_command_time_) >= std::chrono::milliseconds(1))
    {
      std::lock_guard<std::mutex> queue_lock(queue_mutex_);
      if (!command_queue_.empty())
      {
        // Dequeue and send the next command
        last_sent_command_ = command_queue_.front();
        command_queue_.pop();
        send_joint_command(last_sent_command_);
      }
    }
  }

  void send_joint_command(const std::vector<double>& joint_positions)
  {
    // Convert joint positions to comma-separated string
    std::ostringstream oss;
    for (size_t i = 0; i < joint_positions.size(); ++i) {
      oss << joint_positions[i];
      if (i != joint_positions.size() - 1) {
        oss << ",";
      }
    }
    std::string data_str = oss.str();

    // Send data over UDP
    ssize_t sent_bytes = sendto(udp_socket_, data_str.c_str(), data_str.size(), 0,
                                (struct sockaddr*)&dest_addr_, sizeof(dest_addr_));

    if (sent_bytes < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send UDP message");
    } else {
      RCLCPP_INFO(this->get_logger(), "Sent joint command: %s", data_str.c_str());
    }

    // Update the last command time
    last_command_time_ = std::chrono::steady_clock::now();
  }

  int udp_socket_;
  struct sockaddr_in dest_addr_;

  std::queue<std::vector<double>> command_queue_;
  std::vector<double> last_sent_command_;

  bool is_moving_;
  std::mutex queue_mutex_;
  std::mutex status_mutex_;

  // Add the last command time variable
  std::chrono::steady_clock::time_point last_command_time_;

  // Define the maximum queue size
  static constexpr size_t MAX_QUEUE_SIZE = 200;

  rclcpp::Subscription<kuka_messages::msg::ControlMessage>::SharedPtr control_subscription_;
  rclcpp::Subscription<kuka_messages::msg::MovingStatus>::SharedPtr moving_status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KukaUdpSender>());
  rclcpp::shutdown();
  return 0;
}

