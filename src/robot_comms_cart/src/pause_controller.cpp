#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

class PauseController : public rclcpp::Node {
public:
    PauseController()
        : Node("pause_controller"), pause_status_(false), kuka_ip_("172.31.1.147"), kuka_port_(30303)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("pause_status", 10);
        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_pause",
            std::bind(&PauseController::handle_service, this,
                      std::placeholders::_1, std::placeholders::_2));

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&PauseController::publish_status, this));

        RCLCPP_INFO(this->get_logger(), "PauseController node ready");
    }

private:
    void handle_service(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        pause_status_ = request->data;
        std::string command = pause_status_ ? "pause" : "resume";

        bool success = send_udp(command);

        response->success = success;
        response->message = success ?
            ("Sent '" + command + "' to KUKA") :
            ("Failed to send '" + command + "'");

        if (success) {
            RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        }
    }

    bool send_udp(const std::string &msg)
    {
        int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            return false;
        }

        sockaddr_in servaddr{};
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(kuka_port_);
        servaddr.sin_addr.s_addr = inet_addr(kuka_ip_.c_str());

        ssize_t sent = sendto(sockfd, msg.c_str(), msg.size(), 0,
                              reinterpret_cast<sockaddr*>(&servaddr), sizeof(servaddr));

        close(sockfd);
        return sent >= 0;
    }

    void publish_status()
    {
        std_msgs::msg::Bool msg;
        msg.data = pause_status_;
        publisher_->publish(msg);
    }

    bool pause_status_;
    std::string kuka_ip_;
    int kuka_port_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PauseController>());
    rclcpp::shutdown();
    return 0;
}
