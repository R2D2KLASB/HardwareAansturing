#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class send_nodes : public rclcpp::Node
{
  public:
    send_nodes()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("game_info/intern/publish", 10);
      timer_ = this->create_wall_timer(
      100ms, std::bind(&send_nodes::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        std::getline(std::cin, message.data);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<send_nodes>());
  rclcpp::shutdown();
  return 0;
}