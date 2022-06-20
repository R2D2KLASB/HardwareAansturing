/*#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class sendNode : public rclcpp::Node{
private:
	void timer_callback(){
		auto message = std_msgs::msg::String();
		message.data = "test";
		publisher_->publish(message);
	}
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;
	
public:
	sendNode():
		Node("Sender Node"), count_(0){
		publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
		timer_ = this->create_wall_timer(
		500ms, std::bind(&sendNode::timer_callback, this));
}*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class send_node : public rclcpp::Node{
private:
	void timer_callback();
	
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
	size_t count_;

public:
	send_node();

};
