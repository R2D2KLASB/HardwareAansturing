#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "send_node.hpp"

send_node::send_node():
	Node("sendNode"), cound_(0){
	publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
	
	timer_ = this->create_wall_timer(std::chrono::milliseconds(5000), 				std::bind(&Publisher_node::timer_callback, this));
	}
	
void send_node::timer_callback(){
	std::string message ="";
    std::cin >> message;
	
	publisher_->publish(message);
}
