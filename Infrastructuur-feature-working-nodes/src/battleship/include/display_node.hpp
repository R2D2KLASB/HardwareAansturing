#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <../include/SerialStream.h>
#include <bits/stdc++.h>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

using std::placeholders::_1;

class display_node : public rclcpp::Node {
public:
  display_node() : Node("display_node") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "game_info/intern/publish", 10,
        std::bind(&display_node::topic_callback, this, _1));
  }

  void decode_message(std::string s) ;

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) ;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
