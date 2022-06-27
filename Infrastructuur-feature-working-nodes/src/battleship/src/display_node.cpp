#include "display_node.hpp"

void display_node::topic_callback(std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  std::string message = msg->data;
  this->decode_message(message);
}

void display_node::decode_message(std::string message) {
  LibSerial::SerialStream Serial;
  std::cout << "opening device\n";
  Serial.Open("/dev/ttyACM0");
  Serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);

  auto keyWord = message.substr(0, message.find(' '));
  auto player = message.rbegin();
  if (keyWord == "SUNK") {
    if (*player == '0') {
      Serial << '0';
    } else {
      Serial << '3';
    }
  } else if (keyWord == "HIT") {
    if (*player == '0') {
      Serial << '1';
    } else {
      Serial << '4';
    }
  } else if (keyWord == "MISS") {
    if (*player == '0') {
      Serial << '2';
    } else {
      Serial << '5';
    }
  } else if (keyWord == "TEXT") {
    auto text = message.substr(message.find(" ") + 1);
    Serial << '6';
    Serial << text.c_str();
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", text.c_str());
  }
  Serial.Close();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<display_node>());
  rclcpp::shutdown();
  return 0;
}