#include "display_node.hpp"

void display_node::topic_callback(const std_msgs::msg::String::SharedPtr msg) {
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
      Serial << "1\n";
    }  else if (*player == '1')  {
      Serial << "4\n";
    }
  } else if (keyWord == "HIT") {
    if (*player == '0') {
      Serial << "2\n";
    }  else if (*player == '1')  {
      Serial << "5\n";
    }
  } else if (keyWord == "MISS") {
    if (*player == '0') {
      Serial << "3\n";
    } else if (*player == '1') {
      Serial << "6\n";
    }
  } else if (keyWord == "TEXT") {
    auto text = message.substr(message.find(' ')+1);
    Serial << '7' << text.c_str();
  }
  else if ((keyWord == "WIN") or (keyWord == "LOSE")) {
    Serial << "8\n";
  }
  Serial.Close();
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<display_node>());
  rclcpp::shutdown();
  return 0;
}