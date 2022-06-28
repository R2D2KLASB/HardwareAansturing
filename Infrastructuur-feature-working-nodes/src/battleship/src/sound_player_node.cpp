#include "sound_player_node.hpp"
#include <string>

void sound_player_node::play_game_sound(std::string& file_name)  {
  if (file_name == "WIN" or file_name == "LOSE") {
    system("killall --user $USER --ignore-case --signal INT ffplay");
  }

  std::string message = "ffplay -i ~/Sounds/" + file_name +".mp3 -autoexit -nodisp -af 'volume=0.3' &";
  system(message.c_str());
}

void sound_player_node::play_background_sound( std::string& file_name)  {
  std::string message = "ffplay -i ~/Sounds/" + file_name +".mp3 -autoexit -nodisp -af 'volume=0.1' &";
  system(message.c_str());
}

void sound_player_node::topic_callback( std_msgs::msg::String::SharedPtr msg)  {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  std::string message = msg->data;
  if ((message == "LOSE") or (message == "READY") or (message == "WIN") or
      (message == "FIRE")) {
    if (message == "READY")
      this->play_background_sound((std::string &)"BACKGROUND");
    else
      this->play_game_sound(message);
  } else {

    auto player = message.rbegin();
    auto keyWord = message.substr(0, message.find(' '));
    if (*player == '0') {
        if ((keyWord == "HIT") or (keyWord == "MISS") or (keyWord == "SUNK")){
            this->play_game_sound(keyWord);
        }
    }
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sound_player_node>());
  rclcpp::shutdown();
  return 0;
}