#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

using std::placeholders::_1;

class sound_player_node : public rclcpp::Node {
public:
  sound_player_node() : Node("speaker_node") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "game_info/intern/publish", 10,
        std::bind(&sound_player_node::topic_callback, this, _1));
  }
  void play_game_sound( std::string &file_name) ;
  void play_background_sound( std::string &file_name) ;

private:
  void topic_callback(std_msgs::msg::String::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
