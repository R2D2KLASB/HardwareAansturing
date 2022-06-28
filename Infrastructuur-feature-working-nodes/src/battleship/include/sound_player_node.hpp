#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class sound_player_node : public rclcpp::Node {
public:
    sound_player_node() : Node("speaker_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "game_info/intern/publish", 10, std::bind(&sound_player_node::topic_callback, this, _1));
    }
    void play_game_sound(const std::string& file_name) const;
    void play_background_sound(const std::string& file_name) const;

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
