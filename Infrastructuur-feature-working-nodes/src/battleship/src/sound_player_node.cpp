#include "sound_player_node.hpp"
#include <string>

void sound_player_node::play_sound(std::string file_name) const{
    std::string message = "ffplay -i ~/Sounds/" + file_name + ".mp3 -autoexit -nodisp -af 'volume=0.3'";
    system(message.c_str());
}

void sound_player_node::topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    std::string message = msg->data;
    if((message == "HIT") or (message == "MISS") or (message == "WIN")or (message == "LOSE") or (message == "READY")or (message == "SUNK")or (message == "FIRE") ){
        if(message == "READY"){
            this->play_sound("BACKGROUND");
        }
        this->play_sound(message);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sound_player_node>());
    rclcpp::shutdown();
    return 0;
}