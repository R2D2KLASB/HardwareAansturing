#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class printer_node : public rclcpp::Node {
public:
    printer_node()
        : Node("xy_plotter") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
                "game_info/intern/gcode", 10, std::bind(&printer_node::topic_callback, this, _1));
    }
    int plotter_print(std::string s) const;
    std::vector<std::string> getXYCoords(std::string gcodes) const;

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;


};


