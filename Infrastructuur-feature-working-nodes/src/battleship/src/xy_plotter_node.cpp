#include <bits/stdc++.h>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <../include/SerialStream.h>
#include "xy_plotter_node.hpp"

#define BAUDRATE 19200

/// @file

/**
 *   @brief Read the g-code and set it to xy coords.
 *   @param coords this is where the coords are saved in.
 *   @details Reads the g-codes and saves it line for line in a vector.
 *   @return returns new string without letters in it.
 */

void printer_node::topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    std::string message = msg->data;
    this->plotter_print(message);
}

std::vector<std::string> printer_node::getXYCoords(std::string gcodes) const
{
    std::vector<std::string> coords = {};
    std::string newline = "\n";

    size_t pos = 0;
    while ((pos = gcodes.find(newline)) != std::string::npos) {
        std::string line = gcodes.substr(0, pos);
        line.erase(std::remove_if(line.begin(), line.end(), []( unsigned char ch ){ if ((ch == 'G') || (ch == 'X' ) || (ch == 'Y' ) || (ch == 'M')) return true; else return false; } ), line.end());
        coords.push_back(line);
        gcodes.erase(0, pos + newline.length());
    }
    return coords;
}

int printer_node::plotter_print( std::string s ) const {
    // Serial object
    LibSerial::SerialStream Serial;

    std::ofstream myfile;
    myfile.open("gcode_received.txt");
    myfile << s;
    myfile.close();

    std::vector<std::string> coords;
    coords = getXYCoords(s);

    char input = 0;
    std::cout << "opening device" << "\n";
    Serial.Open("/dev/ttyACM0_PLOTTER");

    Serial.SetBaudRate( LibSerial::BaudRate::BAUD_19200);

    std::ofstream myfile2;
    myfile2.open("gcode_omgezet.txt");

    for (const std::string &gcode : coords) {
        for (char c: gcode){
            myfile2 << '_' << int(c) << '_';
        }
        myfile2 << "\n";

        Serial << gcode << '\n';

        while (input != 6) {
            Serial >> input;
        }
        input = 0;
    }
    Serial << "-1" << '\n';
    Serial.Close();
    myfile2.close();
    return 0;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<printer_node>());
    rclcpp::shutdown();
    return 0;
}