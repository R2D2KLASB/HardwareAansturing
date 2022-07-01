#include "xy_plotter_node.hpp"
#include <../include/SerialPort.h>
#include <../include/SerialStreamBuf.h>
#include <bits/stdc++.h>
#include <fstream>
#include <iostream>
#include <unistd.h>

#define BAUDRATE 9600
#define BUFFER_SIZE 1

/// @file

/**
 *   @brief Read the g-code and set it to xy coords.
 *   @param coords this is where the coords are saved in.
 *   @details Reads the g-codes and saves it line for line in a vector.
 *   @return returns new string without letters in it.
 */

void printer_node::topic_callback(
        const std_msgs::msg::String::SharedPtr msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    std::string message = msg->data;
    this->plotter_print(message);
}

std::vector<std::string> printer_node::getXYCoords(std::string gcodes) const {
    std::vector<std::string> coords = {};
    std::string newline = "\n";

    size_t pos = 0;
    while ((pos = gcodes.find(newline)) != std::string::npos) {
        std::string line = gcodes.substr(0, pos);
        line.erase(std::remove_if(line.begin(), line.end(),
                                  [](unsigned char ch) {
                                      if ((ch == 'G') || (ch == 'X') || (ch == 'Y') ||
                                          (ch == 'M') || (ch == 'R') || (ch == 'C') ||
                                          (ch == 'W') || (ch == 'L') || (ch == 'P'))
                                          return true;
                                      else
                                          return false;
                                  }),
                   line.end());
        coords.push_back(line);
        gcodes.erase(0, pos + newline.length());
    }
    return coords;
}

int printer_node::plotter_print(std::string s) const {
    // Serial object
    LibSerial::SerialPort Serial;

    std::ofstream myfile;
    myfile.open("gcode_received.txt");
    myfile << s;
    myfile.close();

    std::vector<std::string> coords;
    if (!Serial.IsOpen()) {
        RCLCPP_INFO(this->get_logger(), "serial not open, opening..");
        Serial.Open("/dev/ttyACM0_PLOTTER");
        Serial.SetBaudRate(LibSerial::BaudRate::BAUD_9600);
//    Serial.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
//    Serial.SetParity(LibSerial::Parity::PARITY_EVEN);
//    Serial.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_SOFTWARE);
//    Serial.SetStopBits(LibSerial::StopBits::STOP_BITS_DEFAULT);
        RCLCPP_INFO(this->get_logger(), "opened serial");
        Serial.FlushIOBuffers();
        RCLCPP_INFO(this->get_logger(), "flushed");
    } else {
        RCLCPP_INFO(this->get_logger(), "serial was already open");
    }
    int timeout = 9999999;

    char begin_input = 'n';
    while (begin_input != 'A') {
        Serial.ReadByte(begin_input, timeout);
    }
    begin_input = 'n';

    coords = getXYCoords(s);
    char input = 'n';
    char input_flush = 'n';
    // handshake for buffer clear
    Serial.Write("-2");
//    Serial.Write("_");
    RCLCPP_INFO(this->get_logger(), "Written -2");
    while (input_flush != 'A') {
        Serial.ReadByte(input_flush, timeout);
	RCLCPP_INFO(this->get_logger(), "character read: '%c'", input_flush);
    }
    input_flush = 'n';
    RCLCPP_INFO(this->get_logger(), "Start writing gcode");
    for (std::string &gcode : coords) {
//        gcode += " ";
//        std::string word = "";
        std::replace(gcode.begin(), gcode.end(), ' ', '_');
//        std::for_each(gcode.begin(), gcode.end(), [&Serial, this, &word](char x) {
//            if (x == ' ') {
//                RCLCPP_INFO(this->get_logger(), "word: '%s'", word.c_str());
//                RCLCPP_INFO(this->get_logger(), "char: '%c'", x);
//
//                Serial.Write((word + '\n'));
//                Serial.Write("_");
//                word = "";
//            } else {
//                word += x;
//            }
//        });
        Serial.Write(gcode);
        RCLCPP_INFO(this->get_logger(), "sent gcode line: '%s'", gcode.c_str());
        while (input != 'A') {
            Serial.ReadByte(input, timeout);
        }
        input = 'n';
        RCLCPP_INFO(this->get_logger(), "Exit input wait");
    }
    Serial.Write("-1\n");
    char input_second = 'n';
    while (input_second != 'A') {
        Serial.ReadByte(input_second, timeout);
    }
    input_second = 'n';
    RCLCPP_INFO(this->get_logger(), "sent -1");
    Serial.Close();
    RCLCPP_INFO(this->get_logger(), "serial closed");
    return 0;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<printer_node>());
    rclcpp::shutdown();
    return 0;
}
