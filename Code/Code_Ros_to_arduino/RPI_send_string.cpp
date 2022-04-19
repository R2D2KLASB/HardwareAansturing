#include "serialib-master/lib/serialib.h"
#include <bits/stdc++.h>
#include <iostream>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyACM0"

using namespace std;

/// @file

/**
 *   @brief Read the g-code and set it to xy coords.
 *   @param coords this is where the coords are saved in.
 *   @details Reads the g-codes and saves it line for line in a vector.
 *   @return returns new string without letters in it.
 */
//vector<string> getXYCoords(string gcodes) {
//  vector<string> coords = {};
//  string space_delimiter = " ";
//  string newline = "\n";
//
//  size_t pos = 0;
//  while ((pos = gcodes.find(newline)) != string::npos) {
//    string line = gcodes.substr(0, pos);
//    line.erase(std::remove_if(line.begin(), line.end(),
//                               [](unsigned char ch) {
//                                 return ((ch == 'G') || (ch == 'X') ||
//                                         (ch == 'Y')); }),line.end());
//    coords.push_back(line);
//    gcodes.erase(0, pos + newline.length());
//  }
//
//  return coords;
//}

string getXYCoord( string gcode){
  gcode.erase(std::remove_if(gcode.begin(), gcode.end(),
                            [](unsigned char ch) {
                               return ((ch == 'G') || (ch == 'X') ||
                                       (ch == 'Y')); }),gcode.end());
  return gcode;
}

int main() {
  // Serial object
  serialib serial;
  char errorOpening = serial.openDevice(SERIAL_PORT, 9600);
  if (errorOpening!=1) {
        #define SERIAL_PORT "/dev/ttyACM0"  //sometimes rpi switched port so
        // change it
  }

  cout << "Successful connection to " << SERIAL_PORT << endl;

  //test string
  string gcode = "G00 X0.000000 Y2.857143\nG01 X1.428571 Y4.571429\nG01 "
                 "X3.571429 Y7.428571\nG01 X4.285714 Y8.571428";


  string coords = getXYCoord(gcode);

  //for debuggin
  cout << coords << endl;

  serial.writeString(coords.c_str());
}
