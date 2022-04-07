//#include "serialib-master/lib/serialib.h"
//#include <iostream>
//#include <unistd.h>
//#include <cstdio>
//
//#define SERIAL_PORT "/dev/ttyACM1"
//
// int main()
//{
//  // Serial object
//  serialib serial;
//
//  // Connection to serial port
//  char errorOpening = serial.openDevice(SERIAL_PORT, 9600);
//
//  // If connection fails, return the error code otherwise, display a success
//  message if (errorOpening!=1) {
//    #define SERIAL_PORT "/dev/ttyACM0"  //sometimes rpi switched port so
//    change it
//  }
//  printf ("Successful connection to %s\n",SERIAL_PORT);
//
//  while(1){
//    serial.writeString("testing connection");
//    usleep(3000000);
//  }
//}


#include "serialib-master/lib/serialib.h"
#include <bits/stdc++.h>
#include <cstdio>
#include <iostream>
#include <unistd.h>

#define SERIAL_PORT "/dev/ttyACM0"

using namespace std;

/// @file

/**
 *   @brief Read the g-code and set it to xy coords.
 *   @param coords this is where the coords are saved in.
 *   @details Reads the g-codes and saves it line for line in a vector.
 *   @return returns new array with shape: g-code number X Y.
 */

vector<string> getXYCoords(string gcodes) {
  vector<string> coords = {};
  string space_delimiter = " ";
  string newline = "\n";

  size_t pos = 0;
  while ((pos = gcodes.find(newline)) != string::npos) {
    string line = gcodes.substr(0, pos);
    line.erase(std::remove_if(line.begin(), line.end(),
                               [](unsigned char ch) {
                                 return ((ch == 'G') || (ch == 'X') ||
                                         (ch == 'Y')); }),line.end());
    coords.push_back(line);
    gcodes.erase(0, pos + newline.length());
  }

  return coords;
}

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

  string gcode = "G00 X0.000000 Y2.857143\nG01 X1.428571 Y4.571429\nG01 "
                 "X3.571429 Y7.428571\nG01 X4.285714 Y8.571428";

  string coords = getXYCoord(gcode);
  cout << coords << endl;
  serial.writeString(coords.c_str());
}
