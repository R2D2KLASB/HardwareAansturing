//#include "serialib-master/lib/serialib.h"
//#include <iostream>
//#include <unistd.h>
//#include <cstdio>
//
//#define SERIAL_PORT "/dev/ttyACM1"
//
//int main()
//{
//  // Serial object
//  serialib serial;
//
//  // Connection to serial port
//  char errorOpening = serial.openDevice(SERIAL_PORT, 9600);
//
//  // If connection fails, return the error code otherwise, display a success message
//  if (errorOpening!=1) {
//    #define SERIAL_PORT "/dev/ttyACM0"  //sometimes rpi switched port so change it
//  }
//  printf ("Successful connection to %s\n",SERIAL_PORT);
//
//  while(1){
//    serial.writeString("testing connection");
//    usleep(3000000);
//  }
//}
