#include <Servo.h>
#include "XYPlotter.hpp"
#include "DataTypes.hpp"
#include "Definitions.hpp"

Servo pen;
XYPlotter Plot(ENABLE_PIN, DIR_X_PIN, STEP_X_PIN, DIR_Y_PIN, STEP_Y_PIN, {MAX_X, MAX_Y}, pen, 9, 10);

//void setup() {
//  Plot.home();
//  liefde();
//}
//
//void loop() {
//  if (Serial.available()>0){
//    String input = Serial.readStringUntil('\n');
//    input.split(' ');
//  }
//}



void print_draw(Coordinate xy, bool pen) {
  SerialUSB.print("DRAW x: ");
  SerialUSB.print(xy.x);
  SerialUSB.print(" Y: ");
  SerialUSB.print(xy.y);
  SerialUSB.print(" bool pen : ");
  SerialUSB.println(pen);
}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // start serial port at 19200 bps:
  Serial.begin(19200);
  SerialUSB.begin(19200);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }
  //  while (!SerialUSB) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }
  //
  //  SerialUSB.println("setup done");
  //  Plot.init();
}

void SerialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void loop() {
  // if we get a valid byte, read analog ins:
  while (Serial.available() == 0) {
  }
  int g_code = Serial.parseInt();
  Coordinate XY;

  switch (g_code) {
    //G00 Z-axis up and move to location (x, y)
    case 0:
      XY.x = Serial.parseInt();
      XY.y = Serial.parseInt();
      Plot.draw(XY, 0);
      print_draw(XY, 0);
      break;

    //G01 Z-axis down and move to location (draw line) (x, y)
    case 1:
      XY.x = Serial.parseInt();
      XY.y = Serial.parseInt();
      Plot.draw(XY, 1);
      break;

    //G28 home
    case 28:
      Plot.home();
      break;

    //G02 Arc movement clockwise arc (x, y, i, j, e)
    case 2:
      break;

    //G03 Arc movement clockwise arc (x, y, i, j, e)
    case 3:
      break;

    default:
      break;
  }
  //  Serial.write('"');
  //  Serial.write(i);
  //  Serial.write('"');
  //  Serial.write(": Ready\n");
  SerialFlush();
  Serial.write(6);
}
