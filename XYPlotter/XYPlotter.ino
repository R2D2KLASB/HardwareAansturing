#include <Servo.h>
#include "XYplotter.hpp"
#include "DataTypes.hpp"
#include "Definitions.hpp"
#include "queue.hpp"

Servo pen;
XYPlotter plot(ENABLE_PIN, DIR_X_PIN, STEP_X_PIN, DIR_Y_PIN, STEP_Y_PIN, {MAX_X, MAX_Y}, pen, 9, 10);

Queue queue;

//void print_draw(Coordinate xy, int mode) {
//  SerialUSB.print("DRAW x: ");
//  SerialUSB.print(xy.x);
//  SerialUSB.print(" Y: ");
//  SerialUSB.print(xy.y);
//  SerialUSB.print(" int mode : ");
//  SerialUSB.println(mode);
//}


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // start serial port at 19200 bps:
  Serial.begin(19200);
  //  SerialUSB.begin(19200);
  plot.init();
}

void SerialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void loop() {
  queue.init_queue();
  Gcode readCode;
  readCode.gcode = 0;
  // if we get a valid byte, read analog ins:
  while (readCode.gcode != -1) {

    while (Serial.available() == 0) {}

    readCode.gcode = Serial.parseInt();
    if (readCode.gcode != -1) {
      switch (readCode.gcode) {
        //G00 Z-axis up and move to location (x, y)
        case 0:
          readCode.location.x = Serial.parseInt();
          readCode.location.y = Serial.parseInt();
          break;

        //G01 Z-axis down and move to location (draw line) (x, y)
        case 1:
          readCode.location.x = Serial.parseInt();
          readCode.location.y = Serial.parseInt();
          break;
        case 28:
          readCode.location.x = 0;
          readCode.location.y = 0;
          break;
        default:
          break;
      }
      queue.append(readCode);
      if (queue.isFull()) {
        break;
      }
      else {
        Serial.write(6);
      }
    }

  }
  Gcode writeCode;
  while (!queue.pop(writeCode)) {
    switch (writeCode.gcode) {
      //G00 Z-axis up and move to location (x, y)
      case 0:
        plot.draw(writeCode.location, 0);
        //        print_draw(writeCode.location, 0);
        break;

      //G01 Z-axis down and move to location (draw line) (x, y)
      case 1:
        plot.draw(writeCode.location, 1);
        //        print_draw(writeCode.location, 1);
        break;

      //G28 home
      case 28:
        plot.home();
        break;

      default:
        break;
    }
  }
}
