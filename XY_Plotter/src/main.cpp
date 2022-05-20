#include <Servo.h>
#include "XYPlotter.hpp"
#include "DataTypes.hpp"
#include "Definitions.hpp"
#include "queue.hpp"

// #define DEBUG

Servo pen;
XYPlotter plot({MAX_X, MAX_Y}, pen, ENABLE_PIN, DIR_X_PIN, STEP_X_PIN,MICROSWITCHX, DIR_Y_PIN, STEP_Y_PIN, MICROSWITCHY);

Queue queue;

#ifdef DEBUG
void print_draw(Coordinate xy, int mode) {
 SerialUSB.print("DRAW x: ");
 SerialUSB.print(xy.x);
 SerialUSB.print(" Y: ");
 SerialUSB.print(xy.y);
 SerialUSB.print(" int mode : ");
 SerialUSB.println(mode);
}
#endif // DEBUG

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // start serial port
  Serial.begin(BAUDRATE);

  #ifdef DEBUG
  SerialUSB.begin(BAUDRATE);
  #endif // DEBUG

  // call init function for plotter
  plot.init();
}

void SerialFlush() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}

void loop() {
  Gcode readCode;
  readCode.gcode = 0;
  // clear buffer for random existing bytes
  SerialFlush();

  // write byte to start a handshake
  Serial.write(6);

  // while not exit gcode
  while (readCode.gcode != -1) {

    // wait for reply
    while (Serial.available() == 0) {}
    readCode.gcode = Serial.parseInt();

    // check if gcode is not equal to exit
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
        // write byte to start a handshake
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
        
        #ifdef DEBUG
        print_draw(writeCode.location, 0);
        #endif // DEBUG

        break;

      //G01 Z-axis down and move to location (draw line) (x, y)
      case 1:
        plot.draw(writeCode.location, 1);

        #ifdef DEBUG
        print_draw(writeCode.location, 1);
        #endif // DEBUG

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
