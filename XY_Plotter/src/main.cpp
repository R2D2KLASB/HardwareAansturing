#include <Servo.h>
#include "XYPlotter.hpp"
#include "DataTypes.hpp"
#include "Definitions.hpp"
#include "queue.hpp"

//#define DEBUG

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

// void print_draw(Gcode command){
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print(command.location.x);
//   SerialUSB.print("Draw y: ");
//   SerialUSB.print(command.location.y);
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
//   SerialUSB.print("Draw x: ");
  
// }
#endif // DEBUG

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // start serial port
  Serial.begin(BAUDRATE, SERIAL_8E1);
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

  SerialFlush();

  // write byte to start a handshake
  Serial.write('A');

  // while not exit gcode
  while (readCode.gcode != -1) {

    // wait for reply
    while (Serial.available() == 0) {}
    readCode.gcode = Serial.parseInt();
    // check if gcode is not equal to exit
    if (readCode.gcode != -1) {
      switch (readCode.gcode) {
        case -2:
          queue.flush();
          SerialFlush();
          break;
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
        case 3:
          break;
        case 4:
          readCode.row = Serial.parseInt();
          readCode.colom = Serial.parseInt();
          readCode.player = Serial.parseInt();
          break;
        case 5:
          readCode.row = Serial.parseInt();
          readCode.colom = Serial.parseInt();
          readCode.player = Serial.parseInt();
          break;
        case 6:
          readCode.row = Serial.parseInt();
          readCode.colom = Serial.parseInt();
          readCode.width = Serial.parseInt();
          readCode.length = Serial.parseInt();
          break;
        case 7:
          readCode.row = Serial.parseInt();
          readCode.colom = Serial.parseInt();
          readCode.width = Serial.parseInt();
          readCode.length = Serial.parseInt();
          readCode.player = Serial.parseInt();
          break;
        case 8:
          readCode.player = Serial.parseInt();
        case 28:
          break;
        default:
          break;
      }
      if(readCode.gcode!=-2){
        queue.append(readCode);
        if (queue.isFull()) {
          break;
        }
        else {
          // write byte to start a handshake
          Serial.write('A');
        }
      }
      else{
        Serial.write('B');
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
      case 3:
        plot.g3();
        break;
      case 4:
        plot.g4(writeCode.row, writeCode.colom, writeCode.player);
        break;
      case 5:
        plot.g5(writeCode.row, writeCode.colom, writeCode.player);
        break;
      case 6:
        plot.g6(writeCode.row, writeCode.colom, writeCode.width, writeCode.length);
        break;
      case 7:
        plot.g7(writeCode.row, writeCode.colom, writeCode.width, writeCode.length, writeCode.player);
        break;
      case 8:
        plot.g8(writeCode.player);
      //G28 home
      case 28:
        plot.home();
        break;
      default:
        break;
    }
  }
  // clear buffer for random existing bytes
  SerialFlush();
}
