
#include "XYPlotter.hpp"

#define ENABLE_PIN 8
#define DIR_X_PIN 5
#define STEP_X_PIN 2

#define DIR_Y_PIN 6
#define STEP_Y_PIN 3

#define ClOCKWISE 1
#define OTHERWISE 0

#define X_diff 30000
#define Y_diff 25000
#define DELAY 500
#define MOVEMENT 15000


XYPlotter Plot(ENABLE_PIN, DIR_X_PIN, STEP_X_PIN, DIR_Y_PIN, STEP_Y_PIN);


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

  // start serial port at 9600 bps:
  SerialUSB.begin(9600);
  Serial.begin(9600);
  //  while (!Serial) {
  //    ; // wait for serial port to connect. Needed for native USB port only
  //  }
  while (!SerialUSB) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  SerialUSB.println("setup done");
}

void loop() {
  // if we get a valid byte, read analog ins:
  if (Serial.available() > 0) {
    SerialUSB.println("in Loop");
    int g_code = Serial.parseInt();
    Coordinate XY;
    //    float x_coor = 0;
    //    float y_coor = 0;

    switch (g_code) {
      //G00 Z-axis up and move to location (x, y)
      case 0:
        XY.x = Serial.parseFloat();
        XY.y = Serial.parseFloat();
        //        Plot.draw(XY);
        print_draw(XY, 0);
        break;

      //G01 Z-axis down and move to location (draw line) (x, y)
      case 1:
        XY.x = Serial.parseFloat();
        XY.y = Serial.parseFloat();
        //        Plot.draw(XY);
        print_draw(XY, 1);
        break;

      //G02 Arc movement clockwise arc (x, y, i, j, e)
      case 2:
        break;

      //G03 Arc movement clockwise arc (x, y, i, j, e)
      case 3:
        break;
    }
  }
}
