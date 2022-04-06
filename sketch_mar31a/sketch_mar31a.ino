/*
  Name:    step_drive.ino

  Author:  mertwhocodes
*/

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
void setup() {
  Serial.begin(9600);
//   Serial.println("1");
//  Plot.setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::counterClockwise);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  } 
//  delay(DELAY);
//  Serial.println("2");
//  Plot.setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::clockwise);
//  for (size_t i = 0; i < MOVEMENT+1; i++) {
//    Plot.step();
//  }
//  delay(DELAY);
//  Serial.println("3");
//  Plot.setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::clockwise);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  delay(DELAY);
//  Serial.println("4");
//  Plot.setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::counterClockwise);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  for(unsigned int i=0; i<360; i++){
//    Plot.draw(i, 100);
//  }

//Plot.draw(0, 10000);
//Plot.draw(180, 10000);
//
//Plot.draw(45, 10000);
//Plot.draw(225, 10000);
//
//Plot.draw(90, 10000);
//Plot.draw(270, 10000);
//
//Plot.draw(135, 10000);
//Plot.draw(315, 10000);
//
//Plot.draw(180, 10000);
//Plot.draw(0, 10000);
//
//Plot.draw(225, 10000);
//Plot.draw(45, 10000);
//
//Plot.draw(270, 10000);
//Plot.draw(90, 10000);
//
//Plot.draw(315, 10000);
//Plot.draw(135, 10000);

for (int i=0; i<360; i+=5){
  Plot.draw(i, 5000);
  Plot.draw((i+180)%360, 5000);
  delay(100);
}

//Plot.draw(0, 10000);
//Plot.draw(180, 10000);
//
//Plot.draw(45, 10000);
//Plot.draw(225, 10000);
//
//Plot.draw(90, 10000);
//Plot.draw(270, 10000);
//
//Plot.draw(135, 10000);
//Plot.draw(315, 10000);
//
//Plot.draw(180, 10000);
//Plot.draw(0, 10000);
//
//Plot.draw(225, 10000);
//Plot.draw(45, 10000);
//
//Plot.draw(270, 10000);
//Plot.draw(90, 10000);
//
//Plot.draw(315, 10000);
//Plot.draw(135, 10000);

//  Plot.setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::clockwise);
//  for (size_t i = 0; i < 10000; i++) {
//   if(i%10000==0){
//    Plot.setYDirection(XYPlotter::Direction::counterClockwise);
//   }
//   Plot.step();
//   Plot.setYDirection(XYPlotter::Direction::standStill);
//  }
//}
}

//void loop(){
//
//}



void loop() {
//  delay(DELAY);
//  Serial.println("5");
//  Plot.setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::standStill);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  delay(DELAY);
//  Serial.println("6");
//  Plot.setXYDirection(XYPlotter::Direction::standStill, XYPlotter::Direction::counterClockwise);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  delay(DELAY);
//  Serial.println("7");
//  Plot.setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::standStill);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  delay(DELAY);
//  Serial.println("8");
//  Plot.setXYDirection(XYPlotter::Direction::standStill, XYPlotter::Direction::clockwise);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  delay(DELAY);
//  Serial.println("9");
//  Plot.setXYDirection(XYPlotter::Direction::standStill, XYPlotter::Direction::standStill);
//  for (size_t i = 0; i < MOVEMENT; i++) {
//    Plot.step();
//  }
//  delay(DELAY);

}
