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



  Plot.draw(Coordinate{10000, 950});

//  Coordinate test = {0, 0};
//  for (int i=0; i<100; i+=10){
////    Serial.println(100-i);
//    test+={100, 100-i};
//    Plot.draw(test);
//  }
//
//  Serial.println("start");
//
//  Plot.draw(Coordinate{0, 1190});
//  Serial.println("_1");
//
//  Plot.draw(Coordinate{714, 1904});
//  Serial.println("_1");
////  
////  Plot.draw(Coordinate{1785, 3095});
////    Serial.println("_1");
//
//  Plot.draw(Coordinate{2142, 3095});
//  Serial.println("_1");
//  
//  Plot.draw(Coordinate{2500, 4285});
//  Serial.println("_1");
//  Plot.draw(Coordinate{2500, 4761});
//  Serial.println("_1");
//
//  Plot.draw(Coordinate{2142, 5000});
//  Serial.println("_1");
//  Plot.draw(Coordinate{1428, 4761});
//
//  Serial.println("_1");
//  Plot.draw(Coordinate{1071, 4285});  
//  Serial.println("_1");
//  Plot.draw(Coordinate{714, 19047});
//  
//  Serial.println("_1");
//  Plot.draw(Coordinate{357, 1904});
//  Serial.println("_1");
//  Plot.draw(Coordinate{0, 0});
// 
//  Serial.println("_1");
//  
 Serial.print("done");




}

void loop(){

}
