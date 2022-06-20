#include "display.hpp"

TFTdisplay TFT;

void setup() {
  TFT.setupScreen();
  Serial.begin(9600);
}

void loop() {
  String message = Serial.readString();
   
  if ( message == "removeBoats\n" ){
    TFT.removeBoats();       
  }
  else if ( message == "addHit\n" ){
    TFT.addHit();
  }
  else if ( message == "addMiss\n" ){
    TFT.addMiss();
  }
  else if ( message == "removeEnemyBoats\n" ){
    TFT.removeEnemyBoats();
  }
  else if ( message == "addEnemyHit\n" ){
    TFT.addEnemyHit();
  }
  else if ( message == "addEnemyMiss\n" ){
    TFT.addEnemyMiss();
  }
}
