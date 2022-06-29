#include "display.hpp"
#include "controls.hpp"

TFTdisplay TFT;
controls control( TFT );

void setup() {
  // TFT screen setup and SerialUSB
  TFT.setupScreen();
  Serial.begin(9600);

  // Joystick and fire button setup and Serial
  SerialUSB.begin(9600);
  control.setupControls();
}

void loop() {
  Serial.println(control.getPosition());
  
  // Serial for Display
  while (SerialUSB.available() > 0) {
    int message = SerialUSB.parseInt();

    if ( message == 6) {
      String string = SerialUSB.readString();
      TFT.setText( string );
    }

    if ( message == 0 ) { // 0 = removeBoats
      TFT.removeBoats();
    }
    else if ( message == 1 ) { // 1 = addHit
      TFT.addHit();
    }
    else if ( message == 2 ) { // 2 = addMiss
      TFT.addMiss();
    }
    else if ( message == 3 ) { // 3 = removeEnemyBoats
      TFT.removeEnemyBoats();
    }
    else if ( message == 4 ) { // 4 = addEnemyHit
      TFT.addEnemyHit();
    }
    else if ( message == 5 ) { // 5 = addEnemyMiss
      TFT.addEnemyMiss();
    }
  }

}
