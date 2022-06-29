#ifndef CONTROLS_HPP
#define CONTROLS_HPP
#include "display.hpp"

class controls {
private:
  const int fire = 53;
  const int up = 51;
  const int down = 47;
  const int right = 45;
  const int left = 49;
  const int switchButton = 43;
  bool sw = false;
  bool pressed = false;
  
  unsigned int joystick_X = 0;
  unsigned int joystick_Y = 0;
  TFTdisplay &TFT;
public:
  controls(TFTdisplay &TFT):
    TFT(TFT)
  {} 

  void setupControls(){
    // Fire button and joystick init
    pinMode(fire, INPUT_PULLUP);
    pinMode(up, INPUT_PULLUP);
    pinMode(down, INPUT_PULLUP);
    pinMode(right, INPUT_PULLUP);
    pinMode(left, INPUT_PULLUP);
    pinMode(switchButton, INPUT_PULLUP);
  }
  
  int getPosition(){
    int returnValue;
    if ( digitalRead( switchButton ) == LOW ){
      if ( digitalRead(fire) == LOW ){
        //Serial.println( TFT.getPositionJoystick() );
        returnValue = 0;
      }
      else if ( digitalRead(up) == LOW ){
        if ( pressed == false ){
          if(joystick_Y > 0){
            joystick_Y--;
            TFT.setPositionJoystick(joystick_X, joystick_Y);
          }
        }
        pressed = true;
        returnValue = 1;
      }
      else if ( digitalRead(down) == LOW ){
        if ( pressed == false ){
            if(joystick_Y < 9){
              joystick_Y++;
              TFT.setPositionJoystick(joystick_X, joystick_Y);
            }
        }
        pressed = true;
        returnValue = 2;
      }
      else if ( digitalRead(left) == LOW ){
        if ( pressed == false ){
          if(joystick_X < 9){
              joystick_X++;
              TFT.setPositionJoystick(joystick_X, joystick_Y);
            }
        }
        pressed = true;
        returnValue = 3;
      }
      else if ( digitalRead(right) == LOW ){
        if ( pressed == false ){
          if(joystick_X > 0){
              joystick_X--;
              TFT.setPositionJoystick(joystick_X, joystick_Y);
            } 
        }
        pressed = true;
        returnValue = 4;
      }
      else{
        pressed = false;
        returnValue = 8;
      }
    }
    else{
      returnValue = 6;
    }
    if (digitalRead( switchButton) != sw){
      returnValue = 5;
    }
    sw = digitalRead( switchButton);
    return returnValue;
  }
};
#endif //CONTROLS_HPP
