class controls {
private:
  const int fire = 53;
  const int up = 51;
  const int down = 47;
  const int right = 45;
  const int left = 49;
  const int switchButton = 43;
  bool sw = false;

public:
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
        returnValue = 0;
      }
      else if ( digitalRead(up) == LOW ){
        returnValue = 1;
      }
      else if ( digitalRead(down) == LOW ){
        returnValue = 2;
      }
      else if ( digitalRead(left) == LOW ){
        returnValue = 3;
      }
      else if ( digitalRead(right) == LOW ){
        returnValue = 4;
      }
      else{
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
