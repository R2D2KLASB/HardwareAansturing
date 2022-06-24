class controls {
private:
  const int fire = 41;
  const int up = 35;
  const int down = 37;
  const int right = 39;
  const int left = 33;

public:
  void setupControls(){
    // Fire button and joystick init
    pinMode(fire, INPUT_PULLUP);
    pinMode(up, INPUT_PULLUP);
    pinMode(down, INPUT_PULLUP);
    pinMode(right, INPUT_PULLUP);
    pinMode(left, INPUT_PULLUP);
  }

  int getPosition(){
    if ( digitalRead(fire) == LOW ){
      return 0;
    }
    if ( digitalRead(up) == LOW ){
      return 1;
    }
    if ( digitalRead(down) == LOW ){
      return 2;
    }
    if ( digitalRead(left) == LOW ){
      return 3;
    }
    if ( digitalRead(right) == LOW ){
      return 4;
    }
    return -1;
  }
};
