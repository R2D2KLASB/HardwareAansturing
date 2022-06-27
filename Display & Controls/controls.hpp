class controls {
private:
  const int fire = 41;
  const int up = 37;
  const int down = 35;
  const int right = 33;
  const int left = 39;

public:
  void setupControls(){
    // Fire button and joystick init
    pinMode(fire, INPUT);
    pinMode(up, INPUT);
    pinMode(down, INPUT);
    pinMode(right, INPUT);
    pinMode(left, INPUT);
  }

  int getPosition(){
    if ( digitalRead(fire) ){
      return 0;
    }
    if ( digitalRead(up) ){
      return 1;
    }
    if ( digitalRead(down) ){
      return 2;
    }
    if ( digitalRead(left) ){
      return 3;
    }
    if ( digitalRead(right) ){
      return 4;
    }
    return -1;
  }
};
