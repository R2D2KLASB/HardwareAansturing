#include "XYPlotter.hpp"

void XYPlotter::setXDirection(Direction direction) {
  currentXDirection = direction;
  if (direction != XYPlotter::Direction::standStill) {
    digitalWrite(xDirectionPin, direction);
  }
}
void XYPlotter::setYDirection(Direction direction) {
  currentYDirection = direction;
  if (direction != XYPlotter::Direction::standStill) {
    digitalWrite(yDirectionPin, direction);
  }
}
void XYPlotter::setXYDirection(Direction xDirection, Direction yDirection) {
  setXDirection(xDirection);
  setYDirection(yDirection);
}

void XYPlotter::setServo(bool draw){
  penHolder.attach(SERVO_PIN);
  if(draw){
    penHolder.write(SERVO_DOWN);
  }
  else{
    penHolder.write(SERVO_UP);
  }
  prevState = draw;
  delay(150);
}


void XYPlotter::up() {
  setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::clockwise);
  step();
}
void XYPlotter::right() {
  setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::counterClockwise);
  step();
}
void XYPlotter::left() {
  setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::clockwise);
  step();
}
void XYPlotter::down() {
  setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::counterClockwise);
  step();
}

XYPlotter::XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin, Coordinate maxDimension, Servo & pen):
  xDirectionPin(xDirectionPin),
  xStepPin(xStepPin),
  yDirectionPin(yDirectionPin),
  yStepPin(yStepPin),
  currentLocation({0, 0}),
  maxDimension(maxDimension),
  penHolder(pen)
{
  pinMode(xDirectionPin, OUTPUT);
  pinMode(xStepPin, OUTPUT);
  pinMode(yDirectionPin, OUTPUT);
  pinMode(yStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(xStepPin, LOW);
  digitalWrite(yStepPin, LOW);
  digitalWrite(enablePin, LOW);
}

bool XYPlotter::draw(int x, int y, bool d) {
  return draw(Coordinate{x/5, y/5}, d);
}


bool XYPlotter::draw(Coordinate finish, bool draw) {
  if (draw != prevState){
    setServo(draw);
  }
  
  if (finish.x > maxDimension.x or finish.y > maxDimension.y) {
    return true;
  }
  Coordinate delta = finish - currentLocation;
  if (delta == Coordinate{0, 0}) {
    return false;
  }
  if (delta.x == 0) {
    if (delta.y > 0) {
      for (unsigned int i = 0; i < delta.y; i++) {
        up();
        currentLocation += {0, 1};
      }
    }
    else {
      for (unsigned int i = 0; i < 0 - delta.y; i++) {
        down();
        currentLocation += {0, -1};
      }
    }
  }
  else if (delta.y == 0) {
    if (delta.x > 0) {
      for (unsigned int i = 0; i < delta.x; i++) {
        right();
        currentLocation += {1, 0};
      }
    }
    else {
      for (unsigned int i = 0; i < 0 - delta.x; i++) {
        left();
        currentLocation += { -1, 0};
      }
    }
  }
  else {

    Coordinate startLocation = currentLocation;
    Coordinate nextLocation = {0, 0};
    float rc = delta.y / (float)delta.x;
    if (delta.x > 0) {
      if (rc > 0) {
        for (int i = 0; i <= delta.x; i++) {
          nextLocation = startLocation + Coordinate{i, int(rc * i)};
          right();
          for (unsigned int i = 0; i < abs(nextLocation.y - currentLocation.y); i++) {
            up();
          }
          currentLocation = nextLocation;
        }
      }
      else {
        for (int i = 0; i <= delta.x; i++) {
          nextLocation = startLocation + Coordinate{i, int(rc * i)};
          right();
          for (unsigned int i = 0; i < abs(nextLocation.y - currentLocation.y); i++) {
            down();
          }
          currentLocation = nextLocation;
        }
      }
    }
    else {
      if (rc > 0) {
        for (int i = 0; i <= 0 - delta.x; i++) {
          nextLocation = startLocation + Coordinate{ -i, int(rc * -i)};
          left();
          for (unsigned int i = 0; i < abs(nextLocation.y - currentLocation.y); i++) {
            down();
          }
          currentLocation = nextLocation;
        }
      }
      else {
        for (int i = 0; i <= 0 - delta.x; i++) {
          nextLocation = startLocation + Coordinate{ -i, int(rc * -i)};
          left();
          for (unsigned int i = 0; i < abs(nextLocation.y - currentLocation.y); i++) {
            up();
          }
          currentLocation = nextLocation;
        }
      }
    }
  }
  return false;
}


void XYPlotter::step() {
  bool delay = false;
  if (currentXDirection != XYPlotter::Direction::standStill) {
    digitalWrite(xStepPin, HIGH);
    delay = true;
    if (currentXDirection == XYPlotter::Direction::counterClockwise) {
      currentPos[0]++;
    }
    else {
      currentPos[0]--;
    }
  }
  if (currentYDirection != XYPlotter::Direction::standStill) {
    digitalWrite(yStepPin, HIGH);
    delay = true;
    if (currentYDirection == XYPlotter::Direction::counterClockwise) {
      currentPos[1]++;
    }
    else {
      currentPos[1]--;
    }
  }
  if (delay) {
    delayMicroseconds(delayUs);
    digitalWrite(xStepPin, LOW);
    digitalWrite(yStepPin, LOW);
    delayMicroseconds(delayUs);
  }
}
