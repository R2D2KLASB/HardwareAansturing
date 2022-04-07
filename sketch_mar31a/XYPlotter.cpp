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

//void XYPlotter::move(int degrees){
//  Angle direction = angles[(int)((degrees%360)/45)];
//  setXYDirection(direction.motorX, direction.motorY);
//  step();
//}


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

XYPlotter::XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin): //, uint8_t servoPin):
  xDirectionPin(xDirectionPin),
  xStepPin(xStepPin),
  yDirectionPin(yDirectionPin),
  yStepPin(yStepPin),
  servoPin(10),
  currentLocation({0, 0})
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

void XYPlotter::draw(Coordinate finish) {
  Coordinate delta = finish - currentLocation;
  if (delta == Coordinate{0, 0}) {
    return;
  }
  float rc = (float)delta.y / (float)delta.x;
  long int powX = pow(delta.x, 2);
  long int powY = pow(delta.y, 2);
  long int XY = powX + powY;

  long int distance = sqrt(XY);

  Serial.print("rc: ");
  Serial.println(rc);
  if (delta.x == 0 and delta.y > 0) {
    for (int i = 0; i < delta.y; i++) {
      up();
    }
  }
  else if (delta.x == 0 and delta.y < 0) {
    for (int i = delta.y; i < 0; i++) {
      down();
    }
  }
  else if (delta.y == 0 and delta.x > 0) {
    for (int i = 0; i < delta.x; i++) {
      right();
    }
  }
  else if (delta.y == 0 and delta.x < 0) {
    for (int i = delta.x; i < 0; i++) {
      left();
    }
  }
  else if (rc<0 and delta.x >= 0) {
    Coordinate start = {0, 0};
    int i = 0;
    while (start != delta) {
      i++;
      long int i2 = i * 100;
      if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        right();
        down();
        start += {1, -1};
      }
      else if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        right();
        start += {1, 0};
      }
      else if (i2 % (long int)(rc * 100) == 0) {
        down();
        start += {0, -1};
      }
    }
  }
  else if (rc < 0 and delta.x < 0) {
    Coordinate start = {0, 0};
    int i = 0;
    while (start != delta) {
      i++;
      long int i2 = i * 100;
      if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        left();
        up();
        start += { -1, 1};
      }
      else if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        up();
        start += { 0, 1};
      }
      else if (i2 % (long int)(rc * 100) == 0) {
        left();
        start += {-1, 0};
      }
    }



  }
  else if (rc > 0 and delta.x >= 0) {
    Coordinate start = {0, 0};
    int i = 0;
    while (start != delta) {
      i++;
      long int i2 = i * 100;
      if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        right();
        up();
        start += {1, 1};
      }
      else if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        right();
        start += {1, 0};
      }
      else if (i2 % (long int)(rc * 100) == 0) {
        up();
        start += {0, 1};
      }
    }
  }
  else if (rc > 0 and delta.x < 0) {
    Coordinate start = {0, 0};
    int i = 0;
    while (start != delta) {
      i++;
      long int i2 = i * 100;
      if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        left();
        down();
        start += { -1, -1};
      }
      else if (i2 % (long int)((1 / (rc) * 100)) == 0) {
        down();
        start += { 0, -1};

      }
      else if (i2 % (long int)(rc * 100) == 0) {
        left();
        start += {-1, 0};
      }
    }
  }
  delay(50);
  currentLocation = finish;
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
//struct Vergroting {
//  unsigned int stappen;
//  unsigned int verhoging;
//};
//
//
//Vergroting findStappen(double ratio) {
//  if (ratio == 0) {
//    return {0, 1};
//  }
//  if (ratio == 1) {
//    return {1, 1};
//  }
//  unsigned int verhoging = 1;
//  while (((1 / ratio)*verhoging) != int((1 / ratio)*verhoging)) {
//    verhoging++;
//  }
//  auto x = (1 / ratio) * verhoging;
//  Vergroting returnValue = {(unsigned int) x, (unsigned int)verhoging};
//  Serial.print("stappen: ");
//  Serial.print(returnValue.stappen);
//  Serial.println("\n\n");
//  return returnValue;
//}
//
//void XYPlotter::draw(int hoek, unsigned int aantalStappen) {
//  XYPlotter::Angle vorigeHoek = angles[(int)((hoek % 360) / ANGLE_INTERVAL)];
//  XYPlotter::Angle volgendeHoek = angles[((int)((hoek % 360) / ANGLE_INTERVAL) + 1) % 8];
//
//  double ratio = (double)(hoek % ANGLE_INTERVAL) / (ANGLE_INTERVAL);
//
//
//  Vergroting returnValue = findStappen(ratio);
//  unsigned int stappen = returnValue.stappen;
//  long unsigned int verhoging = returnValue.verhoging;
//
//  ///Werkend voor 30 graden
//  for (long unsigned int i = 0; i < aantalStappen; i++) {
//    setXYDirection(volgendeHoek.motorX, volgendeHoek.motorY);
//    if ((i * verhoging) % stappen == 0) {
//      setXYDirection(vorigeHoek.motorX, vorigeHoek.motorY);
//    }
//    step();
//  }
//
//
//  ///Werkend voor 10 graden
//  for (long unsigned int i = 0; i < aantalStappen; i++) {
//    setXYDirection(vorigeHoek.motorX, vorigeHoek.motorY);
//    if ((i * verhoging) % stappen == 0) {
//      setXYDirection(volgendeHoek.motorX, volgendeHoek.motorY);
//    }
//    step();
//  }
//
//
//}
//
//
//
//
////  Serial.print(", ");
////  Serial.print("Ratio = ");
////  Serial.print(ratio);
////  Serial.print(", ");
////  Serial.print("VorigeHoekD = ");
////  Serial.print(vorigeHoek.hoek);
////  Serial.print(", ");
////  Serial.print("VorigeHoekX = ");
////  Serial.print(vorigeHoek.motorX);
////  Serial.print(", ");
////  Serial.print("VorigeHoekY = ");
////  Serial.print(vorigeHoek.motorY);
////  Serial.print(", ");
////  Serial.print("VolgendeHoekD = ");
////  Serial.print(volgendeHoek.hoek);
////  Serial.print(", ");
////  Serial.print("VolgendeHoekX = ");
////  Serial.print(volgendeHoek.motorX);
////  Serial.print(", ");
////  Serial.print("VolgendeHoekY = ");
////  Serial.print(volgendeHoek.motorY);
////  Serial.print(", ");
////  Serial.print("Ratio = ");
////  Serial.print(ratio);
////  Serial.print(", ");
////  Serial.print("Stappen = ");
////  Serial.print(stappen);
////  Serial.print(", ");
////  Serial.print("Verhoging = ");
////  Serial.print(verhoging);
////  Serial.print(", ");
////  Serial.print("Coordinate = ");
////  Serial.print(currentPos[0]);
////  Serial.print(", ");
////  Serial.println(currentPos[1]);
//
//
//void XYPlotter::draw2(int hoek, unsigned int aantalStappen) {
//  XYPlotter::Angle vorigeHoek = angles[(int)((hoek % 360) / ANGLE_INTERVAL)];
//  XYPlotter::Angle volgendeHoek = angles[((int)((hoek % 360) / ANGLE_INTERVAL) + 1) % 8];
//
//  double ratio = (double)(hoek % ANGLE_INTERVAL) / (ANGLE_INTERVAL);
//
//
//  Vergroting returnValue = findStappen(ratio);
//  unsigned int stappen = returnValue.stappen;
//  long unsigned int verhoging = returnValue.verhoging;
//
//
//  ///Werkend voor 10 graden
//  for (long unsigned int i = 0; i < aantalStappen; i++) {
//    setXYDirection(vorigeHoek.motorX, vorigeHoek.motorY);
//    if ((i * verhoging) % stappen == 0) {
//      setXYDirection(volgendeHoek.motorX, volgendeHoek.motorY);
//    }
//    step();
//  }
//}
