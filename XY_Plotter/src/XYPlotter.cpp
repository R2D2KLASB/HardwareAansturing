#include "XYPlotter.hpp"

void XYPlotter::home() {
  setServo(0);
  while (digitalRead(ySwitchPin)) {
    up(stepDelayUs);
  }
  while (digitalRead(xSwitchPin)) {
    right(stepDelayUs);
  }
  currentLocation = {30000, 25000};
}

void XYPlotter::init() {
  pinMode(12, OUTPUT);
  pinMode(xDirectionPin, OUTPUT);
  pinMode(xStepPin, OUTPUT);
  pinMode(yDirectionPin, OUTPUT);
  pinMode(yStepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(xSwitchPin, INPUT);
  pinMode(ySwitchPin, INPUT);


  digitalWrite(12, HIGH);
  digitalWrite(xStepPin, LOW);
  digitalWrite(yStepPin, LOW);
  digitalWrite(enablePin, LOW);

  setServo(0);

  home();
}

XYPlotter::XYPlotter(Coordinate maxDimension, Servo & pen, uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t xSwitchPin, uint8_t yDirectionPin, uint8_t yStepPin, uint8_t ySwitchPin):
  Plotter(maxDimension),
  penHolder(pen),
  enablePin(enablePin),
  xDirectionPin(xDirectionPin),
  xStepPin(xStepPin),
  xSwitchPin(xSwitchPin),
  yDirectionPin(yDirectionPin),
  yStepPin(yStepPin),
  ySwitchPin(ySwitchPin)
{}

void XYPlotter::down(unsigned int stepDelay) {
  setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::clockwise);
  step(stepDelay);
}

void XYPlotter::left(unsigned int stepDelay) {
  setXYDirection(XYPlotter::Direction::counterClockwise, XYPlotter::Direction::counterClockwise);
  step(stepDelay);
}

void XYPlotter::right(unsigned int stepDelay) {
  setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::clockwise);
  step(stepDelay);
}

void XYPlotter::setServo(bool draw) {
  penHolder.attach(SERVO_PIN);
  if (draw) {
    penHolder.write(SERVO_DOWN);
  }
  else {
    penHolder.write(SERVO_UP);
  }
  prevState = draw;
  delayMicroseconds(servoDelayUs);
}

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

void XYPlotter::step(unsigned int stepDelay) {
  bool delay = false;
  if (currentXDirection != XYPlotter::Direction::standStill) {
    digitalWrite(xStepPin, HIGH);
    delay = true;
  }
  if (currentYDirection != XYPlotter::Direction::standStill) {
    digitalWrite(yStepPin, HIGH);
    delay = true;
  }
  if (delay) {
    delayMicroseconds(stepDelay);
    digitalWrite(xStepPin, LOW);
    digitalWrite(yStepPin, LOW);
    delayMicroseconds(stepDelay);
  }
}

void XYPlotter::up(unsigned int stepDelay) {
  setXYDirection(XYPlotter::Direction::clockwise, XYPlotter::Direction::counterClockwise);
  step(stepDelay);
}
