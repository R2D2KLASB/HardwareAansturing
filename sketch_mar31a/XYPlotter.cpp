#include "XYPlotter.hpp"

void XYPlotter::setXDirection(Direction direction){
  currentXDirection = direction;
  if(direction != XYPlotter::Direction::standStill){
    digitalWrite(xDirectionPin, direction);
  }
}
void XYPlotter::setYDirection(Direction direction){
  currentYDirection = direction;
  if(direction != XYPlotter::Direction::standStill){
    digitalWrite(yDirectionPin, direction);
  }
}
void XYPlotter::setXYDirection(Direction xDirection, Direction yDirection){
  setXDirection(xDirection);
  setYDirection(yDirection);
}

XYPlotter::XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin): //, uint8_t servoPin):
	xDirectionPin(xDirectionPin),
	xStepPin(xStepPin),
	yDirectionPin(yDirectionPin),
	yStepPin(yStepPin),
	servoPin(10)
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

void XYPlotter::step() {
  bool delay = false;
  if(currentXDirection != XYPlotter::Direction::standStill){
		digitalWrite(xStepPin, HIGH);
    delay = true;
  }
  if(currentYDirection != XYPlotter::Direction::standStill){
    digitalWrite(yStepPin, HIGH);
    delay = true;
  }
  if (delay){
	  delayMicroseconds(delayUs);
    digitalWrite(xStepPin, LOW);
    digitalWrite(yStepPin, LOW);
    delayMicroseconds(delayUs);
  }
}
