#ifndef XYPLOTTER_HPP
#define XYPLOTTER_HPP
#include <Arduino.h>
#include "plotter.hpp"
#include <Dictionary>

class XYPlotter: public Plotter {
public:
  enum Direction{
    standStill=2,
    clockwise=1,
    counterClockwise=0,
  };
	XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin); //, uint8_t servoPin);
  void setXDirection(Direction direction);
  void setYDirection(Direction direction); 
  void setXYDirection(Direction xDirection, Direction yDirection);
	void step();    

   bool draw(const String& gcode){
     return true;
   }

private:
  Dictionary &Angles = new *(Dictionary(8));
  Direction currentXDirection;
  Direction currentYDirection;
  uint8_t xDirectionPin;
  uint8_t xStepPin;
  uint8_t yDirectionPin;
  uint8_t yStepPin;
  uint8_t servoPin;
  uint16_t currentPos[2] = {0, 0};
  uint8_t delayUs = 15;

  Dicti
};


#endif //XYPLOTTER_HPP
