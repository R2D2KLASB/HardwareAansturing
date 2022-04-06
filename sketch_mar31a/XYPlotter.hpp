#ifndef XYPLOTTER_HPP
#define XYPLOTTER_HPP
#include <Arduino.h>
#include "plotter.hpp"

#define ANGLE_INTERVAL 45

class XYPlotter: public Plotter {
public:
  enum Direction{
    counterClockwise,
    clockwise,
    standStill
  };
	XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin); //, uint8_t servoPin);
  void setXDirection(Direction direction);
  void setYDirection(Direction direction); 
  void setXYDirection(Direction xDirection, Direction yDirection);
	void step();    

   bool draw(const String& gcode){
     return true;
   }
   void draw(int hoek, unsigned int aantalStappen);

private:
  struct Angle{
    const int hoek;
    const Direction motorX;
    const Direction motorY;   
  };
  
  const Angle angles[8] = {{0, Direction::counterClockwise, Direction::clockwise},
                           {45, Direction::counterClockwise, Direction::standStill},
                           {90, Direction::counterClockwise, Direction::counterClockwise},
                           {135, Direction::standStill, Direction::counterClockwise},
                           {180, Direction::clockwise, Direction::counterClockwise},
                           {225, Direction::clockwise, Direction::standStill},
                           {270, Direction::clockwise, Direction::clockwise},
                           {315, Direction::standStill, Direction::clockwise}};
  Direction currentXDirection;
  Direction currentYDirection;
  uint8_t xDirectionPin;
  uint8_t xStepPin;
  uint8_t yDirectionPin;
  uint8_t yStepPin;
  uint8_t servoPin;
  uint16_t currentPos[2] = {0, 0};
  uint8_t delayUs = 15;

};


#endif //XYPLOTTER_HPP
