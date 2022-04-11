#ifndef XYPLOTTER_HPP
#define XYPLOTTER_HPP
#include <Arduino.h>
#include <Servo.h>

#include "plotter.hpp"
#include "DataTypes.hpp"
#include "Definitions.hpp"

class XYPlotter: public Plotter {
  public:
    enum Direction {
      counterClockwise,
      clockwise,
      standStill
    };
    XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin, Coordinate maxDimension, Servo & pen);
    void setXDirection(Direction direction);
    void setYDirection(Direction direction);
    void setXYDirection(Direction xDirection, Direction yDirection);
   
    void setServo(bool draw);
  
    bool draw(const String& gcode) {
      return true;
    }
    bool draw(Coordinate finish, bool d = 1);
    bool draw(int x, int y, bool d = 1);
    Coordinate currentLocation;


  private:
      void step();
    Coordinate maxDimension;
    struct Angle {
      const int hoek;
      const Direction motorX;
      const Direction motorY;
    };
    void up();
    void right();
    void left();
    void down();
    Direction currentXDirection;
    Direction currentYDirection;
    uint8_t xDirectionPin;
    uint8_t xStepPin;
    uint8_t yDirectionPin;
    uint8_t yStepPin;
    uint16_t currentPos[2] = {0, 0};
    uint8_t delayUs = 50;

    bool prevState;
    Servo & penHolder;
    

};


#endif //XYPLOTTER_HPP
