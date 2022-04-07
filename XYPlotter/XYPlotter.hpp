#ifndef XYPLOTTER_HPP
#define XYPLOTTER_HPP
#include <Arduino.h>
#include "plotter.hpp"

/// @file
/**
* @brief XYplotter class for controlling the 2 steppermotors X and Y
*/

class XYPlotter: public Plotter {
public:
  /**
  * @brief Sets the direction.
  * @details Multiple directions can be set. It can stand still, rotate clockwise and rotate counter clockwise.
  */
  enum Direction{
    standStill=2,
    clockwise=1,
    counterClockwise=0,
  };
  /**
  * @brief constructor for the XYPlotter class.
  * @param enablePin this is the enable pin for both of the steppermotors.
  * @param xDirectionPin this is the enable pin for the X steppermotor.
  * @param xStepPin this is the enable pin for the X steppermotor.
  * @param yDirectionPin this is the enable pin for the Y steppermotor.
  * @param yStepPin this is the enable pin for the Y steppermotor.
  * @details constructor for the XYplotter class which also sets all the pins.
  */
	XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin); //, uint8_t servoPin);

  /**
  * @brief Sets the direction of the X steppermotor.
  * @param direction the direction the motor needs to turn.
  * @details Sets the director of the X steppermotor to the direction you give as parameter.
  */
  void setXDirection(Direction direction);

  /**
  * @brief Sets the direction of the Y steppermotor.
  * @param direction the direction the motor needs to turn.
  * @details Sets the director of the Y steppermotor to the direction you give as parameter.
  */
  void setYDirection(Direction direction);

  /**
  * @brief Sets the direction of BOTH the steppermotors.
  * @param direction the direction the motor needs to turn.
  * @details Sets the director of BOTH steppermotors to the direction you give as parameter.
  */
  void setXYDirection(Direction xDirection, Direction yDirection);

  /**
  * @brief Does one step in the directions the motors have.
  * @details Does one step in the directions the steppermotors have.
  */
  void step();

  /**
  * @brief Function 
  * @details
  */
  bool draw(const String& gcode){
   return true;
  }

private:
//  Dictionary &Angle = *(new Dictionary(8));
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
