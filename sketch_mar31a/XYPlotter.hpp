#ifndef XYPLOTTER_HPP
#define XYPLOTTER_HPP
#include <Arduino.h>

#include "plotter.hpp"
#include "DataTypes.hpp"

/// @brief

/**
  @brief XYPlotter class that implements Plotter
*/
class XYPlotter: public Plotter {
  public:
    /**
      @brief Direction enum that makes the motordirections better readable
    */
    enum Direction {
      counterClockwise,
      clockwise,
      standStill
    };


    bool draw(Coordinate finish);
    bool draw(int x, int y);

    /**
      @brief Sets the direction of the X steppermotor.
      @param direction the direction the motor needs to turn.
      @details Sets the director of the X steppermotor to the direction you give as parameter.
    */
    void setXDirection(Direction direction);

    /**
      @brief Sets the direction of the Y steppermotor.
      @param direction the direction the motor needs to turn.
      @details Sets the director of the Y steppermotor to the direction you give as parameter.
    */
    void setYDirection(Direction direction);

    /**
      @brief Sets the direction of BOTH the steppermotors.
      @param direction the direction the motor needs to turn.
      @details Sets the director of BOTH steppermotors to the direction you give as parameter.
    */
    void setXYDirection(Direction xDirection, Direction yDirection);






    /**
        @brief constructor for the XYPlotter class.
        @param enablePin this is the enable pin for both of the steppermotors.
        @param xDirectionPin this is the enable pin for the X steppermotor.
        @param xStepPin this is the step pin for the X steppermotor, write this pin high and low causes the steppermotor to make a step.
        @param yDirectionPin this is the enable pin for the Y steppermotor.
        @param xStepPin this is the step pin for the Y steppermotor, write this pin high and low causes the steppermotor to make a step.
        @param maxDimension The maximum drawingsize of the plotter.
        @details constructor for the XYplotter class which also sets all the pins.
        
    */
    XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin, Coordinate maxDimension); //, uint8_t servoPin);

  private:

    Coordinate currentLocation;
    Coordinate maxDimension;
    Direction currentXDirection;
    Direction currentYDirection;

    uint8_t delayUs = 50;
    uint8_t servoPin;
    uint8_t xDirectionPin;
    uint8_t xStepPin;
    uint8_t yDirectionPin;
    uint8_t yStepPin;


    void down();
    void left();
    void right();
    void step();
    void up();
};
#endif //XYPLOTTER_HPP
