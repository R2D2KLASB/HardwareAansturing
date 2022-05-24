#ifndef XYPLOTTER_HPP
#define XYPLOTTER_HPP
#include <Arduino.h>
#include <Servo.h>

#include "plotter.hpp"
#include "DataTypes.hpp"
#include "Definitions.hpp"
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

    enum motorState{
      off = 1,
      on = 0
    };


    void home() override;

    /**
       @brief init function that set the Pinmode sets the servo in the upposition and calls the home-function
    */
    void init();

    void setMotorState(motorState newState);

    /**
      @brief constructor for the XYPlotter class.
      @param enablePin this is the enable pin for both of the steppermotors.
      @param xDirectionPin this is the enable pin for the X steppermotor.
      @param xStepPin this is the step pin for the X steppermotor, write this pin high and low causes the steppermotor to make a step.
      @param yDirectionPin this is the enable pin for the Y steppermotor.
      @param xStepPin this is the step pin for the Y steppermotor, write this pin high and low causes the steppermotor to make a step.
      @param maxDimension The maximum drawingsize of the plotter.
      @param pen Reference to the servo of the pen so that we can lift it from the paper
      @param xSwitchPin The pin to which the microswitch of the x-axis is connected.
      @param ySwitchPin The pin to which the microswitch of the x-axis is connected.
      @details constructor for the XYplotter class which also sets all the pins.
    */

    XYPlotter(Coordinate maxDimension, Servo & pen, uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t xSwitchPin, uint8_t yDirectionPin, uint8_t yStepPin, uint8_t ySwitchPin);
  private:

    Direction currentXDirection;
    Direction currentYDirection;
    
    Servo & penHolder;
    uint8_t enablePin;
    uint8_t xDirectionPin;
    uint8_t xStepPin;
    uint8_t xSwitchPin;
    uint8_t yDirectionPin;
    uint8_t yStepPin;
    uint8_t ySwitchPin;

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
    void step();
    
    void down() override;
    void left() override;
    void right() override;
    void up() override;

    void setServo(bool draw) override;
};
#endif //XYPLOTTER_HPP
