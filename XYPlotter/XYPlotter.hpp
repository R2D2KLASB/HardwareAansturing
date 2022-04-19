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

    /**
      @brief public draw function to move the pen from the current location to the given location the boolean indicates if the pen is on the paper or lifted
      @param finish Struct containing the x and y location of the destination
      @param draw boolean that indicates if the pen is on the paper or lifted
      @return returns if the operation was succesfull or not the reason for failure is that the location is out of bounds
    */
    bool draw(Coordinate finish, bool draw = 1);
    /**
      @brief public draw function that's an interface for the bool draw(Coordinate finish, bool draw) function;
      @param x integer containing value of the x location of the destination
      @param y integer containing value of the y location of the destination
      @param draw boolean that indicates if the pen is on the paper or lifted
      @return returns if the operation was succesfull or not the reason for failure is that the location is out of bounds
    */
    bool draw(int x, int y, bool draw = 1);

    /**
      @brief Function to return to the homeposition using the microswitches
      @details Homing function that uses the microswitches to return to the homeposition over x and y-axis and than resets than calibrates the position.
    */
    bool home();

    /**
       @brief init function that set the Pinmode sets the servo in the upposition and calls the home-function
    */
    void init();


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
    XYPlotter(uint8_t enablePin, uint8_t xDirectionPin, uint8_t xStepPin, uint8_t yDirectionPin, uint8_t yStepPin, Coordinate maxDimension, Servo & pen, uint8_t xSwitchPin, uint8_t ySwitchPin);
  private:

    bool prevState;
    bool firstDraw = true;
    Coordinate currentLocation;
    Coordinate maxDimension;
    Direction currentXDirection;
    Direction currentYDirection;

    Servo & penHolder;

    uint8_t delayUs = 50;
    uint8_t enablePin;
    uint8_t servoPin;
    uint8_t xSwitchPin;
    uint8_t xDirectionPin;
    uint8_t xStepPin;
    uint8_t ySwitchPin;
    uint8_t yDirectionPin;
    uint8_t yStepPin;

    void down();
    void left();
    void right();
    void setServo(bool draw);
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
    void step();
    void up();



};
#endif //XYPLOTTER_HPP
