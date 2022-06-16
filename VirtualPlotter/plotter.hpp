#include "DataTypes.hpp"
#include <math.h>

/// @file

/**
  @brief virtual class implemented by the XY-plotter and the virtual xy-plotter
*/
class Plotter {
  public:
    /**
      @brief constructer for the Plotter class
      @details constructer for the Plotter class that also handles all the setup steps
    */
    Plotter(Coordinate maxDimension):
      maxDimension(maxDimension)
    {};

    /**
      @brief public draw function to move the pen from the current location to the given location the boolean indicates if the pen is on the paper or lifted
      @param finish Struct containing the x and y location of the destination
      @param draw boolean that indicates if the pen is on the paper or lifted
      @return returns if the operation was succesfull or not the reason for failure is that the location is out of bounds
    */
    bool draw(Coordinate finish, bool draw = 1);
    
    /**
      @brief public draw function that's an adapter for the bool draw(Coordinate finish, bool draw) function;
      @param x integer containing value of the x location of the destination
      @param y integer containing value of the y location of the destination
      @param draw boolean that indicates if the pen is on the paper or lifted
      @return returns if the operation was succesfull or not the reason for failure is that the location is out of bounds
    */
    bool draw(int x, int y, bool draw = 1);

    /**
     * @brief function that draws the grid for battleship
     */
    void g3();

    /**
     * @brief function that draws a star ont the game board to indicate a hit
     * 
     * @param row the x-location on the battleship game board
     * @param colom the y-location on the battleship game board
     * @param player indicates if the targetboard is the friendly's game board or the enemie's game board 
     */
    void g4(const int& row, const int& colom, const int& player);

    /**
     * @brief function that draws a cross on the game board to indicate a miss
     * 
     * @param row the x-location on the battleship game board
     * @param colom the y-location on the battleship game board
     * @param player indicates if the targetboard is the friendly's game board or the enemie's game board 
     */
    void g5(const int& row, const int& colom, const int& player);
    
    /**
     * @brief function that draws a boat on the game board
     * 
     * @param row the x-location on the battleship game board
     * @param colom the y-location on the battleship game board
     * @param width the length of the boat over the x-axis
     * @param length the length of the boat over the y-axis
     */
    void g6(const int& row, const int& colom, const int& width, const int& length);

    /**
      @brief Function to return to the homeposition using the microswitches
      @details Homing function that uses the microswitches to return to the homeposition over x and y-axis and than resets than calibrates the position.
    */
    virtual void home() = 0;
  protected:
    bool prevState = false;
    Coordinate currentLocation = {0,0};
    const Coordinate enemyGameboardOrigin = {21000, 0};
    const Coordinate friendlyGameboardOrigin = {0, 0};
    Coordinate maxDimension;

    const float pi = 3.14159265358979323846;


    const int friendlyGameboardSize = 20000;
    const int enemyGameboardSize = 5000;

    unsigned int stepDelayUs = 50;
    unsigned int servoDelayUs = 150000;
    
    virtual void down() = 0;
    virtual void left() = 0;
    virtual void right() = 0;
    virtual void up() = 0;

    virtual void setServo(bool draw) = 0;

    Coordinate pointsOnCircle(const int& radius, const int& angle, const Coordinate& origin);

};
