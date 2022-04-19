#ifndef DATATYPES_HPP
#define DATATYPES_HPP
/// @file

/**
  @brief Coordinate class that stores the coordinates
  @param x the value of the x-location
  @param y the value of the y-location
*/

struct Coordinate {
  int x;
  int y;
  Coordinate operator-(Coordinate lhs) {
    return {x - lhs.x, y - lhs.y};
  }
  bool operator==(Coordinate lhs) {
    return (x == lhs.x and y == lhs.y);
  }
  void operator+=(Coordinate lhs) {
    x += lhs.x;
    y += lhs.y;
  }

  Coordinate operator+(Coordinate lhs) {
    return {x + lhs.x, y + lhs.y};
  }

  bool operator!=(Coordinate lhs) {
    return !(x == lhs.x and y == lhs.y);
  }
};

/**
  @brief Gcode class that contains the gcode command and location
  @param gcode the integervalue corresponding to the gcode command
  @param location the {x, y} location the gcode command has to go to
*/
struct Gcode {
  int gcode = 0;
  Coordinate location = {0, 0};
};
#endif //DATATYPES_HPP
