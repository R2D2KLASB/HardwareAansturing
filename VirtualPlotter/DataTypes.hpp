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
  Coordinate(const int& x, const int& y):
  x(x),
  y(y)
  {}
  Coordinate operator-(const Coordinate & rhs) const{
    return {x - rhs.x, y - rhs.y};
  }
  bool operator==(const Coordinate &rhs) const{
    return (x == rhs.x && y == rhs.y);
  }
  void operator+=(const Coordinate &rhs) {
    x += rhs.x;
    y += rhs.y;
  }
  void operator-=(const Coordinate &rhs) {
    x -= rhs.x;
    y -= rhs.y;
  }
  Coordinate operator+(const Coordinate &rhs) const{
    return {x + rhs.x, y + rhs.y};
  }

  bool operator!=(const Coordinate & rhs) const{
    return !(x == rhs.x && y == rhs.y);
  }
};

/**
  @brief Gcode class that contains the gcode command and location
*/
struct Gcode {
  int gcode = 0;
  Coordinate location = {0, 0};
  int row = 0;
  int colom = 0;
  int player = 0;
  int width = 0;
  int length = 0;
};
#endif //DATATYPES_HPP
