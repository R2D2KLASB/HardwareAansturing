#ifndef DATATYPES_HPP
#define DATATYPES_HPP
/// @file

/**
* @brief Coordinate class
* @param x 
* 
* @details detailed description
* @return return details
*/

struct Coordinate {
  int x;
  int y;
  Coordinate operator-(Coordinate lhs) {
    return {x - lhs.x, y - lhs.y};
  }
  bool operator==(Coordinate lhs){
    return (x==lhs.x and y == lhs.y);
  }
  void operator+=(Coordinate lhs){
    x+=lhs.x;
    y+=lhs.y;
  }

  Coordinate operator+(Coordinate lhs){
    return {x+lhs.x, y+lhs.y};
  }
  
  bool operator!=(Coordinate lhs){
    return !(x==lhs.x and y == lhs.y);
  }
};


struct Gcode{
  int gcode=0;
  Coordinate location={0,0};
};
#endif //DATATYPES_HPP
