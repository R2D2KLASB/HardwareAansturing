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
    Coordinate():
        x(0),
        y(0)
    {}
	template <typename X, typename Y>
    Coordinate(X x, Y y) :
        x(int(x)),
        y(int(y))
    {}
    Coordinate operator-(const Coordinate& rhs) const {
        return { x - rhs.x, y - rhs.y };
    }
    bool operator==(const Coordinate& rhs) const {
        return (x == rhs.x && y == rhs.y);
    }
    void operator+=(const Coordinate& rhs) {
        x += rhs.x;
        y += rhs.y;
    }
    void operator-=(const Coordinate& rhs) {
        x -= rhs.x;
        y -= rhs.y;
    }
    Coordinate operator+(const Coordinate& rhs) const {
        return { x + rhs.x, y + rhs.y };
    }

    bool operator!=(const Coordinate& rhs) const {
        return !(x == rhs.x && y == rhs.y);
    }
};

/**
  @brief Gcode class that contains the gcode command and either the location or the battleship information;
*/
struct Gcode {
    int gcode = 0;
    Coordinate location = { 0, 0 };
    int row = 0;
    int colom = 0;
    int player = 0;
    int width = 0;
    int length = 0;
    Gcode() = default;
	Gcode(int gcode, Coordinate location):
		gcode(gcode),
		location(location)
	{}
};
#endif //DATATYPES_HPP
