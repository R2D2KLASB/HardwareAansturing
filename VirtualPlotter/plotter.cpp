#include "plotter.hpp"

unsigned int absolute(int value) {
	if (value < 0) {
		return -value;
	}
	return value;
}

Coordinate pointsOnCircle(int radius, int angle, Coordinate origin) {
    float PI = 3.14159265358979323846;
    return Coordinate(origin.x + radius * cos(angle * PI / 180), origin.y + radius * sin(angle * PI / 180));
}

bool Plotter::draw(Coordinate finish, bool draw) {
  if (draw != prevState) {
    setServo(draw);
  }
  if (finish.x > maxDimension.x || finish.y > maxDimension.y || finish.y < 0 || finish.x < 0) {
    return true;
  }
  Coordinate delta = finish - currentLocation;
  if (delta == Coordinate{0, 0}) {
    return false;
  }
  if (delta.x == 0) {
    if (delta.y > 0) {
      for (int i = 0; i < delta.y; i++) {
        up();
        currentLocation.y++;
      }
    }
    else {
      for (int i = 0; i < 0 - delta.y; i++) {
        down();
        currentLocation.y--;
      }
    }
  }
  else if (delta.y == 0) {
    if (delta.x > 0) {
      for (int i = 0; i < delta.x; i++) {
        right();
        currentLocation.x++;
      }
    }
    else {
      for (int i = 0; i < 0 - delta.x; i++) {
        left();
        currentLocation.x--;
      }
    }
  }
  else {
    Coordinate startLocation = currentLocation;
    Coordinate nextLocation = {0, 0};
    float rc = delta.y / (float)delta.x;
    if (delta.x > 0) {
      if (rc > 0) {
        for (int i = 0; i <= delta.x; i++) {
          nextLocation = startLocation + Coordinate{i, int(rc * i)};
          right();
          currentLocation.x++;
          for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
            up();
            currentLocation.y++;
          }
          currentLocation = nextLocation;
        }
      }
      else {
        for (int i = 0; i <= delta.x; i++) {
          nextLocation = startLocation + Coordinate{i, int(rc * i)};
          right();
          currentLocation.x++;
          for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
            down();
            currentLocation.y--;
          }
          currentLocation = nextLocation;
        }
      }
    }
    else {
      if (rc > 0) {
        for (int i = 0; i <= 0 - delta.x; i++) {
          nextLocation = startLocation + Coordinate{ -i, int(rc * -i)};
          left();
          currentLocation.x--;
          for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
            down();
            currentLocation.y--;
          }
          currentLocation = nextLocation;
        }
      }
      else {
        for (int i = 0; i <= 0 - delta.x; i++) {
          nextLocation = startLocation + Coordinate{ -i, int(rc * -i)};
          left();
          currentLocation.x--;
          for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
            up();
            currentLocation.y++;
          }
          currentLocation = nextLocation;
        }
      }
    }
  }
  return false;

}

bool Plotter::draw(int x, int y, bool d) {
  return draw(Coordinate{x, y}, d);
}

void Plotter::g3(){
  draw(friendlyGameboardOrigin, 0);
  for (unsigned int i = 0; i < 5; i++) {
		draw({ currentLocation.x + friendlyGameboardSize, currentLocation.y }, 1);
		draw({ currentLocation.x, currentLocation.y + friendlyGameboardSize / 10 }, 1);
		draw({ currentLocation.x - friendlyGameboardSize, currentLocation.y }, 1);
		draw({ currentLocation.x, currentLocation.y + friendlyGameboardSize / 10 }, 1);
	}
	draw({ currentLocation.x + friendlyGameboardSize, currentLocation.y }, 1);

	for (unsigned int i = 0; i < 5; i++) {
		draw({ currentLocation.x, currentLocation.y - friendlyGameboardSize }, 1);
		draw({ currentLocation.x - friendlyGameboardSize / 10, currentLocation.y}, 1);
		draw({ currentLocation.x, currentLocation.y + friendlyGameboardSize }, 1);
		draw({ currentLocation.x - friendlyGameboardSize / 10, currentLocation.y }, 1);
	}
	draw({ currentLocation.x, currentLocation.y - friendlyGameboardSize }, 1);

	draw(enemyGameboardOrigin, 0);
	for (unsigned int i = 0; i < 5; i++) {
		draw({ currentLocation.x + enemyGameboardSize, currentLocation.y }, 1);
		draw({ currentLocation.x, currentLocation.y + enemyGameboardSize / 10 }, 1);
		draw({ currentLocation.x - enemyGameboardSize, currentLocation.y }, 1);
		draw({ currentLocation.x, currentLocation.y + enemyGameboardSize / 10 }, 1);
	}
	draw({ currentLocation.x + enemyGameboardSize, currentLocation.y }, 1);

	for (unsigned int i = 0; i < 5; i++) {
		draw({ currentLocation.x, currentLocation.y - enemyGameboardSize }, 1);
		draw({ currentLocation.x - enemyGameboardSize / 10, currentLocation.y }, 1);
		draw({ currentLocation.x, currentLocation.y + enemyGameboardSize }, 1);
		draw({ currentLocation.x - enemyGameboardSize / 10, currentLocation.y }, 1);
	}
	draw({ currentLocation.x, currentLocation.y - enemyGameboardSize }, 1);
	home();
}

void Plotter::g4(const int& row, const int& colom, const int& player) {
    Coordinate origin(0, 0);
    int celSize;
    int radius;
    if (player == 1) {
        celSize = friendlyGameboardSize / 10;
        origin = { int(friendlyGameboardOrigin.x + (row+0.5) * celSize), int(friendlyGameboardOrigin.y + (colom+0.5) * celSize) };
        radius = celSize * 0.4;
    }
    else {
        celSize = enemyGameboardSize / 10;
        origin = { int(enemyGameboardOrigin.x + (row + 0.5) * celSize), int(enemyGameboardOrigin.y + (colom + 0.5) * celSize) };
        radius = celSize * 0.4;
    }
    float PI = 3.14159265358979323846;

	draw(origin, 0);
    for (unsigned int degree = 0; degree <= 360; degree += 45) {
        draw(pointsOnCircle(radius, degree, origin), 1);
        draw(origin, 1);
    }
    home();
}

void Plotter::g5(const int& row, const int& colom, const int& player) {
    Coordinate origin(0, 0);
    int celSize;
    int radius;
    if (player == 1) {
        celSize = friendlyGameboardSize / 10;
        origin = { int(friendlyGameboardOrigin.x + (row + 0.5) * celSize), int(friendlyGameboardOrigin.y + (colom + 0.5) * celSize) };
        radius = celSize * 0.4;
    }
    else {
        celSize = enemyGameboardSize / 10;
        origin = { int(enemyGameboardOrigin.x + (row + 0.5) * celSize), int(enemyGameboardOrigin.y + (colom + 0.5) * celSize) };
        radius = celSize * 0.4;
    }
    float PI = 3.14159265358979323846;

    draw(origin, 0);
    for (unsigned int degree = 0; degree <= 360; degree += 40) {
        draw(pointsOnCircle(radius, degree, origin), 1);
        draw(origin, 1);
    }
    home();
}

void Plotter::g6(const int& row, const int& colom, const int& width, const int& length){
    float PI = 3.14159265358979323846;
	int celSize = friendlyGameboardSize / 10;
    int radius = celSize * 0.4;
    if (width == 1) {
        draw(friendlyGameboardOrigin + Coordinate((row + 0.9) * celSize, (colom + 0.5) * celSize), 0);
        
        draw(friendlyGameboardOrigin + Coordinate((row + 0.9) * celSize, (colom + length - 0.5) * celSize), 1);
        
        for (unsigned int degree = 0; degree <= 180; degree++) {
            draw(pointsOnCircle(radius, degree, Coordinate((row + 0.5) * celSize, (colom + length - 0.5) * celSize)), 1);
        }
        draw(friendlyGameboardOrigin + Coordinate((row + 0.1) * celSize, (colom + 0.5) * celSize), 1);
        for (unsigned int degree = 0; degree <= 180; degree++) {
            draw(pointsOnCircle(radius, 180 + degree, Coordinate((row + 0.5) * celSize, (colom + 0.5) * celSize)), 1);
        }
    }
    else if (length == 1) {
        draw(friendlyGameboardOrigin + Coordinate((row + 0.5) * celSize, (colom + 0.1) * celSize), 0);
        
        draw(friendlyGameboardOrigin + Coordinate((row + width -0.5) * celSize, (colom + 0.1) * celSize), 1);
        for (unsigned int degree = 0; degree <= 180 ; degree++) {
            draw(pointsOnCircle(radius, 270+degree, Coordinate((row + width - 0.5) * celSize, (colom + 0.5) * celSize)), 1);
        }
        draw(friendlyGameboardOrigin + Coordinate((row + 0.5) * celSize, (colom + 0.9) * celSize), 1);
        for (unsigned int degree = 0; degree <= 180; degree++) {
            draw(pointsOnCircle(radius, 90 + degree, Coordinate((row + 0.5) * celSize, (colom + 0.5) * celSize)), 1);
        }
	}
	else {
        // unallowed size
        return;
    }
}
