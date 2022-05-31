#include "plotter.hpp"

unsigned int absolute(int value) {
	if (value < 0) {
		return -value;
	}
	return value;
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

void Plotter::g4(const int& row, const int& colom, const int& player){
    Coordinate origin(0, 0);
    int celSize;
    if (player == 1) {
        origin = friendlyGameboardOrigin;
        celSize = friendlyGameboardSize / 10;
    }
    else {
        origin = enemyGameboardOrigin;
        celSize = enemyGameboardSize / 10;
    }

    //Top
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 0);
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 6 * 5), 1);

    //Upper right corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5 * 4, colom * celSize + celSize / 5 * 4), 1);
	
	//Right
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 6 * 5, colom * celSize + celSize / 2), 1);

    //Lower right corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5 * 4, colom * celSize + celSize / 5), 1);
	
	//Lower
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 6), 1);

    //Lower left corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5, colom * celSize + celSize / 5), 1);
	
    //Left
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 6, colom * celSize + celSize / 2), 1);

	//Upper left corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5, colom * celSize + celSize / 5 * 4), 1);

    home();
}

void Plotter::g5(const int& row, const int& colom, const int& player) {
    Coordinate origin(0, 0);
    int celSize;
    if (player == 1) {
        origin = friendlyGameboardOrigin;
        celSize = friendlyGameboardSize / 10;
    }
    else {
        origin = enemyGameboardOrigin;
        celSize = enemyGameboardSize / 10;
    }

    //Upper right corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 0);
    draw(origin + Coordinate(row * celSize + celSize / 5 * 4, colom * celSize + celSize / 5 * 4), 1);

    //Lower right corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5 * 4, colom * celSize + celSize / 5), 1);

    //Lower left corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5, colom * celSize + celSize / 5), 1);

    //Upper left corner
    draw(origin + Coordinate(row * celSize + celSize / 2, colom * celSize + celSize / 2), 1);
    draw(origin + Coordinate(row * celSize + celSize / 5, colom * celSize + celSize / 5 * 4), 1);

    home();
}

void Plotter::g6(const int& row, const int& colom, const int& width, const int& length){
	int min = width * 10;
	if (length < width) {
		min = length*10;
	}
	for (int i = 1; i <= min; i++) {
		draw(friendlyGameboardOrigin + Coordinate(row * (friendlyGameboardSize / 10) + ((friendlyGameboardSize / 10) / 20)*i, colom * (friendlyGameboardSize / 10) + ((friendlyGameboardSize / 10) / 20)*i), 0);
		draw(friendlyGameboardOrigin + Coordinate((row + width) * (friendlyGameboardSize / 10) - ((friendlyGameboardSize / 10) / 20)*i, colom * (friendlyGameboardSize / 10) + ((friendlyGameboardSize / 10) / 20) * i), 1);
		draw(friendlyGameboardOrigin + Coordinate((row + width) * (friendlyGameboardSize / 10) - ((friendlyGameboardSize / 10) / 20)*i, (colom + length) * (friendlyGameboardSize / 10) - ((friendlyGameboardSize / 10) / 20) * i), 1);
		draw(friendlyGameboardOrigin + Coordinate(row * (friendlyGameboardSize / 10) + ((friendlyGameboardSize / 10) / 20)*i, (colom + length) * (friendlyGameboardSize / 10) - ((friendlyGameboardSize / 10) / 20) * i), 1);
		draw(friendlyGameboardOrigin + Coordinate(row * (friendlyGameboardSize / 10) + ((friendlyGameboardSize / 10) / 20)*i, colom * (friendlyGameboardSize / 10) + ((friendlyGameboardSize / 10) / 20) * i), 1);
	}
}
