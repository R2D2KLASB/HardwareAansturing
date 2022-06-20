#include "plotter.hpp"
#include <iostream>

unsigned int absolute(int value) {
    if (value < 0) {
        return -value;
    }
    return value;
}

Coordinate Plotter::pointsOnCircle(const int& radius, const int& angle, const Coordinate& origin) {
    if (angle == 0) {
        return origin + Coordinate(0, radius);
    }
	if (angle == 90) {
		return origin + Coordinate(radius, 0);
	}
	if (angle == 180) {
		return origin + Coordinate(0, -radius);
	}
	if (angle == 270) {
		return origin + Coordinate(-radius, 0);
	}
    float pi = 3.14159265358979323846;
    float radians = (90-angle) * pi / 180;
    return origin + Coordinate(radius * cos(radians), radius * sin(radians));
}

bool Plotter::draw(Coordinate finish, bool draw) {
    if (draw != prevState) {
        setServo(draw);
    }
    if (finish.x > maxDimension.x || finish.y > maxDimension.y || finish.y < 0 || finish.x < 0) {
        return true;
    }
    Coordinate delta = finish - currentLocation;
    Coordinate startLocation = currentLocation;
    float rc = delta.y / (float)delta.x;
    if (delta == Coordinate{ 0, 0 }) {
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
        Coordinate nextLocation = { 0, 0 };
        if (delta.x > 0) {
            if (rc > 0) {
                for (int i = 1; i <= delta.x; i++) {
					nextLocation = startLocation + Coordinate{ i, int(rc * i) };
                    right();
                    currentLocation.x++;
                    for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
                        up();
                        currentLocation.y++;
                    }
                }
            }
            else {
                for (int i = 1; i <= delta.x; i++) {
                    nextLocation = startLocation + Coordinate{ i, int(rc * i) };
                    right();
                    currentLocation.x++;
                    for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
                        down();
                        currentLocation.y--;
                    }
                }
            }
        }
        else {
            if (rc > 0) {
                for (int i = 1; i <= 0 - delta.x; i++) {
                    nextLocation = startLocation + Coordinate{ -i, int(rc * -i) };
                    left();
                    currentLocation.x--;
                    for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
                        down();
                        currentLocation.y--;
                    }
                }
            }
            else {
                for (int i = 1; i <= 0 - delta.x; i++) {
                    nextLocation = startLocation + Coordinate{ -i, int(rc * -i) };
                    left();
                    currentLocation.x--;
                    for (unsigned int i = 0; i < absolute(nextLocation.y - currentLocation.y); i++) {
                        up();
                        currentLocation.y++;
                    }
                }
            }
        }
    }
    return false;

}

bool Plotter::draw(int x, int y, bool d) {
    return draw(Coordinate{ x, y }, d);
}

void Plotter::g3() {
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
        draw({ currentLocation.x - friendlyGameboardSize / 10, currentLocation.y }, 1);
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
    Coordinate target = { row - 1, 10 - colom };
    if (player == 1) {
        celSize = friendlyGameboardSize / 10;
        origin = { int(friendlyGameboardOrigin.x + ((target.x + 0.5) * celSize)), int(friendlyGameboardOrigin.y + ((target.y + 0.5) * celSize)) };
        radius = celSize * 0.4;
    }
    else {
        celSize = enemyGameboardSize / 10;
        origin = { int(enemyGameboardOrigin.x + ((target.x + 0.5) * celSize)), int(enemyGameboardOrigin.y + ((target.y + 0.5) * celSize)) };
        radius = celSize * 0.4;
    }
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

    Coordinate target = { row - 1, 10 - colom };
    if (player == 1) {
        celSize = friendlyGameboardSize / 10;
        origin = { int(friendlyGameboardOrigin.x + ((target.x + 0.5) * celSize)), int(friendlyGameboardOrigin.y + ((target.y + 0.5) * celSize)) };
        radius = celSize * 0.4;
    }
    else {
        celSize = enemyGameboardSize / 10;
        origin = { int(enemyGameboardOrigin.x + ((target.x + 0.5) * celSize)), int(enemyGameboardOrigin.y + ((target.y + 0.5) * celSize)) };
        radius = celSize * 0.4;
    }
    draw(origin, 0);
    for (unsigned int degree = 45; degree <= 360; degree += 90) {
        draw(pointsOnCircle(radius, degree, origin), 1);
        draw(origin, 1);
    }
    home();
}

void Plotter::g6(const int& row, const int& colom, const int& width, const int& length) {
    int celSize = friendlyGameboardSize / 10;
    int radius = celSize * 0.4;
    Coordinate target = { row - 1, 10 - colom };
    if (width == 1) {
        draw(friendlyGameboardOrigin + Coordinate((target.x + 0.9) * celSize, (target.y + 0.5) * celSize), 0);

        for (unsigned int degree = 90; degree <= 270; degree++) {
            draw(pointsOnCircle(radius, degree, Coordinate((target.x + 0.5) * celSize, (target.y - length + 1.5) * celSize)), 1);
        }
        for (unsigned int degree = 270; degree <= 450; degree++) {
            draw(pointsOnCircle(radius, degree, Coordinate((target.x + 0.5) * celSize, (target.y + 0.5) * celSize)), 1);
        }
    }
    else if (length == 1) {
        draw(pointsOnCircle(radius, 0, Coordinate((target.x + 0.5) * celSize, (target.y + 0.5) * celSize)), 0);
        for (unsigned int degree = 0; degree <= 180; degree++) {
            draw(pointsOnCircle(radius, degree, Coordinate((target.x + width - 0.5) * celSize, (target.y + 0.5) * celSize)), 1);
        }
        for (unsigned int degree = 180; degree <= 360; degree++) {
            draw(pointsOnCircle(radius, degree, Coordinate((target.x + 0.5) * celSize, (target.y + 0.5) * celSize)), 1);
        }
    }
    else {
        // unallowed size
        return;
    }
}

void Plotter::g7(const int& row, const int& colom, const int& width, const int& length, const int& player){
    float pi = 3.14159265358979323846;

    int celSize;
	Coordinate origin;
    Coordinate target = { row - 1, 10 - colom };
    if (player == 0) {
        celSize = friendlyGameboardSize / 10;
        origin = friendlyGameboardOrigin;
    }
    else if (player == 1) {
		celSize = enemyGameboardSize / 10;
		origin = enemyGameboardOrigin;
	}
    else {
        return; // unallowed player
    }
    int radius = celSize * 0.4;
    if (width == 1) {
        Coordinate boatOrigin = origin + Coordinate((target.x-0.2) * celSize, target.y * celSize);
        draw(boatOrigin + Coordinate(celSize / 5 * sin(10 * pi / 180), 10), 0);
        for (unsigned int a = 0; a < 3; a++) {
            boatOrigin.x += celSize/3;
            draw(boatOrigin + Coordinate(celSize / 7 * sin(10 * pi / 180), 10), 0);
            for (unsigned int i = 10; i + 10 < length * celSize; i++) {
                draw(boatOrigin + Coordinate(celSize / 10 * sin(i * (500 / (float)celSize) * pi / 180), i), 1);
            }
        }
    }

    else if (length == 1) {
		Coordinate boatOrigin = origin + Coordinate(target.x * celSize, (target.y-0.2) * celSize);
		for (unsigned int a = 0; a < 3; a++) {
			boatOrigin.y += celSize / 3;
			draw(boatOrigin + Coordinate(10, celSize / 10 * sin(0)), 0);
			for (unsigned int i = 10; i + 10 < width * celSize; i++) {
                draw(boatOrigin + Coordinate(i, celSize / 10 * sin(i * (500 / (float)celSize) * pi / 180)), 1);
				}
			}
		}
	else {
		// unallowed size
		return;
	}
}
