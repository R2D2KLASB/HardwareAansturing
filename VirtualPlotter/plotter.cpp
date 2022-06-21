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


void Plotter::drawBoard(Coordinate origin, int size) {
    draw(origin, 0);
    for (unsigned int i = 0; i < 5; i++) {
        draw({ currentLocation.x + size, currentLocation.y }, 1);
        draw({ currentLocation.x, currentLocation.y + size / 10 }, 1);
        draw({ currentLocation.x - size, currentLocation.y }, 1);
        draw({ currentLocation.x, currentLocation.y + size / 10 }, 1);
    }
    draw({ currentLocation.x + size, currentLocation.y }, 1);

    for (unsigned int i = 0; i < 5; i++) {
        draw({ currentLocation.x, currentLocation.y - size }, 1);
        draw({ currentLocation.x - size / 10, currentLocation.y }, 1);
        draw({ currentLocation.x, currentLocation.y + size }, 1);
        draw({ currentLocation.x - size / 10, currentLocation.y }, 1);
    }
    draw({ currentLocation.x, currentLocation.y - size }, 1);

    int celsize = size / 10;
    float scale = (celsize / 250) * 0.15;
    Coordinate character_origin = { origin.x + celsize / 2, origin.y + celsize * 10.5 };

    draw({ 62 * scale + character_origin.x, 83 * scale + character_origin.y }, 0);
    draw({ 0 * scale + character_origin.x, 194 * scale + character_origin.y }, 0);
    draw({ -125 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, 194 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, 138 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 27 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -83 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ -312 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -83 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, -27 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 187 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, -138 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 27 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ -500 * scale + character_origin.x, -11 * scale + character_origin.y }, 0);
    draw({ -357 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -142 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -71 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -71 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -214 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -285 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -357 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -428 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -428 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -357 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -285 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -142 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 71 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 71 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 142 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 214 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 357 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ 136 * scale + character_origin.x, 138 * scale + character_origin.y }, 0);
    draw({ 136 * scale + character_origin.x, 194 * scale + character_origin.y }, 1);
    draw({ 45 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -136 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -318 * scale + character_origin.x, 194 * scale + character_origin.y }, 1);
    draw({ -409 * scale + character_origin.x, 138 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 27 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -83 * scale + character_origin.y }, 1);
    draw({ -409 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ -227 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 45 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 318 * scale + character_origin.x, -138 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 27 * scale + character_origin.y }, 1);


    character_origin += Coordinate{ celsize, 0 };

    draw({ 62 * scale + character_origin.x, 35 * scale + character_origin.y }, 0);
    draw({ 0 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -125 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -312 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, 178 * scale + character_origin.y }, 0);
    draw({ 62 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 187 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ -400 * scale + character_origin.x, -138 * scale + character_origin.y }, 0);
    draw({ -200 * scale + character_origin.x, -83 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, -27 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 194 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -200 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -400 * scale + character_origin.x, 194 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -83 * scale + character_origin.y }, 1);
    draw({ -400 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ -200 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 200 * scale + character_origin.x, -194 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, -138 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 27 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ -115 * scale + character_origin.x, 265 * scale + character_origin.y }, 0);
    draw({ 192 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 346 * scale + character_origin.x, 507 * scale + character_origin.y }, 1);
    draw({ 423 * scale + character_origin.x, 568 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 659 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 719 * scale + character_origin.y }, 1);
    draw({ 423 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 269 * scale + character_origin.x, 719 * scale + character_origin.y }, 1);
    draw({ 192 * scale + character_origin.x, 659 * scale + character_origin.y }, 1);
    draw({ 38 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -192 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ -423 * scale + character_origin.x, -68 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -159 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -219 * scale + character_origin.y }, 1);
    draw({ -423 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -269 * scale + character_origin.x, -219 * scale + character_origin.y }, 1);
    draw({ -192 * scale + character_origin.x, -128 * scale + character_origin.y }, 1);
    draw({ -115 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ -38 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 115 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 269 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ 346 * scale + character_origin.x, 174 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 265 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ 33 * scale + character_origin.x, 416 * scale + character_origin.y }, 0);
    draw({ -233 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 607 * scale + character_origin.y }, 0);
    draw({ 33 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 166 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ -500 * scale + character_origin.x, -11 * scale + character_origin.y }, 0);
    draw({ -366 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 166 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ -71 * scale + character_origin.x, 250 * scale + character_origin.y }, 0);
    draw({ -71 * scale + character_origin.x, 214 * scale + character_origin.y }, 1);
    draw({ 71 * scale + character_origin.x, 214 * scale + character_origin.y }, 1);
    draw({ 71 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -71 * scale + character_origin.x, 250 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -71 * scale + character_origin.y }, 0);
    draw({ -214 * scale + character_origin.x, 71 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -142 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -214 * scale + character_origin.y }, 1);
    draw({ -357 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -214 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 71 * scale + character_origin.x, -214 * scale + character_origin.y }, 1);
    draw({ 214 * scale + character_origin.x, -178 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, -71 * scale + character_origin.y }, 1);

    character_origin += Coordinate{ celsize, 0 };

    draw({ 33 * scale + character_origin.x, 403 * scale + character_origin.y }, 0);
    draw({ 166 * scale + character_origin.x, 557 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, -134 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, -211 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -211 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -134 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -19 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 96 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 173 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 211 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 288 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 403 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ 233 * scale + character_origin.x, 711 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 711 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);

    character_origin = { origin.x - celsize/2, origin.y + celsize * 9.4 };
	
    draw({ 250 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ -375 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, -250 * scale + character_origin.y }, 0);
    draw({ 500 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -125 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);

    character_origin -= {0, celsize};
	
    draw({ 264 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ 382 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 441 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 441 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 382 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 264 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -88 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -147 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ -88 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -147 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -205 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -205 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -147 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -88 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 88 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 264 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 441 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 441 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 323 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 147 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -88 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -264 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -382 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -441 * scale + character_origin.x, -154 * scale + character_origin.y }, 0);
    draw({ -382 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -264 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 29 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 205 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 323 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 382 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 323 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 205 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 29 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -264 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);

    character_origin -= {0, celsize};
	
    draw({ -187 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ -125 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -125 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 187 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 187 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ 375 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, 273 * scale + character_origin.y }, 0);
    draw({ 62 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);

	character_origin -= {0, celsize};

    draw({ 375 * scale + character_origin.x, 702 * scale + character_origin.y }, 0);
    draw({ 0 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, -250 * scale + character_origin.y }, 0);
    draw({ 437 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);

	character_origin -= {0, celsize};
	
    draw({ 29 * scale + character_origin.x, 369 * scale + character_origin.y }, 0);
    draw({ 147 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 205 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 264 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 264 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 205 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 88 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -29 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -88 * scale + character_origin.x, 702 * scale + character_origin.y }, 0);
    draw({ 205 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -88 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -382 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -323 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -147 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 29 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 205 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 264 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 323 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 323 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 264 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 147 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -29 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -205 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -382 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -441 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -441 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -382 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -441 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
	
	character_origin -= {0, celsize};

    draw({ 100 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ -33 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 607 * scale + character_origin.y }, 0);
    draw({ 366 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 166 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, -250 * scale + character_origin.y }, 0);
    draw({ 100 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);

	character_origin -= {0, celsize};
	
    draw({ -433 * scale + character_origin.x, 607 * scale + character_origin.y }, 0);
    draw({ -233 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 654 * scale + character_origin.y }, 0);
    draw({ -233 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ -500 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 464 * scale + character_origin.y }, 0);
    draw({ -100 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
	
	character_origin -= {0, celsize};

    draw({ 62 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ -125 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 62 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -125 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -187 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -125 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -312 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 125 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 437 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 375 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -62 * scale + character_origin.x, 273 * scale + character_origin.y }, 0);
    draw({ -250 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -437 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -250 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -250 * scale + character_origin.y }, 0);
    draw({ 187 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 312 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 250 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);

	character_origin -= {0, celsize};

    draw({ 433 * scale + character_origin.x, 416 * scale + character_origin.y }, 0);
    draw({ 366 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 100 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, 702 * scale + character_origin.y }, 0);
    draw({ 433 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 166 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 226 * scale + character_origin.y }, 0);
    draw({ -300 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -100 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);

	character_origin -= {celsize/8, celsize};

    draw({ -250 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ -875 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -750 * scale + character_origin.x, -250 * scale + character_origin.y }, 0);
    draw({ 0 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -375 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -750 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -1000 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -625 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -125 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 600 * scale + character_origin.x, 750 * scale + character_origin.y }, 0);
    draw({ 400 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 266 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 133 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 66 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 66 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 133 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 266 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 400 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 600 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 733 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 866 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 933 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 1000 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 1000 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 933 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 866 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 733 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 600 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 466 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 333 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 200 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 133 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 66 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 66 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 133 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 266 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 400 * scale + character_origin.x, -250 * scale + character_origin.y }, 0);
    draw({ 533 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 666 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 800 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 866 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 933 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 933 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 866 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 733 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
}

void Plotter::g3() {
    float scale = ((friendlyGameboardSize / 10) / 250) * 0.3;
	
    Coordinate character_origin = { friendlyGameboardOrigin.x + friendlyGameboardSize / 2, friendlyGameboardOrigin.y + friendlyGameboardSize * 1.3 };
    draw({ -2523 * scale + character_origin.x, 654 * scale + character_origin.y }, 0);
    draw({ -2476 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -2476 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -2523 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -2571 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -2619 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -2714 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -2809 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -2904 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -2952 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -3000 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -3000 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -2952 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -2904 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -2857 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -2761 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -2666 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -2571 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -2428 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -2190 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -2095 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -2047 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -2000 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -2000 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -2047 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -2095 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -2190 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -2333 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -2428 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -2476 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -1777 * scale + character_origin.x, 178 * scale + character_origin.y }, 0);
    draw({ -1666 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -1444 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -1277 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -1166 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ -1111 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -1111 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -1166 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -1277 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -1333 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -1388 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -1444 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -1500 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -1555 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -1611 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -1722 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -1833 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -1944 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -2000 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -2000 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -1944 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -1833 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -1722 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -1555 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -1388 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -1277 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -1111 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -1000 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -888 * scale + character_origin.x, -202 * scale + character_origin.y }, 0);
    draw({ -888 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -722 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -555 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -333 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -222 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -333 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ -444 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ -611 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -722 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -777 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -777 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -722 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -611 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -444 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -277 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 157 * scale + character_origin.x, 537 * scale + character_origin.y }, 0);
    draw({ 52 * scale + character_origin.x, 568 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 628 * scale + character_origin.y }, 1);
    draw({ 0 * scale + character_origin.x, 659 * scale + character_origin.y }, 1);
    draw({ 52 * scale + character_origin.x, 719 * scale + character_origin.y }, 1);
    draw({ 157 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 210 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 315 * scale + character_origin.x, 719 * scale + character_origin.y }, 1);
    draw({ 368 * scale + character_origin.x, 659 * scale + character_origin.y }, 1);
    draw({ 368 * scale + character_origin.x, 598 * scale + character_origin.y }, 1);
    draw({ 315 * scale + character_origin.x, 477 * scale + character_origin.y }, 1);
    draw({ 263 * scale + character_origin.x, 386 * scale + character_origin.y }, 1);
    draw({ 210 * scale + character_origin.x, 265 * scale + character_origin.y }, 1);
    draw({ 210 * scale + character_origin.x, 204 * scale + character_origin.y }, 1);
    draw({ 263 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ 315 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 421 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 526 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ 631 * scale + character_origin.x, 204 * scale + character_origin.y }, 1);
    draw({ 736 * scale + character_origin.x, 295 * scale + character_origin.y }, 1);
    draw({ 789 * scale + character_origin.x, 356 * scale + character_origin.y }, 1);
    draw({ 894 * scale + character_origin.x, 537 * scale + character_origin.y }, 1);
    draw({ 1000 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 894 * scale + character_origin.x, 537 * scale + character_origin.y }, 0);
    draw({ 736 * scale + character_origin.x, 234 * scale + character_origin.y }, 1);
    draw({ 631 * scale + character_origin.x, 53 * scale + character_origin.y }, 1);
    draw({ 526 * scale + character_origin.x, -98 * scale + character_origin.y }, 1);
    draw({ 421 * scale + character_origin.x, -219 * scale + character_origin.y }, 1);
    draw({ 315 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 263 * scale + character_origin.x, -219 * scale + character_origin.y }, 1);
    draw({ 263 * scale + character_origin.x, -159 * scale + character_origin.y }, 1);
    draw({ 315 * scale + character_origin.x, -68 * scale + character_origin.y }, 1);
    draw({ 421 * scale + character_origin.x, 22 * scale + character_origin.y }, 1);
    draw({ 578 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 736 * scale + character_origin.x, 174 * scale + character_origin.y }, 1);
    draw({ 1000 * scale + character_origin.x, 265 * scale + character_origin.y }, 1);
    draw({ 1733 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ 1733 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 1800 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 1933 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 2000 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 2000 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 1933 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 1733 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1466 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1266 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 1200 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 1200 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 1266 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ 1333 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 1533 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 1333 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 1133 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 1066 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 1000 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 1000 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 1066 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 1133 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 1333 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 1533 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 1733 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 1866 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 1933 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 2500 * scale + character_origin.x, 654 * scale + character_origin.y }, 0);
    draw({ 2550 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 2550 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 2500 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 2450 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 2400 * scale + character_origin.x, 35 * scale + character_origin.y }, 1);
    draw({ 2300 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 2200 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 2100 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 2050 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 2000 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 2000 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 2050 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ 2100 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 2150 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 2250 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 2350 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 2450 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 2600 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 2800 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 2900 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 2950 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 3000 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 3000 * scale + character_origin.x, 416 * scale + character_origin.y }, 1);
    draw({ 2950 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 2900 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 2800 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 2650 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 2500 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 2550 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ 2600 * scale + character_origin.x, 130 * scale + character_origin.y }, 1);
    draw({ 2600 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 2650 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 2750 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 2850 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 2900 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 3000 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);


	drawBoard(friendlyGameboardOrigin, friendlyGameboardSize);
    
    scale = ((enemyGameboardSize / 10) / 250) * 0.3;

    character_origin = { enemyGameboardOrigin.x + enemyGameboardSize / 2, enemyGameboardOrigin.y + enemyGameboardSize * 1.3 };
	
    draw({ -1766 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ -1766 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -1700 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -1566 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -1500 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -1500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -1566 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -1766 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -2033 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -2233 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -2300 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -2300 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -2233 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -2166 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ -1966 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -2166 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -2366 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -2433 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -2500 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -2500 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -2433 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ -2366 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -2166 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -1966 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -1766 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -1633 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ -1566 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -1272 * scale + character_origin.x, 83 * scale + character_origin.y }, 0);
    draw({ -1136 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -1045 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ -1000 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -909 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -818 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -727 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -681 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -681 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -727 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -818 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -818 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -772 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -727 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ -636 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -590 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ -1363 * scale + character_origin.x, 416 * scale + character_origin.y }, 0);
    draw({ -1454 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -1500 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ -1500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -1454 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -1363 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -1318 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -1227 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -1181 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -1181 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ -1227 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -1272 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -1363 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 559 * scale + character_origin.y }, 0);
    draw({ 233 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 300 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -33 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ -300 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ -233 * scale + character_origin.x, 369 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 321 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, 226 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, 178 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ -500 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ -433 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ -366 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ -166 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 33 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 233 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 366 * scale + character_origin.x, -107 * scale + character_origin.y }, 1);
    draw({ 433 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 983 * scale + character_origin.x, 83 * scale + character_origin.y }, 0);
    draw({ 1080 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 1145 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 1177 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 1241 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1274 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1338 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 1370 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 1370 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 1338 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 1274 * scale + character_origin.x, -59 * scale + character_origin.y }, 1);
    draw({ 1274 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 1306 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 1338 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 1403 * scale + character_origin.x, -202 * scale + character_origin.y }, 1);
    draw({ 1435 * scale + character_origin.x, -154 * scale + character_origin.y }, 1);
    draw({ 1500 * scale + character_origin.x, -11 * scale + character_origin.y }, 1);
    draw({ 596 * scale + character_origin.x, 416 * scale + character_origin.y }, 0);
    draw({ 532 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 559 * scale + character_origin.y }, 1);
    draw({ 500 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 532 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 596 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 629 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 693 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 725 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 725 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 693 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 661 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 596 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 661 * scale + character_origin.x, 83 * scale + character_origin.y }, 0);
    draw({ 758 * scale + character_origin.x, 464 * scale + character_origin.y }, 1);
    draw({ 822 * scale + character_origin.x, 654 * scale + character_origin.y }, 1);
    draw({ 854 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 919 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 951 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1016 * scale + character_origin.x, 702 * scale + character_origin.y }, 1);
    draw({ 1048 * scale + character_origin.x, 607 * scale + character_origin.y }, 1);
    draw({ 1048 * scale + character_origin.x, 511 * scale + character_origin.y }, 1);
    draw({ 1016 * scale + character_origin.x, 273 * scale + character_origin.y }, 1);
    draw({ 983 * scale + character_origin.x, 83 * scale + character_origin.y }, 1);
    draw({ 919 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 1657 * scale + character_origin.x, 537 * scale + character_origin.y }, 0);
    draw({ 1552 * scale + character_origin.x, 568 * scale + character_origin.y }, 1);
    draw({ 1500 * scale + character_origin.x, 628 * scale + character_origin.y }, 1);
    draw({ 1500 * scale + character_origin.x, 659 * scale + character_origin.y }, 1);
    draw({ 1552 * scale + character_origin.x, 719 * scale + character_origin.y }, 1);
    draw({ 1657 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1710 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 1815 * scale + character_origin.x, 719 * scale + character_origin.y }, 1);
    draw({ 1868 * scale + character_origin.x, 659 * scale + character_origin.y }, 1);
    draw({ 1868 * scale + character_origin.x, 598 * scale + character_origin.y }, 1);
    draw({ 1815 * scale + character_origin.x, 477 * scale + character_origin.y }, 1);
    draw({ 1763 * scale + character_origin.x, 386 * scale + character_origin.y }, 1);
    draw({ 1710 * scale + character_origin.x, 265 * scale + character_origin.y }, 1);
    draw({ 1710 * scale + character_origin.x, 204 * scale + character_origin.y }, 1);
    draw({ 1763 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ 1815 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 1921 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 2026 * scale + character_origin.x, 143 * scale + character_origin.y }, 1);
    draw({ 2131 * scale + character_origin.x, 204 * scale + character_origin.y }, 1);
    draw({ 2236 * scale + character_origin.x, 295 * scale + character_origin.y }, 1);
    draw({ 2289 * scale + character_origin.x, 356 * scale + character_origin.y }, 1);
    draw({ 2394 * scale + character_origin.x, 537 * scale + character_origin.y }, 1);
    draw({ 2500 * scale + character_origin.x, 750 * scale + character_origin.y }, 1);
    draw({ 2394 * scale + character_origin.x, 537 * scale + character_origin.y }, 0);
    draw({ 2236 * scale + character_origin.x, 234 * scale + character_origin.y }, 1);
    draw({ 2131 * scale + character_origin.x, 53 * scale + character_origin.y }, 1);
    draw({ 2026 * scale + character_origin.x, -98 * scale + character_origin.y }, 1);
    draw({ 1921 * scale + character_origin.x, -219 * scale + character_origin.y }, 1);
    draw({ 1815 * scale + character_origin.x, -250 * scale + character_origin.y }, 1);
    draw({ 1763 * scale + character_origin.x, -219 * scale + character_origin.y }, 1);
    draw({ 1763 * scale + character_origin.x, -159 * scale + character_origin.y }, 1);
    draw({ 1815 * scale + character_origin.x, -68 * scale + character_origin.y }, 1);
    draw({ 1921 * scale + character_origin.x, 22 * scale + character_origin.y }, 1);
    draw({ 2078 * scale + character_origin.x, 113 * scale + character_origin.y }, 1);
    draw({ 2236 * scale + character_origin.x, 174 * scale + character_origin.y }, 1);
    draw({ 2500 * scale + character_origin.x, 265 * scale + character_origin.y }, 1);

	
    drawBoard(enemyGameboardOrigin, enemyGameboardSize);
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
    if (player == 1) {
        celSize = friendlyGameboardSize / 10;
        origin = friendlyGameboardOrigin;
    }
    else if (player == 0) {
		celSize = enemyGameboardSize / 10;
		origin = enemyGameboardOrigin;
	}
    else {
        return; // unallowed player
    }
    int radius = celSize * 0.4;
    if (width == 1) {
        Coordinate boatOrigin = origin + Coordinate((target.x-0.2) * celSize, (target.y+1) * celSize);
        draw(boatOrigin + Coordinate(celSize / 5 * sin(10 * pi / 180), 10), 0);
        for (unsigned int a = 0; a < 3; a++) {
            boatOrigin.x += celSize/3;
            draw(boatOrigin + Coordinate(celSize / 7 * sin(10 * pi / 180), 10), 0);
            for (unsigned int i = 10; i + 10 < length * celSize; i++) {
                draw(boatOrigin - Coordinate(celSize / 10 * sin(i * (500 / (float)celSize) * pi / 180), i), 1);
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
