#include <bits/stdc++.h>
using namespace std;

/// @file

/**
*   @brief Read the g-code and set it to xy coords.
*   @param coords this is where the coords are saved in.
*   @details Reads the g-codes and saves it line for line in a vector.
*   @return returns new array with shape: g-code number X Y.
*/

vector<string> getXYCoords(string gcodes)
{
    vector<string> coords = {};
    string space_delimiter = " ";
    string newline = "\n";

    size_t pos = 0;
    while ((pos = gcodes.find(newline)) != string::npos) {
      string line = gcodes.substr(0, pos);
      line.erase(std::remove_if(line.begin(), line.end(), []( unsigned char ch ){ if ((ch == 'G') || (ch == 'X' ) || (ch == 'Y' )) return true; else return false; } ), line.end());
      coords.push_back(line);
      gcodes.erase(0, pos + newline.length());
    }

    return coords;
}

int main()
{
    string gcode = "G00 X0.000000 Y2.857143\nG01 X1.428571 Y4.571429\nG01 X3.571429 Y7.428571\nG01 X4.285714 Y8.571428\n";
    vector<string> coords = getXYCoords(gcode);

    for ( auto & coord : coords ){
      cout << coord << endl;
    }

    return 0;
}
