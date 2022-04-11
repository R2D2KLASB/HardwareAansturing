#include <Arduino.h>

/// @file

/**
* @brief virtual class implemented by the delta and XY-plotter
*/
class Plotter{
    public:
        /**
        * @brief constructer for the Plotter class
        * @details constructer for the Plotter class that also handles all the setup steps
        */
        Plotter(){};
        
        /**
        * @brief virtual to draw the gcode
        * @param target the targetlocation the plotter is going to move to
        * @param draw boolean if the plotter has to draw a line between the current and target location
        * @details virtual function implemented by the delta and XY-plotter to draw the line
        * @return returns a boolean if the command succeeded or failed
        */
         virtual bool draw(Coordinate target, bool draw);

    private:
        bool lift;
};
