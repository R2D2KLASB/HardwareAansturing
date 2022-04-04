#include <string>

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
        Plotter();
        
        /**
        * @brief virtual to draw the gcode
        * @param gcode the gcode-command to be plotted
        * @details virtual function implemented by the delta and XY-plotter to draw the gcode-command
        * @return returns a boolean if the command succeeded or failed
        */
        virtual bool plot(const std::string& gcode);

    private:
        bool draw;
};