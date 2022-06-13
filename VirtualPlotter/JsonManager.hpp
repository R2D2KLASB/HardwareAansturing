#ifndef PROJECT_GAME_JSONMANAGER_HPP
#define PROJECT_GAME_JSONMANAGER_HPP

#include "Definitions.hpp"
#include "Library/JSON/json/json.h"
#include <fstream>
#include <SFML/Graphics.hpp>
#include <vector>

/**
* @file JsonManager.hpp
* @brief Project_Game: JsonManager used to interact with the Jsonfile
*/


/** @brief JsonManager class used to read from and write to the Jsonfile
*/
class JsonManager {
public:
    /**
    * @brief Constructor
    * @param fileName The name of the Jsonfile
	*/
    JsonManager(const std::string& gameFile);
    
    /**
    * @brief function to retrieve if travels are shown or not
    * @return If the travels are enabled
    */
    bool getShowTravels();
	
	/**
    * @brief function to get the color of the background
    * @return The color of the background
	*/
    sf::Color getBackground();
    
    /**
    * @brief function to get the color of the foreground
    * @return The color of the foreground
    */
    sf::Color getForeground();

	/**
    * @brief function to get the color used when traveling
    * @return The color used when traveling
    */
    sf::Color getTraveling();

	/**
    * @brief function to get the color used when pixel is traveled and drawed
    * @return The color used when pixel is traveled and drawed
	*/
    sf::Color getTravelingDrawed();

	/**
    * @brief function to get the color used for the text
    */
    sf::Color getText();

	/**
    * @brief function to set the color used for the foreground
    * @param newColor The color used for the foreground
	*/
    void setForeground(sf::Color newColor);
    
    /**
    * @brief function to set the color used for the background
    * @param newColor The color used for the background
    */
    void setBackground(sf::Color newColor);

    /**
    * @brief function to enable and disable the display of travels
    * @param newShowTravels The boolean that indicates if travels atre shown or not
    */
    void setShowTravels(bool newShowTravels);

    /**
    * @brief function to set the color used for the foreground
    * @param newColor The color used when the pixel is traveled
	*/
    void setTraveling(sf::Color newColor);

    /**
    * @brief function to set the color used for the foreground
    * @param newColor The color used when the pixel is both traveled and drawed
    */
    void setTravelingDrawed(sf::Color newColor);

    /**
    * @brief function to set the color used for the text
    * @param newColor The color used for the text
    */
    void setText(sf::Color newColor);

    /**
    * @brief Update function that runs on closing of the gamestate that checks if the boolean is set if that is the case it writes the data to the jsonfile.
    */
    void write();

private:
    /**
    * @brief JsonData struct that contains all data read from the Jsonfile
    */
    struct JsonData {
        sf::Color background;
        sf::Color foreground;
        bool show_travels;
        sf::Color traveling;
        sf::Color traveling_drawed;
        sf::Color text;
    };
	/**
    * @brief function that is called by the constructor that calls getJsonFromFile to read the data from the jsonfile and stores the returned data in the JsonData-struct
    */
    void getData();

    JsonData data;
    Json::Value jsonData;

    /**
    * @brief function that turn std::string into sf::color using the colors vector
    */
    sf::Color jsonGetColorFromString(std::string colorString);
	
    /**
    * @brief
    * function that turn sf::color into std::string using the colors vector
    */
    std::string colorToString(const sf::Color &colorSf) const;

    /**
    * @brief function that reads the data from the jsonfile
    * @return all the data in the jsonfile in the Json::Value type which can be converted in integers, boolean, const char* and std::strings
    */
    Json::Value getJsonFromFile();

    /**
    * @brief function that writes the data stored in JsonManager::jsonData to the jsonfile
    */
    void writeJsonToFile();

    std::string gameFile;
    bool writeOut;

    struct Color {
        const std::string colorString;
        const sf::Color colorSf;
    };
};

#endif // PROJECT_GAME_JSONMANAGER_HPP
