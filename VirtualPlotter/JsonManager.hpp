#ifndef PROJECT_GAME_JSONMANAGER_HPP
#define PROJECT_GAME_JSONMANAGER_HPP

#include "Definitions.hpp"
#include "Library/JSON/json/json.h"
#include <fstream>
#include <SFML/Graphics.hpp>
#include <vector>

///@file JsonManager.hpp
///@brief
/// Project_Game: JsonManager used to interact with the Jsonfile



///@brief
/// JsonManager class used to read from and write to the Jsonfile
class JsonManager {
public:
    JsonManager(const std::string& gameFile);
    
    sf::Color getForeground();

    sf::Color getBackground();

    sf::Color getTraveling();

    sf::Color getTravelingDrawed();

    sf::Color getText();

    void setForeground(sf::Color newColor);

    void setBackground(sf::Color newColor);

    void setTraveling(sf::Color newColor);

    void setTravelingDrawed(sf::Color newColor);

    void setText(sf::Color newColor);


    /// @brief
    /// Update function that runs on closing of the game that checks if the boolean is set if that is the case it writes the data to the jsonfile.
    void write();

private:
    /// @brief
    /// JsonData struct that contains all data read from the Jsonfile
    struct JsonData {
        sf::Color background;
        sf::Color foreground;
        sf::Color traveling;
        sf::Color traveling_drawed;
        sf::Color text;
    };
    /// @brief
    /// function that is called by the constructor that calls getJsonFromFile to read the data from the jsonfile and stores the returned data in the JsonData-struct
    void getData();

    JsonData data;
    Json::Value jsonData;


    /// @brief
    /// function that turn std::string into sf::color using the colors vector
    sf::Color stringToColor(const std::string &colorString) const;

    /// @brief
    /// function that turn sf::color into std::string using the colors vector
    std::string colorToString(const sf::Color &colorSf) const;

    /// @brief
    /// function that reads the data from the jsonfile
    /// \return all the data in the jsonfile in the Json::Value type which can be converted in integers, boolean, const char* and std::strings
    Json::Value getJsonFromFile();

    /// @brief
    /// function that writes the data stored in JsonManager::jsonData to the jsonfile
    void writeJsonToFile();

    std::string gameFile;
    bool writeOut;

    struct Color {
        const std::string colorString;
        const sf::Color colorSf;
    };
    const std::vector<Color> colors = { {"Black",   sf::Color::Black},
                                       {"White",   sf::Color::White},
                                       {"Red",     sf::Color::Red},
                                       {"Green",   sf::Color::Green},
                                       {"Blue",    sf::Color::Blue},
                                       {"Yellow",  sf::Color::Yellow},
                                       {"Magenta", sf::Color::Magenta},
                                       {"Cyan",    sf::Color::Cyan}};

};

#endif // PROJECT_GAME_JSONMANAGER_HPP
