#ifndef PROJECT_GAME_STARTUPSTATE_HPP
#define PROJECT_GAME_STARTUPSTATE_HPP

#include <iostream>
#include <fstream>
#include <SFML/Graphics.hpp>
#include "StateMachine.hpp"
#include "Definitions.hpp"
#include "Exceptions.hpp"
#include "Game.hpp"
#include "VirtualPlotter.hpp"

///@file StartUpState.hpp
/// @brief
/// Project_Game: This is the StartUpState which is the first state you will see if the program has started


/// @brief
/// This class is used to create the StartUpState and all of its contents
class MainState : public GameState {
private:
    GameDataReference gameData;
    sf::Image image;
    sf::Texture texture;
    sf::Sprite sprite;
    sf::Text statistics;
    sf::Text statistics_data;
    sf::Font statistics_font;
    sf::Color background = gameData->jsonManager.getBackground();
    sf::Color foreground = gameData->jsonManager.getForeground();
    sf::Color traveling = gameData->jsonManager.getTraveling();
    sf::Color traveling_drawed = gameData->jsonManager.getTravelingDrawed();
    sf::Color text = gameData->jsonManager.getText();
    std::vector<std::vector<std::string>> gcode;
    VirtualPlotter plotter = VirtualPlotter(foreground, background, traveling, traveling_drawed);
    unsigned int speed = 1;
    unsigned int i = 0;

public:
    ///\brief
    /// This constructor constructs an object of GameOverState
    /// \param gameData
    MainState(GameDataReference gameData);

    ///\brief
    /// This function initializes all of the textures and set their positions
    void init() override;

    /// @brief
    /// In this function all of the input from the user is checked and the corresponding actions are taken
    void handleInput() override;

    /// @brief
    /// This function switches to the MainMenuState.
    void update() override;

    /// @brief
    /// This function draws all of it's contents onto the screen
    void draw() override;

};

#endif //PROJECT_GAME_STARTUPSTATE_HPP