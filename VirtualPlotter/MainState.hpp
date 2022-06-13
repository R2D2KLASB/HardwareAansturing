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

/**
* @file StartUpState.hpp
* @brief Project_Game: This is the StartUpState which is the first state you will see if the program has started
*/


/**
* @brief This class is used to create the StartUpState and all of its contents
*/
class MainState : public GameState {
private:
    bool showTravels;
    GameDataReference gameData;
    sf::Image image;
    sf::Texture texture;
    sf::Sprite sprite;
    sf::Text statistics;
    sf::Text statistics_data;
    sf::Font textFont;
    sf::Text configText;
	sf::RectangleShape configButton;
    sf::Text exportImageText;
	sf::RectangleShape exportImageButton;
    sf::Text exportImageStatsText;
	sf::RectangleShape exportImageStatsButton;
    sf::Color background;
    sf::Color foreground;
    sf::Color traveling;
    sf::Color traveling_drawed;
    sf::Color text;
    std::vector<std::string> gcodeStrings;
    VirtualPlotter plotter;
    unsigned int speed = 1;
    unsigned int i = 0;

public:
    /**
    * @brief This constructor constructs an object of GameOverState
    * @param gameData
    */
    MainState(GameDataReference gameData);

	/**
    * @brief This function initializes all of the textures and set their positions
    */
    void init() override;
    /**
    * @brief In this function all of the input from the user is checked and the corresponding actions are taken
    */
    void handleInput() override;
    /**
    * @brief This function switches to the MainMenuState.
    */
    void update() override;
    /**
    * @brief This function draws all of it's contents onto the screen
    */
    void draw() override;

};

#endif //PROJECT_GAME_STARTUPSTATE_HPP