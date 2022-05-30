#ifndef PROJECT_GAME_ConfigState_HPP
#define PROJECT_GAME_ConfigState_HPP

#include <iostream>
#include <SFML/Graphics.hpp>
#include "StateMachine.hpp"
#include "Definitions.hpp"
#include "Game.hpp"
#include "StateMachine.hpp"
#include "MainState.hpp"
#include <vector>
#include "ColorPicker.hpp"

/// @file ConfigState.hpp
/// @brief
/// Project_Game: This is the ConfigState which is the state you will see after the StartUpState


/// @brief
/// This class is used to create the ConfigState and all of its contents
class ConfigState : public GameState {
private:

    ColorPicker* colorPickerBackground;
    sf::Color background;
    sf::Text backgroundText;
    ColorPicker* colorPickerForeground;
    sf::Color foreground;
    sf::Text foregroundText;
    ColorPicker* colorPickerTraveling;
    sf::Color traveling;
    sf::Text travelingText;
    ColorPicker* colorPickerTravelingDrawed;
    sf::Color travelingDrawed;
	sf::Text travelingDrawedText;
    ColorPicker* colorPickerText;
    sf::Color text;
	sf::Text textText;
    GameDataReference gameData;
	
    sf::Font Textfont;
	
    sf::CircleShape playButton;
public:
    ///\brief
    /// This constructor constructs an object of GameOverState
    /// \param gameData
    ConfigState(GameDataReference gameData);

    ///\brief
    /// This function initializes all of the sounds, textures and set their positions
    void init() override;

    /// @brief
    /// In this function all of the input from the user is checked and the corresponding actions are taken
    void handleInput() override;

    /// @brief
    /// This function Checks if the time elapsed is greater then the START_UP_TIME macro and switches states
    void update() override;

    /// @brief
    /// This function draws all of it's contents onto the screen
    void draw() override;

    ///// @brief
    ///// This function makes necessary changes to variables when this state is recalled.
    //void resume() override;

};

#endif //PROJECT_GAME_ConfigState_HPP
