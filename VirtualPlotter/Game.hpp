#ifndef PROJECT_GAME_GAME_HPP
#define PROJECT_GAME_GAME_HPP

#include <SFML/Graphics.hpp>
#include <memory>
#include "StateMachine.hpp"
#include "Definitions.hpp"
#include "InputManager.hpp"
#include "JsonManager.hpp"

/**
* @file Game.hpp
* @brief Project_Game: Game class that constructs all the machines, the window and contains the gameloop
*/

/**
* @brief struct that contains all the Machines, the window and the score.
*/
struct GameData {
    StateMachine machine;
    sf::RenderWindow window;
    JsonManager jsonManager = JsonManager(JSON_FILE);
    InputManager input;
};

/**
* @brief type-definition to define std::shared_prt<GameData> as GameDataReference
*/
typedef std::shared_ptr<GameData> GameDataReference;


/**
* @brief Class that creates the window and contains the gameloop
*/
class Game {
private:
    const float delta = 1.0 / FRAMERATE;
    sf::Clock clock;
    GameDataReference gameData = std::make_shared<GameData>();

    void start();

public:
    /**
    * @brief constructor that creates the window, and loads the StartUpstate into the statemachine.
    * @param screenWidth The width of the renderwindow
    * @param screenHeight The height of the renderwindow
    * @param gameTitle The title of the game shown in the menubar of the window
    */
    Game(const int &screenWidth, const int &screenHeight, const std::string &gameTitle);
};


#endif //PROJECT_GAME_GAME_HPP
