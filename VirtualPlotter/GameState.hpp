#ifndef PROJECT_GAME_GAMESTATE_HPP
#define PROJECT_GAME_GAMESTATE_HPP

/// @file GameState.hpp
/// @brief
/// Project_Game: abstract state class that`s used as a template for all GameStates

/// @brief
/// Abstract state class that`s used as a template for all GameStates
class GameState {
public:
    /// @brief
    /// init function that`s called first time when state is active in the statemachine
    virtual void init() = 0;

    /// @brief
    /// handleInput function that`s called regularly in the gameloop to handle userinput
    virtual void handleInput() = 0;

    /// @brief
    /// update function that`s called regularly in the gameloop to update the position of the objects on the screen
    virtual void update() {};

    /// draw function that`s called once every loop to draw the object on the screen
    virtual void draw() = 0;

    /// @brief
    /// Function that`s called when new state is appended to statemachine on top of the current gamestate
    virtual void pause() {}

    /// @brief
    /// Function that`s called when the state is resumed after state that was on top of the gamestate is removed
    virtual void resume() {}

};


#endif //PROJECT_GAME_GAMESTATE_HPP
