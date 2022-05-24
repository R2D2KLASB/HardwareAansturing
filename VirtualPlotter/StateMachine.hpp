#ifndef PROJECT_GAME_STATEMACHINE_HPP
#define PROJECT_GAME_STATEMACHINE_HPP

#include <memory>
#include "GameState.hpp"
#include <stack>
/// @file StateMachine.hpp
/// @brief
/// Project_Game: This is the StateMachine which is handles all gamestate changes


/// @brief
/// type-definition so that std::unique_ptr can be called GameStateReference
typedef std::unique_ptr<GameState> GameStateReference;


///@brief
///StateMachine class that handles all state changes
class StateMachine {
private:
    std::stack<GameStateReference> gameStates;
    GameStateReference newState;
    bool removing;
    bool adding;
    bool replacing;

public:
    /// @brief
    /// Function to add a new gamestate if isReplacing is true the state replaces the current state otherwise it is add on top of the current state.
    /// \param newState GameStateReference containing unique_ptr to the gamestate
    /// \param isReplacing boolean that determines if the state is added on top of the current state or if it replaces the current state
    void addGameState(GameStateReference newState, bool isReplacing = true);

    /// @brief
    /// Remove the current gamestate and continue with the previous one
    void removeGameState();

    /// @brief
    /// function that does the actual changes since addGameState and removeGameState only que the changes
    void processGameStateChanges();

    /// @brief
    /// function to get GameStateReference to the current gamestate
    GameStateReference &getActiveGameState();
};


#endif //PROJECT_GAME_STATEMACHINE_HPP
