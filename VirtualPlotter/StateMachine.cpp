#include "StateMachine.hpp"

void StateMachine::addGameState(GameStateReference newState_, bool isReplacing) {
    adding = true;
    replacing = isReplacing;
    newState = std::move(newState_);
}

void StateMachine::removeGameState() {
    removing = true;
}

void StateMachine::processGameStateChanges() {
    if (removing and !gameStates.empty()) {
        gameStates.pop();
        if (!gameStates.empty()) {
            gameStates.top()->resume();
        }
        removing = false;
    }
    if (adding){
        if(!gameStates.empty()){
            if(replacing){
                gameStates.pop();
            }
            else{
                gameStates.top()->pause();
            }
        }
        gameStates.push(std::move(newState));
        gameStates.top()->init();
        adding=false;
    }
}

GameStateReference &StateMachine::getActiveGameState() {
    return gameStates.top();
}