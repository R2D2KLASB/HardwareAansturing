#include "InputManager.hpp"
InputManager::InputManager() {
    arrowCursor.loadFromSystem(sf::Cursor::Arrow);
    handCursor.loadFromSystem(sf::Cursor::Hand);
}

bool InputManager::isKeyPressed(const sf::Keyboard::Key &key){
    return sf::Keyboard::isKeyPressed(key);
}

bool InputManager::isButtonPressed(const sf::Mouse::Button &button){
    return sf::Mouse::isButtonPressed(button);
}

bool InputManager::isSpriteClicked(const sf::Sprite &sprite, const sf::Mouse::Button &button, sf::RenderWindow& window) {
    if (sf::Mouse::isButtonPressed(button)) {
        return (sprite.getGlobalBounds().contains(getMousePosition(window)));
    }
    return false;
}

bool InputManager::isRectangleClicked(const sf::RectangleShape& rectangle, const sf::Mouse::Button& button, sf::RenderWindow& window) {
    if (sf::Mouse::isButtonPressed(button)) {
        return (rectangle.getGlobalBounds().contains(getMousePosition(window)));
    }
    return false ;
}

bool InputManager::isMouseIntersectingSprite(const sf::Sprite& sprite, sf::RenderWindow& window) {
    return (sprite.getGlobalBounds().contains(getMousePosition(window)));
}

bool InputManager::isMouseIntersectingSprite(const sf::Sprite* sprite, sf::RenderWindow& window) {
    return (sprite->getGlobalBounds().contains(getMousePosition(window)));
}

bool InputManager::changeMouseWhenHoveringOverButton(const std::vector<sf::Sprite*>& buttons, sf::RenderWindow& window) {
    for (const auto& button : buttons) {
        if (isMouseIntersectingSprite(button, window)) {
            window.setMouseCursor(handCursor);
            return true;
        }
    }
    window.setMouseCursor(arrowCursor);
    return false;
}

sf::Vector2f InputManager::getMousePosition(sf::RenderWindow &window) {
    return static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
}