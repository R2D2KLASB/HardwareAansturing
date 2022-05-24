#ifndef PROJECT_GAME_INPUTMANAGER_HPP
#define PROJECT_GAME_INPUTMANAGER_HPP

#include <SFML/Graphics.hpp>
#include "Definitions.hpp"
#include <vector>

/// @file InputManager.hpp
/// @brief
/// Project_Game: The input manager is used to handle all kinds of input generated by the user of this application.

/// @brief
/// This class is used to handle input, generated by the user.
class InputManager {
private:
    sf::Cursor handCursor;
    sf::Cursor arrowCursor;
public:

    /// @brief
    /// This constructor constructs an object of the InputManager class.
    InputManager();

    /// @brief
    /// This function checks if a certain keyboard key is pressed.
    /// \param key The key you want to check for.
    ///
    /// \return A boolean for if the given key is pressed or not
    bool isKeyPressed(const sf::Keyboard::Key &key);

    /// @brief
    /// This function checks if a given mouse button is pressed.
    /// \param button The button you want to check.
    ///
    /// \return A boolean for if the given button was pressed.
    bool isButtonPressed(const sf::Mouse::Button &button);

    /// @brief
    /// This function checks if a given sprite is clicked with a certain mouse button.
    /// @details
    /// If the mouse button is clicked, it gets the cursor position on the given window, and checks if that position is inside the sprite boundaries or not.
    /// \param sprite The sprite for which you want to see if it's clicked.
    /// \param button The mouse button you want to check for to determine a click.
    /// \param window The window of which you want to get the cursor position from.
    ///
    /// \return A boolean for if the sprite was clicked or not.
    bool isSpriteClicked(const sf::Sprite &sprite, const sf::Mouse::Button &button, sf::RenderWindow &window);

    /// @brief
    /// This function checks if a given rectangle is clicked with a certain mouse button.
    /// @details
    /// If the mouse button is clicked, it gets the cursor position on the given window, and checks if that position is inside the rectangle's boundaries or not.
    /// \param rectangle The rectangle for which you want to see if it's clicked.
    /// \param button The mouse button you want to check for to determine a click.
    /// \param window The window of which you want to get the cursor position from.
    ///
    /// \return A boolean for if the given rectangle was clicked or not.
    bool isRectangleClicked(const sf::RectangleShape &rectangle, const sf::Mouse::Button &button, sf::RenderWindow &window);

    /// @brief
    /// This function checks if the current mouse position is inside a sprite.
    /// \param sprite The sprite of which you want to check for.
    /// \param window The window on which you want to get the mouse position from.
    ///
    /// \return A boolean for if the mouse position is inside the boundaries of the given sprite.
    bool isMouseIntersectingSprite(const sf::Sprite &sprite, sf::RenderWindow &window);

    /// @brief
    /// This function checks if the current mouse position is inside a sprite.
    /// \param sprite The sprite of which you want to check for.
    /// \param window The window on which you want to get the mouse position from.
    ///
    /// \return A boolean for if the mouse position is inside the boundaries of the given sprite.
    bool isMouseIntersectingSprite(const sf::Sprite *sprite, sf::RenderWindow &window);

    /// @brief
    /// This function changes the mouse cursor.
    /// @details
    /// If the mouse is hovering over a sprite, it will change to a hand cursor, else it will turn into an arrow.
    /// \param buttons A vector with all of the sprites displayed on the screen.
    /// \param window The window on which you want to extract the cursor location from.
    ///
    /// \return A boolean for if the mouse position is inside the boundaries of a sprite.
    bool changeMouseWhenHoveringOverButton(const std::vector<sf::Sprite *> &buttons, sf::RenderWindow &window);

    /// @brief
    /// This function returns the mouse position on the given window.
    /// \param window The window on which you want to get the mouse position of.
    ///
    /// \return A sf::Vector2f containing the float values of the x and y coordinate of the cursor.
    sf::Vector2f getMousePosition(sf::RenderWindow &window);
};


#endif //PROJECT_GAME_INPUTMANAGER_HPP
