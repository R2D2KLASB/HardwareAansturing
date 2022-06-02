#pragma once
#include "Game.hpp"

class ColorWheel{
public:
	/**
	* @brief Constructor
	* @param game The game object
	* @param position The position of the color wheel
	*/
	ColorWheel(GameDataReference gameData, sf::Vector2f position);
	
	/**
	* @brief Destructor
	*/
	~ColorWheel();

	/**
	* @brief function to get the color of the current pixel in the canvas
	* @details This function is used to get the color of the current pixel in the canvas if clicked out of bounds it will return the param currentColor
	* @param pos The position of the mouse
	* @param currentColor The color of the pixel
	* @return The color of the pixel
	*/
	sf::Color getPixel(sf::Vector2f pos, sf::Color currentColor);
	
	/**
	* @brief function to get the globalBounds of the objects
	* @return The globalBounds of the object
	*/
	sf::FloatRect getGlobalBounds();
	
	/**
	* @brief draw function to draw the object
	*/
	void draw();
	
private:
	GameDataReference gameData;

	sf::Texture texture;
	sf::RectangleShape body;

};

