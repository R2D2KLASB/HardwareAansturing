#pragma once
#include <SFML/Graphics.hpp>
#include "Game.hpp"
#include "ColorWheel.hpp"

class ColorPicker {
public:
	/** 
		@brief Constructor for the ColorPicker class
		@param game A pointer to the game object
		@param position The position of the color picker
		@param size The size of the color picker
	*/
	ColorPicker(GameDataReference gameData, sf::Color& color, sf::Vector2f position, sf::Vector2f size);
	/**
	* @brief Destructor for the ColorPicker class
	*/
	~ColorPicker();
	
	/**
	* @brief Updates the color picker
	* @param mousePos The current mouse position
	*/
	void update(const sf::Vector2f& mousePos);
	/**
	* @brief Draws the color picker
	*/
	void draw();
	
private:
	GameDataReference gameData;

	sf::RectangleShape body;

	sf::Color& color;
	
	ColorWheel* colorWheel;

	bool keyCheck = false;
	
	bool wheelVisible = false;
	
	void InitBody(const sf::Vector2f position, const sf::Vector2f size);
};

