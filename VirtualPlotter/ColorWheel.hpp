#pragma once
#include "Game.hpp"

class ColorWheel{
public:
	ColorWheel(GameDataReference gameData, sf::Vector2f position);
	
	~ColorWheel();

	sf::Color getPixel(sf::Vector2f pos, sf::Color currentColor);
	
	sf::FloatRect getGlobalBounds();

	void draw();
	
private:
	GameDataReference gameData;

	sf::Texture texture;
	sf::RectangleShape body;

};

