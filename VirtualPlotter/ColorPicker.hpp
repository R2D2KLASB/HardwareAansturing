#pragma once
#include <SFML/Graphics.hpp>
#include "Game.hpp"
#include "ColorWheel.hpp"

class ColorPicker {
public:
	ColorPicker(GameDataReference gameData, sf::Color& color, sf::Vector2f position, sf::Vector2f size);
	~ColorPicker();
	
	void update(const sf::Vector2f& mousePos);
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

