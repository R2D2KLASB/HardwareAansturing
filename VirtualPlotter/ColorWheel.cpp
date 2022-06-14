#include "ColorWheel.hpp"

ColorWheel::ColorWheel(GameDataReference gameData, sf::Vector2f position):
	gameData(gameData)
{

	texture.loadFromFile("spectrum_chart2.jpg");
	body.setTexture(&texture);
	body.setSize({ 394, 255 });
	body.setOutlineThickness(3);
	body.setOrigin({ body.getSize().x / 2, body.getSize().y / 2 });
	body.setPosition(position);
	
}

ColorWheel::~ColorWheel() {
	
}

bool ColorWheel::isMouseOver(sf::Vector2f mousePosition)
{
	return body.getGlobalBounds().contains(mousePosition);
}

sf::Color ColorWheel::getPixel(sf::Vector2f pos, sf::Color currentColor) {
	pos -= body.getPosition() - body.getSize() / 2.0f;

	if (pos.x < 0 || pos.x > 394)
		return currentColor;

	if (pos.y < 0 || pos.y > 255)
		return currentColor;

	return texture.copyToImage().getPixel(pos.x, pos.y);
}

sf::FloatRect ColorWheel::getGlobalBounds() {
	return body.getGlobalBounds();
}

void ColorWheel::draw() {
	gameData->window.draw(body);
}

