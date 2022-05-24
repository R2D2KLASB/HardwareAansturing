#include "ColorPicker.hpp"

ColorPicker::ColorPicker(GameDataReference gameData, sf::Color& color, sf::Vector2f position, sf::Vector2f size) :
	gameData(gameData),
	color(color)
{
	InitBody(position, size);
	colorWheel = new ColorWheel(gameData, position);

}
ColorPicker::~ColorPicker() {
	delete colorWheel;
}

void ColorPicker::update(const sf::Vector2f& mousePos) {
	if (wheelVisible) {
		if (sf::Mouse::isButtonPressed(sf::Mouse::Left) && !keyCheck) {
			wheelVisible = false;
			if(colorWheel->getGlobalBounds().contains(mousePos)) {
				color = colorWheel->getPixel(mousePos, color);
				body.setFillColor(color);
			}
		}
	}
	else {
		if (body.getGlobalBounds().contains(mousePos) && sf::Mouse::isButtonPressed(sf::Mouse::Left) && !keyCheck) {
			wheelVisible = true;
		}
	}

	keyCheck = sf::Mouse::isButtonPressed(sf::Mouse::Left);
}
void ColorPicker::draw() {
	if (wheelVisible) {
		colorWheel->draw();
	}
	else {
		gameData->window.draw(body);
	}
}

void ColorPicker::InitBody(const sf::Vector2f position, const sf::Vector2f size) {
	body.setSize(size);
	body.setOrigin({size.x/2, size.y/2});
	body.setPosition(position);
	body.setFillColor(color);
	body.setOutlineThickness(3);
}