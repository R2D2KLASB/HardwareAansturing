#include "ConfigState.hpp"
#include <utility>

ConfigState::ConfigState(GameDataReference gameData) :
	gameData(gameData)
{}


void ConfigState::init() { 
	background = gameData->jsonManager.getBackground();
	foreground = gameData->jsonManager.getForeground();
	traveling = gameData->jsonManager.getTraveling();
	travelingDrawed = gameData->jsonManager.getTravelingDrawed();
	text = gameData->jsonManager.getText();
	colorPickerBackground = new ColorPicker(gameData, background, { SCREEN_WIDTH/6 * 1, SCREEN_HEIGHT /3 * 2 }, { 100, 100 });
	colorPickerForeground = new ColorPicker(gameData, foreground, { SCREEN_WIDTH / 6 * 2, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerTraveling = new ColorPicker(gameData, traveling, { SCREEN_WIDTH / 6 * 3, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerTravelingDrawed = new ColorPicker(gameData, travelingDrawed, { SCREEN_WIDTH/6*4, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerText = new ColorPicker(gameData, text, { SCREEN_WIDTH/6*5, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	
	
	Textfont.loadFromFile(BIT_FONT_PATH);
	

	backgroundText.setFont(Textfont);
	backgroundText.setString("Background");
	backgroundText.setCharacterSize(20);
	backgroundText.setFillColor(sf::Color::White);
	backgroundText.setPosition(SCREEN_WIDTH / 6 * 1, SCREEN_HEIGHT / 3 * 2 - 100);
	backgroundText.setOrigin(backgroundText.getGlobalBounds().width / 2, backgroundText.getGlobalBounds().height / 2);
	
	foregroundText.setFont(Textfont);
	foregroundText.setString("Foreground");
	foregroundText.setCharacterSize(20);
	foregroundText.setFillColor(sf::Color::White);
	foregroundText.setPosition(SCREEN_WIDTH / 6 * 2, SCREEN_HEIGHT / 3 * 2 - 100);
	foregroundText.setOrigin(foregroundText.getGlobalBounds().width / 2, foregroundText.getGlobalBounds().height / 2);
	
	travelingText.setFont(Textfont);
	travelingText.setString("Traveling");
	travelingText.setCharacterSize(20);
	travelingText.setFillColor(sf::Color::White);
	travelingText.setPosition(SCREEN_WIDTH / 6 * 3, SCREEN_HEIGHT / 3 * 2 - 100);
	travelingText.setOrigin(travelingText.getGlobalBounds().width / 2, travelingText.getGlobalBounds().height / 2);

	travelingDrawedText.setFont(Textfont);
	travelingDrawedText.setString("Traveling Drawed");
	travelingDrawedText.setCharacterSize(20);
	travelingDrawedText.setFillColor(sf::Color::White);
	travelingDrawedText.setPosition(SCREEN_WIDTH / 6 * 4, SCREEN_HEIGHT / 3 * 2 - 100);
	travelingDrawedText.setOrigin(travelingDrawedText.getGlobalBounds().width / 2, travelingDrawedText.getGlobalBounds().height / 2);

	textText.setFont(Textfont);
	textText.setString("Text");
	textText.setCharacterSize(20);
	textText.setFillColor(sf::Color::White);
	textText.setPosition(SCREEN_WIDTH / 6 * 5, SCREEN_HEIGHT / 3 * 2 - 100);
	textText.setOrigin(textText.getGlobalBounds().width / 2, textText.getGlobalBounds().height / 2);
	

	playButton = sf::CircleShape(60, 3);
	playButton.setFillColor(sf::Color::White);
	playButton.setOutlineColor(sf::Color::Red);
	playButton.setOutlineThickness(5);
	playButton.setPosition(sf::Vector2f(gameData->window.getSize().x / 2-18, gameData->window.getSize().y / 6 * 5));
	playButton.setOrigin(playButton.getRadius(), playButton.getRadius());
	playButton.setRotation(90);
}

void ConfigState::handleInput() {
	sf::Event event{};
	while (gameData->window.pollEvent(event)) {
		if (sf::Event::Closed == event.type) {
			gameData->window.close();
		}
		if (sf::Event::MouseButtonPressed == event.type) {
			if (event.mouseButton.button == sf::Mouse::Left) {
				if (playButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
					gameData->jsonManager.setBackground(background);
					gameData->jsonManager.setForeground(foreground);
					gameData->jsonManager.setTraveling(traveling);
					gameData->jsonManager.setTravelingDrawed(travelingDrawed);
					gameData->jsonManager.setText(text);
					gameData->jsonManager.write();



					gameData->machine.addGameState(GameStateReference(new MainState(gameData)));
				}
			}
		}
	}
}

void ConfigState::update() {
	colorPickerBackground->update(sf::Vector2f(sf::Mouse::getPosition(gameData->window)));
	colorPickerForeground->update(sf::Vector2f(sf::Mouse::getPosition(gameData->window)));
	colorPickerTraveling->update(sf::Vector2f(sf::Mouse::getPosition(gameData->window)));
	colorPickerTravelingDrawed->update(sf::Vector2f(sf::Mouse::getPosition(gameData->window)));
	colorPickerText->update(sf::Vector2f(sf::Mouse::getPosition(gameData->window)));

	
    //gameData->machine.addGameState(GameStateReference(new MainState(gameData)));
}

void ConfigState::draw() {
    gameData->window.clear();

	gameData->window.draw(backgroundText);
	gameData->window.draw(foregroundText);
	gameData->window.draw(travelingText);
	gameData->window.draw(travelingDrawedText);
	gameData->window.draw(textText);

	colorPickerBackground->draw();
	colorPickerForeground->draw();
	colorPickerTraveling->draw();
	colorPickerTravelingDrawed->draw();
	colorPickerText->draw();
	gameData->window.draw(playButton);

    gameData->window.display();
}
