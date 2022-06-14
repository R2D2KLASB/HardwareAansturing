#include "ConfigState.hpp"
#include <utility>

ConfigState::ConfigState(GameDataReference gameData) :
	gameData(gameData)
{}


void ConfigState::init() {
	showTravels = gameData->jsonManager.getShowTravels();

	background = gameData->jsonManager.getBackground();
	foreground = gameData->jsonManager.getForeground();
	traveling = gameData->jsonManager.getTraveling();
	travelingDrawed = gameData->jsonManager.getTravelingDrawed();
	text = gameData->jsonManager.getText();
	colorPickerBackground = new ColorPicker(gameData, background, { SCREEN_WIDTH / 7 * 1, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerForeground = new ColorPicker(gameData, foreground, { SCREEN_WIDTH / 7 * 2, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerTraveling = new ColorPicker(gameData, traveling, { SCREEN_WIDTH / 7 * 3, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerTravelingDrawed = new ColorPicker(gameData, travelingDrawed, { SCREEN_WIDTH / 7 * 4, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });
	colorPickerText = new ColorPicker(gameData, text, { SCREEN_WIDTH / 7 * 5, SCREEN_HEIGHT / 3 * 2 }, { 100, 100 });

	Textfont.loadFromFile(BIT_FONT_PATH);



	backgroundText.setFont(Textfont);
	backgroundText.setString("Background");
	backgroundText.setCharacterSize(20);
	backgroundText.setFillColor(sf::Color::White);
	backgroundText.setPosition(SCREEN_WIDTH / 7 * 1, SCREEN_HEIGHT / 3 * 2 - 100);
	backgroundText.setOrigin(backgroundText.getGlobalBounds().width / 2, backgroundText.getGlobalBounds().height / 2);

	foregroundText.setFont(Textfont);
	foregroundText.setString("Foreground");
	foregroundText.setCharacterSize(20);
	foregroundText.setFillColor(sf::Color::White);
	foregroundText.setPosition(SCREEN_WIDTH / 7 * 2, SCREEN_HEIGHT / 3 * 2 - 100);
	foregroundText.setOrigin(foregroundText.getGlobalBounds().width / 2, foregroundText.getGlobalBounds().height / 2);

	showTravelsText.setFont(Textfont);
	showTravelsText.setString("Show travels:");
	showTravelsText.setCharacterSize(20);
	showTravelsText.setFillColor(sf::Color::White);
	showTravelsText.setPosition(SCREEN_WIDTH / 7 * 6, SCREEN_HEIGHT / 3 * 2 - 100);
	showTravelsText.setOrigin(showTravelsText.getGlobalBounds().width / 2, showTravelsText.getGlobalBounds().height / 2);
	
	showTravelsStateText.setFont(Textfont);
	showTravelsStateText.setCharacterSize(20);
	showTravelsStateText.setPosition(SCREEN_WIDTH / 7 * 6, SCREEN_HEIGHT / 3 * 2);
	showTravelsStateText.setFillColor(sf::Color::White);


	if (showTravels) {
		showTravelsStateText.setString("ON");
		showTravelsStateText.setOrigin(showTravelsStateText.getGlobalBounds().width / 2, showTravelsStateText.getGlobalBounds().height / 2 + 5);
		showTravelsButton.setFillColor(sf::Color::Green);

	}
	else {
		showTravelsStateText.setString("OFF");
		showTravelsStateText.setOrigin(showTravelsStateText.getGlobalBounds().width / 2, showTravelsStateText.getGlobalBounds().height / 2 + 5);
		showTravelsButton.setFillColor(sf::Color::Red);
	}

	showTravelsButton.setSize({ 100, 100 });
	showTravelsButton.setPosition(SCREEN_WIDTH / 7 * 6, SCREEN_HEIGHT / 3 * 2);
	showTravelsButton.setOrigin(showTravelsButton.getGlobalBounds().width / 2, showTravelsButton.getGlobalBounds().height / 2);
	showTravelsButton.setOutlineThickness(3);


	travelingText.setFont(Textfont);
	travelingText.setString("Traveling");
	travelingText.setCharacterSize(20);
	travelingText.setFillColor(sf::Color::White);
	travelingText.setPosition(SCREEN_WIDTH / 7 * 3, SCREEN_HEIGHT / 3 * 2 - 100);
	travelingText.setOrigin(travelingText.getGlobalBounds().width / 2, travelingText.getGlobalBounds().height / 2);

	travelingDrawedText.setFont(Textfont);
	travelingDrawedText.setString("Traveling Drawed");
	travelingDrawedText.setCharacterSize(20);
	travelingDrawedText.setFillColor(sf::Color::White);
	travelingDrawedText.setPosition(SCREEN_WIDTH / 7 * 4, SCREEN_HEIGHT / 3 * 2 - 100);
	travelingDrawedText.setOrigin(travelingDrawedText.getGlobalBounds().width / 2, travelingDrawedText.getGlobalBounds().height / 2);

	textText.setFont(Textfont);
	textText.setString("Text");
	textText.setCharacterSize(20);
	textText.setFillColor(sf::Color::White);
	textText.setPosition(SCREEN_WIDTH / 7 * 5, SCREEN_HEIGHT / 3 * 2 - 100);
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
		if (event.type == sf::Event::Closed) {
			gameData->window.close();
		}
		else if (event.type == sf::Event::MouseButtonPressed) {
			if (event.mouseButton.button == sf::Mouse::Left) {
				if (showTravelsButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
					showTravels = !showTravels;
					if (showTravels) {
						showTravelsStateText.setString("ON");
						showTravelsStateText.setOrigin(showTravelsStateText.getGlobalBounds().width / 2, showTravelsStateText.getGlobalBounds().height / 2 + 5);
						showTravelsButton.setFillColor(sf::Color::Green);

					}
					else {
						showTravelsStateText.setString("OFF");
						showTravelsStateText.setOrigin(showTravelsStateText.getGlobalBounds().width / 2, showTravelsStateText.getGlobalBounds().height / 2 + 5);
						showTravelsButton.setFillColor(sf::Color::Red);
					}
				}
				else if (playButton.getGlobalBounds().contains(event.mouseButton.x, event.mouseButton.y)) {
					gameData->jsonManager.setShowTravels(showTravels);
					gameData->jsonManager.setBackground(background);
					gameData->jsonManager.setForeground(foreground);
					gameData->jsonManager.setTraveling(traveling);
					gameData->jsonManager.setTravelingDrawed(travelingDrawed);
					gameData->jsonManager.setText(text);
					gameData->jsonManager.write();
					gameData->machine.addGameState(GameStateReference(new MainState(gameData)));
				}
				else {
					update();
				}
			}
		}
		if (event.type == sf::Event::MouseButtonPressed || event.type == sf::Event::MouseMoved) {
			sf::Vector2f mousePos = sf::Vector2f(sf::Mouse::getPosition(gameData->window).x / 1.0f, sf::Mouse::getPosition(gameData->window).y / 1.0f);
			if (playButton.getGlobalBounds().contains(mousePos)) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (showTravelsButton.getGlobalBounds().contains(mousePos)) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (colorPickerBackground->isMouseOver(mousePos) == -1) {
				gameData->window.setMouseCursor(gameData->crossCursor);
			}
			else if (colorPickerBackground->isMouseOver(mousePos) == 1) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (colorPickerForeground->isMouseOver(mousePos) == -1) {
				gameData->window.setMouseCursor(gameData->crossCursor);
			}
			else if (colorPickerForeground->isMouseOver(mousePos) == 1) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (colorPickerTraveling->isMouseOver(mousePos) == -1) {
				gameData->window.setMouseCursor(gameData->crossCursor);
			}
			else if (colorPickerTraveling->isMouseOver(mousePos) == 1) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (colorPickerTravelingDrawed->isMouseOver(mousePos) == -1) {
				gameData->window.setMouseCursor(gameData->crossCursor);
			}
			else if (colorPickerTravelingDrawed->isMouseOver(mousePos) == 1) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (colorPickerText->isMouseOver(mousePos) == -1) {
				gameData->window.setMouseCursor(gameData->crossCursor);
			}
			else if (colorPickerText->isMouseOver(mousePos) == 1) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else {
				gameData->window.setMouseCursor(gameData->arrowCursor);
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
}

void ConfigState::draw() {
    gameData->window.clear();
	gameData->window.draw(playButton);

	gameData->window.draw(backgroundText);
	gameData->window.draw(foregroundText);
	gameData->window.draw(showTravelsText);
	gameData->window.draw(showTravelsButton);
	gameData->window.draw(showTravelsStateText);
	gameData->window.draw(travelingText);
	gameData->window.draw(travelingDrawedText);
	gameData->window.draw(textText);

	colorPickerBackground->draw();
	colorPickerForeground->draw();
	colorPickerTraveling->draw();
	colorPickerTravelingDrawed->draw();
	colorPickerText->draw();

    gameData->window.display();
}
