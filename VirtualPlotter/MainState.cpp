#include "MainState.hpp"
#include <utility>
#include "ConfigState.hpp"

MainState::MainState(GameDataReference gameData) :
	gameData(gameData),
	showTravels(gameData->jsonManager.getShowTravels()),
	background(gameData->jsonManager.getBackground()),
	foreground(gameData->jsonManager.getForeground()),
	traveling(gameData->jsonManager.getTraveling()),
	traveling_drawed(gameData->jsonManager.getTravelingDrawed()),
	text(gameData->jsonManager.getText()),
	plotter(VirtualPlotter(foreground, background, traveling, traveling_drawed, showTravels))
{}

std::vector<std::string> splitString(std::string str, int offset, std::string delimiter = " ") {
	std::vector<std::string> words;
	int start = 0;
	int end = str.find(delimiter);
	while (end != -1) {
		words.push_back(str.substr(start, end - start));
		start = end + delimiter.size() + offset;
		end = str.find(delimiter, start);
	}
	words.push_back(str.substr(start, end - start));
	return words;
}

void MainState::init() {
	std::ifstream inputFile;
	inputFile.open("gcode.txt");
	std::string temp;
	while (std::getline(inputFile, temp)) {
		gcodeStrings.push_back(temp); //splitString(temp, 1));
	}
	textFont.loadFromFile(BIT_FONT_PATH);
	statistics.setFont(textFont);
	statistics.setCharacterSize(20);
	statistics.setString(plotter.statistics());
	statistics_data.setFont(textFont);
	statistics_data.setCharacterSize(20);
	statistics_data.setString(plotter.statistics_values(i, gcodeStrings.size()));

	exportImageButton.setSize({ 450, 100 });
	exportImageButton.setFillColor(sf::Color::Black);
	exportImageButton.setPosition(sf::Vector2f(gameData->window.getSize().x - 350, gameData->window.getSize().y - 450));
	exportImageButton.setOutlineColor(sf::Color::White);
	exportImageButton.setOutlineThickness(5);
	exportImageButton.setOrigin(exportImageButton.getSize().x / 2, exportImageButton.getSize().y / 2);

	exportImageText.setFont(textFont);
	exportImageText.setCharacterSize(30);
	exportImageText.setString("Export image");
	exportImageText.setPosition(sf::Vector2f(gameData->window.getSize().x - 350, gameData->window.getSize().y - 450));
	exportImageText.setOrigin(exportImageText.getGlobalBounds().width / 2, exportImageText.getGlobalBounds().height / 2 + 10);

	exportImageStatsButton.setSize({ 450, 100 });
	exportImageStatsButton.setFillColor(sf::Color::Black);
	exportImageStatsButton.setPosition(sf::Vector2f(gameData->window.getSize().x - 350, gameData->window.getSize().y - 300));
	exportImageStatsButton.setOutlineColor(sf::Color::White);
	exportImageStatsButton.setOutlineThickness(5);
	exportImageStatsButton.setOrigin(exportImageStatsButton.getSize().x / 2, exportImageStatsButton.getSize().y / 2);
	
	exportImageStatsText.setFont(textFont);
	exportImageStatsText.setCharacterSize(30);
	exportImageStatsText.setString("Export image with statistics");
	exportImageStatsText.setPosition(sf::Vector2f(gameData->window.getSize().x - 350, gameData->window.getSize().y - 300));
	exportImageStatsText.setOrigin(exportImageStatsText.getGlobalBounds().width / 2, exportImageStatsText.getGlobalBounds().height / 2 + 10);
	
	configButton.setSize({ 450, 100});
	configButton.setFillColor(sf::Color::Black);
	configButton.setPosition(sf::Vector2f(gameData->window.getSize().x-350, gameData->window.getSize().y - 150));
	configButton.setOutlineColor(sf::Color::White);
	configButton.setOutlineThickness(5);
	configButton.setOrigin(configButton.getSize().x / 2, configButton.getSize().y / 2);
	
	configText.setFont(textFont);
	configText.setCharacterSize(30);
	configText.setString("Return to config menu");
	configText.setPosition(sf::Vector2f(gameData->window.getSize().x - 350, gameData->window.getSize().y - 150));
	configText.setOrigin(configText.getGlobalBounds().width / 2, configText.getGlobalBounds().height / 2+10);
	
	gameData->window.setMouseCursor(gameData->arrowCursor);
}

void MainState::handleInput() {
	sf::Event event{};
	while (gameData->window.pollEvent(event)) {
		if (event.type == sf::Event::Closed) {
			gameData->window.close();
		}
		if (event.type == sf::Event::KeyPressed) {
			if (event.key.code == sf::Keyboard::Left and speed >= 1) {
				speed--;
			}
			else if (event.key.code == sf::Keyboard::Right) {
				speed++;
			}
			else if (event.key.code == sf::Keyboard::Up) {
				speed = 4294967290;
			}
			else if (event.key.code == sf::Keyboard::Down) {
				speed = 0;
			}
			else if (event.key.code == sf::Keyboard::Space) {
				speed = 8;
			}
			else if (event.key.code == sf::Keyboard::R) {
				gameData->machine.addGameState(GameStateReference(new MainState(gameData)));
			}
		}
		else if (event.type == sf::Event::MouseButtonPressed) {
			if (event.mouseButton.button == sf::Mouse::Left) {
				if (configButton.getGlobalBounds().contains(gameData->window.mapPixelToCoords(sf::Mouse::getPosition(gameData->window)))) {
					gameData->machine.addGameState(GameStateReference(new ConfigState(gameData)));
				}
				if (exportImageButton.getGlobalBounds().contains(gameData->window.mapPixelToCoords(sf::Mouse::getPosition(gameData->window)))) {
					plotter.export_picture("image.bmp");
				}
				if (exportImageStatsButton.getGlobalBounds().contains(gameData->window.mapPixelToCoords(sf::Mouse::getPosition(gameData->window)))) {
					draw();
					
					sf::Vector2u windowSize = gameData->window.getSize();
					sf::RectangleShape hideButtons({ 600, 800 });
					hideButtons.setFillColor(background);
					hideButtons.setPosition(sf::Vector2f(windowSize.x - 300, windowSize.y - 400));
					hideButtons.setOrigin(hideButtons.getSize().x / 2, hideButtons.getSize().y / 2);
					gameData->window.draw(hideButtons);
					
					sf::Texture texture;
					texture.create(windowSize.x, windowSize.y);
					texture.update(gameData->window);
					
					texture.copyToImage().saveToFile("image.bmp");
				}
			}
		}
		else if (event.type == sf::Event::MouseMoved) {
			if (configButton.getGlobalBounds().contains(gameData->window.mapPixelToCoords(sf::Mouse::getPosition(gameData->window)))) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (exportImageButton.getGlobalBounds().contains(gameData->window.mapPixelToCoords(sf::Mouse::getPosition(gameData->window)))) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else if (exportImageStatsButton.getGlobalBounds().contains(gameData->window.mapPixelToCoords(sf::Mouse::getPosition(gameData->window)))) {
				gameData->window.setMouseCursor(gameData->handCursor);
			}
			else {
				gameData->window.setMouseCursor(gameData->arrowCursor);
			}
		}
	}
}

void MainState::update() {
	for (unsigned int x = 0; x < speed; x++) {
		if (gcodeStrings.size() == 0 || i >= gcodeStrings.size()) {
			statistics_data.setString(plotter.statistics_values(i, gcodeStrings.size()));
			return;
		}
		std::vector<std::string>gcode = splitString(gcodeStrings[i], 1);
 		if ((gcode[0] == "G0" || gcode[0] == "G00") && gcode.size() == 3) {
			plotter.draw({ std::stoi(gcode[1]), std::stoi(gcode[2]) }, 0);
		}
		else if ((gcode[0] == "G1" || gcode[0] == "G01") && gcode.size() == 3) {
			plotter.draw({ std::stoi(gcode[1]), std::stoi(gcode[2]) }, 1);
		}
		else if ((gcode[0] == "G3" || gcode[0] == "G03") && gcode.size() == 1) {
			plotter.g3();
		}
		else if ((gcode[0] == "G4" || gcode[0] == "G04") && gcode.size() == 4) {
			plotter.g4(std::stoi(gcode[1]), std::stoi(gcode[2]), std::stoi(gcode[3]));
		}
		else if ((gcode[0] == "G5" || gcode[0] == "G05") && gcode.size() == 4) {
			plotter.g5(std::stoi(gcode[1]), std::stoi(gcode[2]), std::stoi(gcode[3]));
		}
		else if ((gcode[0] == "G6" || gcode[0] == "G06") && gcode.size() == 5) {
			plotter.g6(std::stoi(gcode[1]), std::stoi(gcode[2]), std::stoi(gcode[3]), std::stoi(gcode[4]));
		}
		else if ((gcode[0]) == "G7" && gcode.size() == 6) {
			plotter.g7(std::stoi(gcode[1]), std::stoi(gcode[2]), std::stoi(gcode[3]), std::stoi(gcode[4]), std::stoi(gcode[5]));
		}
		else if (gcode[0] == "G28" && gcode.size() == 1) {
			plotter.home();
		}
		else {
			std::string error = "The command: '" + gcodeStrings[i] + "' is not found as Gcode\n";
			std::cout << error;
//			throw std::exception(error.c_str());
		}
		i++;
	}
	statistics_data.setString(plotter.statistics_values(i, gcodeStrings.size()));
}

void MainState::draw() {
    gameData->window.clear();

	//Create background
	sf::RectangleShape backgroundShape(sf::Vector2f(gameData->window.getSize().x, gameData->window.getSize().y));
	backgroundShape.setFillColor(background);
	
	//Create drawable object of plotter
	sf::Texture texture;
	texture.loadFromImage(plotter.getImage());
	sf::Sprite sprite;
	sprite.setPosition(10, 10);
	sprite.setTexture(texture);

	//Create drawable border for plotter
	sf::RectangleShape borderShape(sf::Vector2f(texture.getSize().x, texture.getSize().y));
	borderShape.setPosition(sprite.getGlobalBounds().left, sprite.getGlobalBounds().top);
	if (background.r > 200 && background.g > 200 && background.b > 200) {
		borderShape.setOutlineColor(sf::Color::Black);
	}
	else {
		borderShape.setOutlineColor(sf::Color::White);
	}
	borderShape.setOutlineThickness(5);

	//Create drawable object of statistics
	statistics.setPosition(1300, 10);
	statistics.setFillColor(text);
	statistics_data.setPosition(1550, 10);
	statistics_data.setFillColor(text);


	gameData->window.draw(backgroundShape);
	gameData->window.draw(borderShape);
	gameData->window.draw(sprite);
	gameData->window.draw(statistics);
	gameData->window.draw(statistics_data);

	gameData->window.draw(exportImageButton);
	gameData->window.draw(exportImageText);

	gameData->window.draw(exportImageStatsButton);
	gameData->window.draw(exportImageStatsText);

	gameData->window.draw(configButton);
	gameData->window.draw(configText);

    gameData->window.display();
}