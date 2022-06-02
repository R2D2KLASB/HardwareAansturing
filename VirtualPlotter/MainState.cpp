#include "MainState.hpp"
#include <utility>


MainState::MainState(GameDataReference gameData) : gameData(std::move(gameData)) {}

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
	statistics_font.loadFromFile(BIT_FONT_PATH);
	statistics.setFont(statistics_font);
	statistics.setCharacterSize(20);
	statistics.setString(plotter.statistics());
	statistics_data.setFont(statistics_font);
	statistics_data.setCharacterSize(20);
	statistics_data.setString(plotter.statistics_values(i, gcodeStrings.size()));
}

void MainState::handleInput() {
	sf::Event event{};
	while (gameData->window.pollEvent(event)) {
		if (sf::Event::Closed == event.type) {
			gameData->window.close();
			plotter.export_picture("image.bmp");
		}
		if (sf::Event::KeyPressed == event.type) {
			if(event.key.code == sf::Keyboard::Left and speed >= 1) {
				speed--;
			}
			else if(event.key.code == sf::Keyboard::Right) {
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

    gameData->window.display();
}