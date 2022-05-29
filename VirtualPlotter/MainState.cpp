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

sf::Color getColorFromString(std::string colorString) {
	std::vector<std::string> colors = splitString(colorString, 0, ",");
	if (colors.size() == 3) {
		int r = 0;
		int g = 0;
		int b = 0;
		if (colors[0] != "") {
			r = std::stoi(colors[0]);
		}
		if (colors[1] != "") {
			g = std::stoi(colors[1]);
		}
		if (colors[2] != "") {
			b = std::stoi(colors[2]);
		}
		return sf::Color(r, g, b);
	}
	else {
		throw InvalidJsonException("Invalid color string");
	}
}

void MainState::init() {
	std::ifstream inputFile;
	inputFile.open("gcode.txt");
	std::string temp;
	while (std::getline(inputFile, temp)) {
		gcode.push_back(splitString(temp, 1));
	}
	statistics_font.loadFromFile(BIT_FONT_PATH);
	statistics.setFont(statistics_font);
	statistics.setCharacterSize(20);
	statistics.setString(plotter.statistics());
	statistics_data.setFont(statistics_font);
	statistics_data.setCharacterSize(20);
	statistics_data.setString(plotter.statistics_values(i, gcode.size()));
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
		if (gcode.size() == 0 || i >= gcode.size()) {
			statistics_data.setString(plotter.statistics_values(i, gcode.size()));
			return;
		}
		if (gcode[i][0] == "G0" || gcode[i][0] == "G00") {
			plotter.draw({ std::stoi(gcode[i][1]), std::stoi(gcode[i][2]) }, 0);
		}
		else if (gcode[i][0] == "G1" || gcode[i][0] == "G01") {
			plotter.draw({ std::stoi(gcode[i][1]), std::stoi(gcode[i][2]) }, 1);
		}
		else if (gcode[i][0] == "G3" || gcode[i][0] == "G03") {
			plotter.g3();
		}
		else if (gcode[i][0] == "G4" || gcode[i][0] == "G04") {
			plotter.g4(std::stoi(gcode[i][1]), std::stoi(gcode[i][2]), std::stoi(gcode[i][3]));
		}
		else if (gcode[i][0] == "G5" || gcode[i][0] == "G05") {
			plotter.g5(std::stoi(gcode[i][1]), std::stoi(gcode[i][2]), std::stoi(gcode[i][3]));
		}
		
		else if (gcode[i][0] == "G28") {
			plotter.home();
		}
		i++;
	}
	statistics_data.setString(plotter.statistics_values(i, gcode.size()));
}

void MainState::draw() {
    gameData->window.clear();

	//Create background
	sf::Image imageBackground;

	imageBackground.create(gameData->window.getSize().x, gameData->window.getSize().y, background);
	sf::Texture textureBackground;
	textureBackground.loadFromImage(imageBackground);
	sf::Sprite spriteBackground;
	spriteBackground.setTexture(textureBackground);

	//Create drawable object of plotter
	sf::Texture texture;
	texture.loadFromImage(plotter.getImage());
	sf::Sprite sprite;
	sprite.setPosition(15, 15);
	sprite.setTexture(texture);

	//Create drawable border for plotter
	sf::Image imageBorder;
	
	if (background.r > 200 && background.g > 200 && background.b > 200) {
		imageBorder.create(texture.getSize().x + 30, texture.getSize().y + 30, sf::Color::Black);
	}
	else {
		imageBorder.create(texture.getSize().x + 30, texture.getSize().y + 30, sf::Color::White);
	}

	sf::Texture textureBorder;
	textureBorder.loadFromImage(imageBorder);
	sf::Sprite spriteBorder;
	spriteBorder.setTexture(textureBorder);
	

	//Create drawable object of statistics
	statistics.setPosition(1300, 10);
	statistics.setFillColor(text);
	statistics_data.setPosition(1550, 10);
	statistics_data.setFillColor(text);





	gameData->window.draw(spriteBackground);
	gameData->window.draw(spriteBorder);
	gameData->window.draw(sprite);
	gameData->window.draw(statistics);
	gameData->window.draw(statistics_data);

    gameData->window.display();
}