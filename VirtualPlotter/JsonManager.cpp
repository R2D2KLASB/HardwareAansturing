#include "JsonManager.hpp"
#include <iostream>
#include <utility>
#include "Exceptions.hpp"

std::vector<std::string> jsonSplitString(std::string str, int offset, std::string delimiter = " ") {
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

sf::Color jsonGetColorFromString(std::string colorString) {
	std::vector<std::string> colors = jsonSplitString(colorString, 0, ",");
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


JsonManager::JsonManager(const std::string & gameFile):
	gameFile(gameFile)
{
    this->getData();
}

void JsonManager::getData() {
	jsonData = getJsonFromFile();

    data = {
		jsonGetColorFromString(jsonData["background"].asString()),
		jsonGetColorFromString(jsonData["foreground"].asString()),
		jsonGetColorFromString(jsonData["traveling"].asString()),
		jsonGetColorFromString(jsonData["travelingDrawed"].asString()),
		jsonGetColorFromString(jsonData["text"].asString())
    };
}

sf::Color JsonManager::getForeground() {
	return data.foreground;
}

sf::Color JsonManager::getBackground() {
	return data.background;
}

sf::Color JsonManager::getTraveling() {
	return data.traveling;
}

sf::Color JsonManager::getTravelingDrawed() {
	return data.traveling_drawed;
}

sf::Color JsonManager::getText() {
	return data.text;
}

void JsonManager::setForeground(sf::Color newColor) {
	if (newColor == data.foreground) {
		return;
	}
	data.foreground = newColor;
	jsonData["foreground"] = colorToString(newColor);
	writeOut = true;
}

void JsonManager::setBackground(sf::Color newColor) {
	if (newColor == data.background) {
		return;
	}
	data.background = newColor;
	jsonData["background"] = colorToString(newColor);
	writeOut = true;
}

void JsonManager::setTraveling(sf::Color newColor) {
	if (newColor == data.traveling) {
		return;
	}
	data.traveling = newColor;
	jsonData["traveling"] = colorToString(newColor);
	writeOut = true;
}
void JsonManager::setTravelingDrawed(sf::Color newColor) {
	if (newColor == data.traveling_drawed) {
		return;
	}
	data.traveling_drawed = newColor;
	jsonData["travelingDrawed"] = colorToString(newColor);
	writeOut = true;
}
void JsonManager::setText(sf::Color newColor) {
	if (newColor == data.text) {
		return;
	}
	data.text = newColor;
	jsonData["text"] = colorToString(newColor);
	writeOut = true;
}

void JsonManager::write() {
	if (writeOut) {
		writeOut = false;
		writeJsonToFile();
	}
}


std::string JsonManager::colorToString(const sf::Color &colorSf) const{
	std::string temp = std::to_string(colorSf.r) + ','+std::to_string(colorSf.g) + ',' + std::to_string(colorSf.b);
	return temp;
}


Json::Value JsonManager::getJsonFromFile() {
	std::ifstream inputFile;
	Json::CharReaderBuilder builder;
	inputFile.open(gameFile);
	if (!inputFile.is_open()) {
		throw OpenFileException(gameFile);
	}
	builder["collectComments"] = true;
	JSONCPP_STRING errs;
	if (!parseFromStream(builder, inputFile, &jsonData, &errs)) {
		throw InvalidJsonException(errs);
	}
	inputFile.close();
	return jsonData;
}

void JsonManager::writeJsonToFile() {
	std::ofstream outputFile;
	outputFile.open(gameFile);
	if (!outputFile.is_open()) {
		throw OpenFileException(gameFile);
	}
	outputFile << jsonData;
	outputFile.close();
}
