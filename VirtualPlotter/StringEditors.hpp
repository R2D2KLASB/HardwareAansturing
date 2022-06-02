#ifndef STRING_EDITORS_HPP
#define STRING_EDITORS_HPP
#include <vector>
#include <string>
#include <SFML/Graphics.hpp>

/**
* @brief function that slits std::string on the given delimiter plus the offset and that gives back a vector of strings
* @param str the string to slit
* @param delimiter the delimiter
* @param offset the offset
* @return a vector of strings
*/
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

/**
* @brief function that turns a string containing rgb seperated by commas into a sf::Color
* @param colorString the string to convert
* @return the sf::Color
*/
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
}

#endif //STRING_EDITORS_HPP