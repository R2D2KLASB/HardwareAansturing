#ifndef VIRTUALPLOTTER_HPP
#define VIRTUALPLOTTER_HPP

#include <SFML/Graphics.hpp>
#include <string>

#include "plotter.hpp"

class VirtualPlotter: public Plotter {
public:
	/**
	* @brief Constructor for the VirtualPlotter class
	* @param foreground const sf::Color& The color of the foreground
	* @param background const sf::Color& The color of the background
	* @param traveling const sf::Color& The color used when traveling
	* @param traveling_drawed cosnt sf::Color& The color used when traveling and drawn
	*/
	
	VirtualPlotter(const sf::Color& foreground, const sf::Color& background, const sf::Color& traveling, const sf::Color& traveling_drawed, const bool show_travels);

	/**
	* @brief implementation of the inherited function home
	*/
	void home() override;

	/**
	* @brief function that`s called on closing the program to export the picture
	* @param path const std::string& The path to the file
	*/
	void export_picture(std::string path);

	/**
	* @brief statistics function that`s called on initiaton that returns the statistics headers
	* @return std::string The statistics headers
	*/
	std::string statistics();
	/**
	* @brief statistics function that`s called on every update that return the statistics values
	* @param amount_commands_processed const int& The amount of commands processed
	* @param total_commands const int& The total amounts of commands
	* @return std::string The statistics values
	*/
	std::string statistics_values(const int& amount_commands_processed, const int& total_commands);

	/**
	* @brief function that`s called on every draw that returns the current image of the plotter drawing area
	* @return sf::Image The current image
	*/
	sf::Image getImage() {
		return image;
	}

private:
	enum drawing_mode {
		BACKGROUND,
		FOREGROUND,
		TRAVELING,
		TRAVELING_DRAWED
	};

	void setServo(bool draw) override;

	void up() override;

	void right() override;

	void down() override;

	void left() override;

	void plot_pixel(const int& x, const int& y, const drawing_mode& mode);

	unsigned int us_per_s = 1000000;
	unsigned int servoChanges = 0;
	unsigned int amountStepsDrawing = 0;
	unsigned int amountStepsTotal = 0;
	const unsigned int scale = 25;
	unsigned int padding = 10;

	sf::Color foreground;
	sf::Color background;
	sf::Color traveling;
	sf::Color traveling_drawed;
	sf::Image image;
	bool show_travels;

};

#endif //VIRTUALPLOTTER_HPP
