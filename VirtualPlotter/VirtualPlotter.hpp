#ifndef VIRTUALPLOTTER_HPP
#define VIRTUALPLOTTER_HPP

#include <SFML/Graphics.hpp>
#include <string>

#include "plotter.hpp"

class VirtualPlotter: public Plotter {
public:
	VirtualPlotter(const sf::Color& foreground, const sf::Color& background, const sf::Color& traveling, const sf::Color& traveling_drawed);

	void home() override;

	void export_picture(std::string path);

	std::string statistics();
	std::string statistics_values(const int& amount_commands_processed, const int& total_commands);


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
};

#endif //VIRTUALPLOTTER_HPP
