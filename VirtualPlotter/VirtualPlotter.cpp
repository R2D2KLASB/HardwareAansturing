#include "VirtualPlotter.hpp"
#include <iostream>

VirtualPlotter::VirtualPlotter(const sf::Color& foreground, const sf::Color& background, const sf::Color& traveling, const sf::Color& traveling_drawed, const bool show_travels) :
	Plotter({30000, 25000}),
	foreground(foreground),
	background(background),
	traveling(traveling),
	traveling_drawed(traveling_drawed),
	show_travels(show_travels)
{
	currentLocation = { 30000, 25000 };
	image.create(maxDimension.x/scale + padding*2, maxDimension.y/scale + padding * 2, background);
}

void VirtualPlotter::home() {
	setServo(false);
	while (currentLocation.y < maxDimension.y) {
		up(stepDelayUs);
		currentLocation.y++;
	}
	while (currentLocation.x < maxDimension.x) {
		right(stepDelayUs);
		currentLocation.x++;
	}
	currentLocation = { 30000, 25000 };
}

void VirtualPlotter::export_picture(std::string path) {
	image.saveToFile(path.c_str());
}

std::string VirtualPlotter::statistics() {
	return	"steps total:\nsteps drawing:\nTravel to drawing ratio:\nservo changes:\nGcodes done:\nTotal gcodes:\nProgress:\ntime physical plotter:";
}

std::string VirtualPlotter::statistics_values(const int& amount_commands_processed, const int& total_commands) {
	std::string drawing_percentage = std::to_string(((float)amountStepsDrawing / (float)amountStepsTotal) * 100);
	drawing_percentage = drawing_percentage.substr(0, drawing_percentage.find(".") + 3);
	std::string proces_percentage = std::to_string((amount_commands_processed / (total_commands * 1.0f) * 100));
	proces_percentage = proces_percentage.substr(0, proces_percentage.find(".") + 3);
	long long int time = stepDelayUs*2 * amountStepsTotal + servoDelayUs * servoChanges;
	std::string time_second = std::to_string(int(time / us_per_s) % 60);
	while (time_second.length() != 2){
		time_second = '0' + time_second;
	}
	return	std::to_string(amountStepsTotal) + '\n'
		+ std::to_string(amountStepsDrawing) + '\n'
		+ drawing_percentage + "%\n"
		+ std::to_string(servoChanges) + '\n'
		+ std::to_string(amount_commands_processed) + '\n'
		+ std::to_string(total_commands) + '\n'
		+ proces_percentage + '%' + '\n'
		+ std::to_string(int(int(time / us_per_s) / 60)) + ":" + time_second + '\n'
		+ "new:\t\t " + std::to_string(drawingTime + servoDelayUs * servoChanges) + '\n'
		+ "original:\t" + std::to_string(stepDelayUs * 2 * amountStepsTotal + servoDelayUs * servoChanges) + '\n';
		
}

void VirtualPlotter::setServo(bool draw) {
	servoChanges++;
	prevState = draw;
}

void VirtualPlotter::plot_pixel(const int& x, const int& y, const drawing_mode& mode) {
	unsigned int drawX = (x / scale) + padding; //scale of 1:25 with a padding of 10px
	unsigned int drawY = (maxDimension.y / scale) - (y / scale) + padding;
	if (drawX >= 0 && drawX < image.getSize().x && drawY >= 0 && drawY < image.getSize().y) {
		sf::Color temp = image.getPixel(drawX, drawY);
		if (mode == drawing_mode::FOREGROUND) {
			if (temp == background || temp == foreground) {
				image.setPixel(drawX, drawY, foreground);
			}
			else if ((temp == traveling || temp == traveling_drawed) && show_travels) {
				image.setPixel(drawX, drawY, traveling_drawed);
			}
		}
		else if (mode == drawing_mode::TRAVELING && show_travels) {
			if (temp == foreground || temp == traveling_drawed) {
				image.setPixel(drawX, drawY, traveling_drawed);
			}
			else if (temp == background || temp == traveling) {
				image.setPixel(drawX, drawY, traveling);
			}
		}
	}
}

void VirtualPlotter::up(unsigned int stepDelay) {
	if (prevState) {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::FOREGROUND);
		amountStepsDrawing++;
	}
	else {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::TRAVELING);
	}
	drawingTime += stepDelay * 2;
	//drawingTime += stepDelayUs * 2;
	amountStepsTotal++;
}

void VirtualPlotter::right(unsigned int stepDelay) {
	if (prevState) {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::FOREGROUND);
		amountStepsDrawing++;
	}
	else {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::TRAVELING);
	}
	drawingTime += stepDelay * 2;
	//drawingTime += stepDelayUs * 2;
	amountStepsTotal++;
}

void VirtualPlotter::down(unsigned int stepDelay) {
	if (prevState) {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::FOREGROUND);
		amountStepsDrawing++;
	}
	else {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::TRAVELING);
	}
	drawingTime += stepDelay * 2;
	//drawingTime += stepDelayUs * 2;
	amountStepsTotal++;
}

void VirtualPlotter::left(unsigned int stepDelay) {
	if (prevState) {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::FOREGROUND);
		amountStepsDrawing++;
	}
	else {
		plot_pixel(currentLocation.x, currentLocation.y, drawing_mode::TRAVELING);
	}
	drawingTime += stepDelay * 2;
	//drawingTime += stepDelayUs*2;
	amountStepsTotal++;
}
