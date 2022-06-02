#pragma once

#include <exception>
#include <string>

/**
* @file Exceptions.hpp
* @brief Project_Game: This file contains all error exceptions used throughout the program.
*/

/**
* @brief This class is used to catch load errors and give a detailed description of the problem.
*/
class LoadException : public std::exception {
private:
    std::string error;

public:

    /**
    * @brief This function defines the error message with the given parameters.
    * @param type This is the type of item you're trying to load out of a file.
    * @param name This is the name of the item you're trying to load out of a file.
    * @param filename This is the filename of the file you're trying to load the item out of
    */
    LoadException(const std::string &type, const std::string &name, const std::string &filename) :
            error{"Error with loading item of type: '" + type + "' with name: '" + name + "' and filename: '" +
                  filename + "'"} {}

	/**
    * @brief This function returns the error message.
    * @return An error message as a constant character pointer.
    */
    [[nodiscard]] const char *what() const noexcept override {
        return error.c_str();
    }
};

/**
* @brief
* This class is used to catch return errors and give a detailed description of the problem.
*/
class ReturnException : public std::exception {
private:
    std::string error;

public:

    /**
    * @brief This function defines the error message with the given parameters.
    * @param type This is the type of item you're trying to return.
    * @param name This is the name of the item you're trying to return.
    */
    ReturnException(const std::string &type, const std::string &name) :
            error{"Error with returning item of type: '" + type + "' with name: '" + name +
                  "' the item doesn't exist"} {}

    /**
    * @brief This function returns the error message.
    * @return An error message as a constant character pointer.
    */
    [[nodiscard]] const char *what() const noexcept override {
        return error.c_str();
    }
};

/**
* @brief This class is used to file opening errors and give a detailed description of the problem.
*/
class OpenFileException : public std::exception {
private:
    std::string error;

public:

    /**
    * @brief This function defines the error message with the given parameters.
    * @param filename This is name of the file you're tying to open.
	*/
    OpenFileException(const std::string &filename) :
            error("Could open file with filename: '" + filename + "'") {}

    /**
    * @brief This function returns the error message.
    * @return An error message as a constant character pointer.
    */
    [[nodiscard]] const char *what() const noexcept override {
        return error.c_str();
    }
};

/**
* @brief This class is used to catch invalid json errors and give a detailed description of the problem.
*/
class InvalidJsonException : public std::exception {
private:
    std::string error;
public:

    /**
    * @brief This function defines the error message with the given parameters.
    * @param error This is name of the file you're tying to open.
    */
    InvalidJsonException(const std::string &error) :
            error(error) {}

    /**
    * @brief This function returns the error message.
    * @return An error message as a constant character pointer.
    */
    [[nodiscard]] const char *what() const noexcept override {
        return error.c_str();
    }
};

/**
* @brief This class is used to catch unknown colors and give a detailed description of the problem.
*/
class UnknownColorException : public std::exception {
private:
    std::string error;
public:

    /**
    * @brief This function defines the error message with the given parameters.
    * @details This function describes the color name.
    * @param color This is the color name you're trying to access.
    */
    UnknownColorException(const std::string &color) :
            error("The color: " + color + " is not found as sf::Color") {}

    /**
    * @brief This function defines the error message with the given parameters.
    * @details This function describes the RGB values of the color you're trying to access.
    * @param color This is the color you're trying to access.
    */
    UnknownColorException(const sf::Color &color) :
            error("The color with rgb value: (" + std::to_string(color.r) + ", " + std::to_string(color.g) + ", " +
                  std::to_string(color.b) + ") is not found") {}

    /**
    * @brief This function returns the error message.
    * @return An error message as a constant character pointer.
    */
    [[nodiscard]] const char *what() const noexcept override {
        return error.c_str();
    }
};

