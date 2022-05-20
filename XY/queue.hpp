#ifndef QUEUE_HPP
#define QUEUE_HPP
#include "DataTypes.hpp"
#include "Definitions.hpp"

/**
   @brief This class is used to implement a buffer for fast gcode reading.
*/
class Queue {
  public:

    /**
       @brief this constructor constructs an object of the Queue class.
       @details this constructor initializes the head and tail private variables to 0.
    */
    Queue();

    /**
       @brief this function adds a line of gcode to the buffer.
       @param code this is a line of gcode that is added to the buffer.
       @details this funtion adds a line of gcode to the buffer and increments the tail counter
    */
    void append(const Gcode& code);

    /**
       @brief This function pops the last line of the buffer into the given parameter. With this action, it also removes the line from the buffer.
       @param code this parameter's value is changed to the popped line of gcode.
       @details because the parameter is given by reference, it has no return value, instead it changes the parameter's value.
    */
    bool pop(Gcode& code);

    /**
       @brief this funtion checks if the buffer is full
       @return this function returns if the buffer is full or not
    */
    bool isFull();

  private:
    Gcode queue[BUFFER_SIZE] = {};
    int head;
    int tail;
};

#endif //QUEUE_HPP
