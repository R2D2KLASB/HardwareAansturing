#ifndef QUEUE_HPP
#define QUEUE_HPP
#include "DataTypes.hpp"

class Queue {
  private:
    Gcode queue[5000] = {};
    int head = 0;
    int tail = 0;

  public:
    void init_queue() {
      head = 0;
      tail = 0;
    }
    void append(const Gcode& code) {
      queue[tail] = code;
      tail++;
    }

    bool pop(Gcode& code) {
      if (head < tail) {
        code = queue[head];
        head++;
        return false;
      }
      head = 0;
      tail = 0;
      return true;
    }

    bool isFull() {
      return tail >= 4999;
    }
};
#endif //QUEUE_HPP
