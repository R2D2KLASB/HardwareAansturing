#ifndef QUEUE_HPP
#define QUEUE_HPP
#include "DataTypes.hpp"

class Queue {
  private:
    Gcode queue[5000] = {};
    int head = 0;
    int tail = 0;

  public:
    bool append(const Gcode& code) {
      if(tail<5000){
        queue[tail] = code;
        tail++;
        return false;
      }
      return true;
    }
    bool pop(Gcode& code) {
      if(head<=tail){
        code = queue[head];
        head++;
        return false;
      }
      head = 0;
      tail = 0;
      return true;
    }
};
#endif //QUEUE_HPP
