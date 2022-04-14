#ifndef QUEUE_HPP
#define QUEUE_HPP
#include "DataTypes.hpp"

class Queue {
  private:
    Gcode queue[5000] = {};
    int head;
    int tail;

  public:    
    int getTail(){
      return tail;
    }
    int getHead(){
      return head;
    }
    bool append(const Gcode& code) {
      if(tail<5000){
        queue[tail] = code;
        tail++;
        return false;
      }
      return true;
    }
    bool pop(Gcode& code) {
      if(head<tail){
        code = queue[head];
        head++;
        return false;
      }
      head = 0;
      tail = 0;
      return true;
    }
    bool isFull(){
      return tail >= 5000;
    }
};
#endif //QUEUE_HPP
