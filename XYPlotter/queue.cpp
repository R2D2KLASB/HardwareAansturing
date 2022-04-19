#include "queue.hpp"
void Queue::append(const Gcode& code) {
      queue[tail] = code;
      tail++;
    }

    void Queue::init_queue() {
      head = 0;
      tail = 0;
    }
    
    bool Queue::pop(Gcode& code) {
      if (head < tail) {
        code = queue[head];
        head++;
        return false;
      }
      head = 0;
      tail = 0;
      return true;
    }

bool Queue::isFull() {
      return tail >= 4999;
}