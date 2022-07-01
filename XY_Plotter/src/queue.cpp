#include "queue.hpp"

Queue::Queue():
  head(0),
  tail(0)
{}

void Queue::append(const Gcode& code) {
  queue[tail] = code;
  tail++;
}

void Queue::flush(){
  head=0;
  tail=0;
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
  return tail >= BUFFER_SIZE-1;
}
