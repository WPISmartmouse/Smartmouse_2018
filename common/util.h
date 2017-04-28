#pragma once

void print(const char *fmt, ...);

extern struct global_program_settings_t {
  bool quiet;
} GlobalProgramSettings;

template<typename type_name, int size>
class CircularQueue {
private:
  type_name *cqueue_arr;
  int front_idx, rear_idx;
public:
  CircularQueue() : front_idx(-1), rear_idx(-1){
    cqueue_arr = new type_name[size];
  }

  bool full() {
    return ((front_idx == 0 && rear_idx == size - 1) || (front_idx == rear_idx + 1));
  }

  void clear() {
    front_idx = -1;
    rear_idx = -1;
  }

  int push_back(type_name item) {
    if ((front_idx == 0 && rear_idx == size - 1) || (front_idx == rear_idx + 1)) {
      return -1;
    }
    if (front_idx == -1) {
      front_idx = 0;
      rear_idx = 0;
    } else {
      if (rear_idx == size - 1)
        rear_idx = 0;
      else
        rear_idx = rear_idx + 1;
    }
    cqueue_arr[rear_idx] = item;
    return 0;
  }

  bool empty() {
    return (front_idx == -1);
  }

  type_name front() {
    // DOES NOT CHECK IF EMPTY. YOU MUST DO THAT
    return cqueue_arr[front_idx];
  }

  int pop_front() {
    if (front_idx == -1) {
      return -1;
    }
    if (front_idx == rear_idx) {
      front_idx = -1;
      rear_idx = -1;
    } else {
      if (front_idx == size - 1)
        front_idx = 0;
      else
        front_idx = front_idx + 1;
    }
    return 0;
  }
};