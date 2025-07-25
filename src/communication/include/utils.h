#ifndef COMMUNICATION_UTILS
#define COMMUNICATION_UTILS

#include <stdint.h>
#include <vector>

template <typename T>
union ExchangeData {
  T data;
  uint8_t buffer[sizeof(T)];
};

struct  __attribute__((packed, aligned(1))) TinyGimbal {
  float target_x;
  float target_y;
  float target_z;
  float yaw;
  float pitch;
  int16_t armor_id;
  bool is_shoot_vision; 
  int shoot_frequncy;
  bool is_get;
};

struct  __attribute__((packed, aligned(1))) NaviNuc {
  int16_t my_color;
};

#endif