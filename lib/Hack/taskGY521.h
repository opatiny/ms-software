#pragma once

#define IMU_ADDRESS 0x68

struct XyzData {
  int x;
  int y;
  int z;
};

struct ImuData {
  XyzData acceleration;
  XyzData rotation;
  int temperature;
};

void taskGY521();