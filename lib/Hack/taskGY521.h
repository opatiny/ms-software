#pragma once

#define IMU_ADDRESS 0x68

struct XyzData {
  double x;
  double y;
  double z;
};

struct ImuData {
  XyzData acceleration;  // in m/s^2
  XyzData rotation;      // in rad/s
  int temperature;
};

void taskGY521();