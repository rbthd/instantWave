#pragma once

#include "vector.h"
#include <iostream>
#include <string>

#define MAX_CHAR_PER_LINE 80

using namespace std;

struct DATA //To store data from FILE*
{
	int32_t *m_x;
	int32_t *m_y;
	int32_t *m_z;
	int32_t *a_x;
	int32_t *a_y;
	int32_t *a_z;
	int32_t *g_x;
	int32_t *g_y;
	int32_t *g_z;
};

class imu
{
public:
  virtual void read_raw()
  {
    read_mag_raw();
    read_acc_raw();
    read_gyro_raw();
  }

  virtual void read_acc_raw() = 0;
  virtual void read_mag_raw() = 0;
  virtual void read_gyro_raw() = 0;

	virtual void read_pressure_raw() = 0;


  virtual void read_acc_raw_all() = 0;
  virtual void read_gyro_raw_all() = 0;
  virtual void read_mag_raw_all() = 0;

  virtual float get_acc_scale() const = 0;
  virtual float get_gyro_scale() const = 0;

  int32_t m[3];
  int32_t a[3];
  int32_t g[3];

	float p;

  // TODO: remove stuff below this point

  // Scaled readings
  virtual vector read_mag() = 0;  // In body coords, scaled to -1..1 range
  virtual vector read_acc() = 0;  // In body coords, with units = g
  virtual vector read_gyro() = 0; // In body coords, with units = rad/sec

  virtual vector read_acc_all() = 0;
  virtual vector read_gyro_all() = 0;
  virtual vector read_mag_all() = 0;

  virtual vector read_mag_conv(int32_t m_x, int32_t m_y, int32_t m_z) = 0;  // In body coords, scaled to -1..1 range
  virtual vector read_acc_conv(int32_t a_x, int32_t a_y, int32_t a_z) = 0;  // In body coords, with units = g
  virtual vector read_gyro_conv(int32_t g_x, int32_t g_y, int32_t g_z) = 0; // In body coords, with units = rad/sec

  virtual void measure_offsets_conv(int32_t* g_x, int32_t* g_y, int32_t* g_z) = 0;

  virtual void retrieve(char* PATH, DATA* data, int l_max) = 0;


  void read(){ read_mag(); read_acc(); read_gyro(); }

  virtual void measure_offsets() = 0;
  virtual void enable() = 0;
  virtual void load_calibration() = 0;

  vector gyro_offset;
  int_vector mag_min, mag_max;
};
