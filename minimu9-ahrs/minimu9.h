#pragma once

// TODO: I'd rather have the minimu9 and imu class not know anything about
// floating point vectors and calibration.  Maybe just provide floating
// point scaling factors so that higher-level code can make sense of the
// raw vectors.

#include "imu.h"
#include "lsm6.h"
#include "lis3mdl.h"
#include "lps25h.h"
#include "sensor_set.h"
#include <string>

namespace minimu9
{
  // Represents the sensors of the MinIMU-9 and how to communicate with them.
  struct comm_config {
    lis3mdl::comm_config lis3mdl;
    lsm6::comm_config lsm6;
    lps25h::comm_config lps25h;
  };

  comm_config auto_detect(const std::string & i2c_bus_name);

  sensor_set config_sensor_set(const comm_config &);

  comm_config disable_redundant_sensors(const comm_config &, const sensor_set &);

  class handle : public imu {
  public:
    void open(const comm_config &);

    comm_config config;
    lsm6::handle lsm6;
    lis3mdl::handle lis3mdl;
    lps25h::handle lps25h;


    virtual void read_acc_raw();
    virtual void read_mag_raw();
    virtual void read_gyro_raw();

    virtual void read_acc_raw_all();
    virtual void read_gyro_raw_all();
    virtual void read_mag_raw_all();

    virtual float read_pressure_raw();


    virtual float get_acc_scale() const;
    virtual float get_gyro_scale() const;

    virtual vector read_acc();
    virtual vector read_mag();
    virtual vector read_gyro();

    virtual vector read_acc_all();
    virtual vector read_gyro_all();
    virtual vector read_mag_all();

    virtual vector read_acc_conv(int32_t a_x, int32_t a_y, int32_t a_z);
    virtual vector read_mag_conv(int32_t m_x, int32_t m_y, int32_t m_z);
    virtual vector read_gyro_conv(int32_t g_x, int32_t g_y, int32_t g_z);

    virtual void measure_offsets_conv(int32_t* g_x, int32_t* g_y, int32_t* g_z);

    virtual void retrieve(char* FILE, DATA* data, int l_max);

    virtual void enable();
    virtual void load_calibration();
    virtual void measure_offsets();
  };
}
