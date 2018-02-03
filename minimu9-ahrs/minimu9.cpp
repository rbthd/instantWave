#include "vector.h"
#include "minimu9.h"
#include "exceptions.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <wordexp.h>
#include <string>

#define SEP					" "
#define CMAX				9

typedef struct DATA DATA;
using namespace std;

minimu9::comm_config minimu9::auto_detect(const std::string & i2c_bus_name)
{
  i2c_bus bus(i2c_bus_name.c_str());
  minimu9::comm_config config;
{
  // Detect LSM6 devices.
    auto addrs = { lsm6::SA0_LOW_ADDR, lsm6::SA0_HIGH_ADDR };
    for (uint8_t addr : addrs)
    {
      int result = bus.try_write_byte_and_read_byte(addr, lsm6::WHO_AM_I);
      if (result == lsm6::LSM6DS33)
      {
        config.lsm6.use_sensor = true;
        config.lsm6.device = (lsm6::device_type)result;
        config.lsm6.i2c_bus_name = i2c_bus_name;
        config.lsm6.i2c_address = (lsm6::i2c_addr)addr;
        break;
      }
    }
}
{
  // Detect LIS3MDL sensor.
      auto addrs = { lis3mdl::SA1_LOW_ADDR, lis3mdl::SA1_LOW_ADDR };
    for (uint8_t addr : addrs)
    {
      int result = bus.try_write_byte_and_read_byte(addr, lis3mdl::WHO_AM_I);
      if (result == lis3mdl::LIS3MDL)
      {
        config.lis3mdl.use_sensor = true;
        config.lis3mdl.device = (lis3mdl::device_type)result;
        config.lis3mdl.i2c_bus_name = i2c_bus_name;
        config.lis3mdl.i2c_address = (lis3mdl::i2c_addr)addr;
        break;
      }
    }
 }
 {
   // Detect LPS25H sensor.
       auto addrs = { lps25h::SA0_LOW_ADDR};
     for (uint8_t addr : addrs)
     {
       int result = bus.try_write_byte_and_read_byte(addr, lps25h::WHO_AM_I);
       if (result == lis3mdl::LIS3MDL)
       {
         config.lps25h.use_sensor = true;
         config.lps25h.device = (lps25h::device_type)result;
         config.lps25h.i2c_bus_name = i2c_bus_name;
         config.lps25h.i2c_address = (lps25h::i2c_addr)addr;
         break;
       }
     }
  }
  return config;
}

sensor_set minimu9::config_sensor_set(const comm_config & config)
{
  sensor_set set;

  if (config.lsm6.use_sensor)
  {
    set.acc = true;
    set.gyro = true;
  }

  if (config.lis3mdl.use_sensor)
  {
    set.mag = true;
  }

  if (config.lps25h.use_sensor)
  {
    set.pressure = true;
  }
  return set;
}

minimu9::comm_config minimu9::disable_redundant_sensors(
  const comm_config & in, const sensor_set & needed)
{
  comm_config config = in;

  sensor_set missing = needed;

  if (!(missing.acc || missing.gyro))
  {
    config.lsm6.use_sensor = false;
  }
  else if (config.lsm6.use_sensor)
  {
    missing.acc = false;
    missing.gyro = false;
  }

  if (!missing.mag)
  {
    config.lis3mdl.use_sensor = false;
  }
  else if (config.lis3mdl.use_sensor)
  {
    missing.mag = false;
  }

  if (!missing.pressure)
  {
    config.lps25h.use_sensor = false;
  }
  else if (config.lps25h.use_sensor)
  {
    missing.pressure = false;
  }
  return config;
}

void minimu9::handle::open(const comm_config & config)
{
  this->config = config;

  if (config.lsm6.use_sensor)
  {
    lsm6.open(config.lsm6);
  }

  if (config.lis3mdl.use_sensor)
  {
    lis3mdl.open(config.lis3mdl);
  }

  if (config.lps25h.use_sensor)
  {
    lps25h.open(config.lps25h);
  }
}

void minimu9::handle::enable()
{
  if (config.lsm6.use_sensor)
  {
    lsm6.enable();
  }

  if (config.lis3mdl.use_sensor)
  {
    lis3mdl.enable();
  }

  if (config.lps25h.use_sensor)
  {
    lps25h.enable();
  }
}

void minimu9::handle::load_calibration()
{
  wordexp_t expansion_result;
  wordexp("/home/pi/instantWave/minimu9-ahrs/minimu9-ahrs-cal", &expansion_result, 0);

  std::ifstream file(expansion_result.we_wordv[0]);
  if (file.fail())
  {
    throw posix_error("Failed to open calibration file /home/pi/instantWave/minimu9-ahrs/minimu9-ahrs-cal");
  }

  file >> mag_min(0) >> mag_max(0)
       >> mag_min(1) >> mag_max(1)
       >> mag_min(2) >> mag_max(2);
  if (file.fail() || file.bad())
  {
    throw std::runtime_error("Failed to parse calibration file /home/pi/instantWave/minimu9-ahrs/minimu9-ahrs-cal");
  }
}

void minimu9::handle::read_mag_raw()
{
  if (config.lis3mdl.use_sensor)
  {
    lis3mdl.read();
    for (int i = 0; i < 3; i++) { m[i] = lis3mdl.m[i]; }
  }
   else
  {
    throw std::runtime_error("No magnetometer to read.");
  }
}

void minimu9::handle::read_mag_raw_all()
{
  if (config.lis3mdl.use_sensor)
  {
    lis3mdl.read();
    for (int i = 0; i < 3; i++) { m[i] = lis3mdl.m[i]; cout << m[i] << " " ; }
  }
  else
  {
    throw std::runtime_error("No magnetometer to read.");
  }
}

void minimu9::handle::read_acc_raw()
{
  if (config.lsm6.use_sensor)
  {
    lsm6.read_acc();
    for (int i = 0; i < 3; i++) { a[i] = lsm6.a[i]; }
  }

  else
  {
    throw std::runtime_error("No accelerometer to read.");
  }
}

void minimu9::handle::read_acc_raw_all()
{
  if (config.lsm6.use_sensor)
  {
    lsm6.read_acc();
    for (int i = 0; i < 3; i++) { a[i] = lsm6.a[i]; cout << a[i] << " " ; }
  }
  else
  {
    throw std::runtime_error("No accelerometer to read.");
  }
}

void minimu9::handle::read_gyro_raw()
{
  if (config.lsm6.use_sensor)
  {
    lsm6.read_gyro();
    for (int i = 0; i < 3; i++) { g[i] = lsm6.g[i]; }
  }
  else
  {
    throw std::runtime_error("No gyro to read.");
  }
}

void minimu9::handle::read_gyro_raw_all()
{
  if (config.lsm6.use_sensor)
  {
    lsm6.read_gyro();
    for (int i = 0; i < 3; i++) { g[i] = lsm6.g[i]; cout << g[i] << " " ; }
  }
  else
  {
    throw std::runtime_error("No gyro to read.");
  }
}

float minimu9::handle::read_pressure_raw()
{
  if (config.lps25h.use_sensor)
  {
    lps25h.read();
    p = lps25h.p; 
  }
  else
  {
    throw std::runtime_error("No gyro to read.");
  }
}

float minimu9::handle::get_acc_scale() const
{
  // Info about linear acceleration sensitivity from datasheets:
  // LSM303DLM: at FS = 8 g, 3.9 mg/digit (12-bit reading)
  // LSM303DLHC: at FS = 8 g, 4 mg/digit (12-bit reading probably an approximation)
  // LSM303DLH: at FS = 8 g, 3.9 mg/digit (12-bit reading)
  // LSM303D: at FS = 8 g, 0.244 mg/LSB (16-bit reading)
  // LSM6DS33: at FS = 8 g, 0.244 mg/LSB (16-bit reading)
  //			at FS = 2 g, 0.061 mg/LSB
  return 0.000061;
}

float minimu9::handle::get_gyro_scale() const
{
  // Info about sensitivity from datasheets:
  // L3G4200D, FS = 2000 dps: 70 mdps/digit
  // L3GD20,   FS = 2000 dps: 70 mdps/digit
  // L3GD20H,  FS = 2000 dps: 70 mdps/digit
  // LSM6DS33, FS = 2000 dps: 70 mdps/digit
  return 0.004375 * 3.14159265 / 180;
}

void minimu9::handle::measure_offsets()
{
  // LSM303 accelerometer: 8 g sensitivity.  3.9 mg/digit; 1 g = 256.
  gyro_offset = vector::Zero();
  const int sampleCount = 511;
  for(int i = 0; i < sampleCount; i++)
  {
    read_gyro_raw();
    gyro_offset += vector_from_ints(&g);
    usleep(20 * 1000);
  }
  gyro_offset /= sampleCount;
}

void minimu9::handle::measure_offsets_conv(int32_t* g_x, int32_t* g_y, int32_t* g_z)
{
  // LSM303 accelerometer: 8 g sensitivity.  3.8 mg/digit; 1 g = 256.
  gyro_offset = vector::Zero();
  const int sampleCount = 32;
  for(int i = 0; i < sampleCount; i++)
  {
    //read_gyro_raw();
    g[0] = g_x[i];
    g[1] = g_y[i];
    g[2] = g_z[i];
    gyro_offset += vector_from_ints(&g);
    usleep(20 * 1000);
  }
  gyro_offset /= sampleCount;
}

vector minimu9::handle::read_mag()
{
  read_mag_raw();

  vector v;
  v(0) = (float)(m[0] - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
  v(1) = (float)(m[1] - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
  v(2) = (float)(m[2] - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
  return v;
}

vector minimu9::handle::read_mag_all()
{
  read_mag_raw_all();

  vector v;
  v(0) = (float)(m[0] - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
  v(1) = (float)(m[1] - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
  v(2) = (float)(m[2] - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
  return v;
}

vector minimu9::handle::read_acc()
{
  read_acc_raw();
  return vector_from_ints(&a) * get_acc_scale();
}

vector minimu9::handle::read_acc_all()
{
  read_acc_raw_all();
  return vector_from_ints(&a) * get_acc_scale();
}

vector minimu9::handle::read_gyro()
{
  read_gyro_raw();
  return (vector_from_ints(&g) - gyro_offset) * get_gyro_scale();
}

vector minimu9::handle::read_gyro_all()
{
  read_gyro_raw_all();
  return (vector_from_ints(&g) - gyro_offset) * get_gyro_scale();
}

vector minimu9::handle::read_mag_conv(int32_t m_x, int32_t m_y, int32_t m_z)
{
  vector v;
  v(0) = (float)(m_x - mag_min(0)) / (mag_max(0) - mag_min(0)) * 2 - 1;
  v(1) = (float)(m_y - mag_min(1)) / (mag_max(1) - mag_min(1)) * 2 - 1;
  v(2) = (float)(m_z - mag_min(2)) / (mag_max(2) - mag_min(2)) * 2 - 1;
  return v;
}


vector minimu9::handle::read_acc_conv(int32_t a_x, int32_t a_y, int32_t a_z)
{
			a[0] = a_x;
			a[1] = a_y;
			a[2] = a_z;
			return vector_from_ints(&a) * get_acc_scale();

}


vector minimu9::handle::read_gyro_conv(int32_t g_x, int32_t g_y, int32_t g_z)
{
			g[0]=g_x;
			g[1]=g_y;
			g[2]=g_z;

  return (vector_from_ints(&g) - gyro_offset) * get_gyro_scale();
}


void minimu9::handle::retrieve(char* PATH, DATA* data, int l_max)
{
FILE* results;
char* token;
char buffer[MAX_CHAR_PER_LINE];
int i = 0;
int j = 0;
int32_t values[l_max][CMAX];
int32_t m_x[l_max], m_y[l_max], m_z[l_max], a_x[l_max], a_y[l_max], a_z[l_max], g_x[l_max], g_y[l_max], g_z[l_max];

results=fopen(PATH,"rt");

	while(!feof(results))
	{
		j=0;
		fgets(buffer,MAX_CHAR_PER_LINE,results);
		token = strtok(buffer,SEP);

		while(token !=NULL && j<CMAX)
		{
			values[i][j] = atoi(token);
			token = strtok(NULL,SEP);
			j++;
		}
		i++;
	}

fclose(results); // Closing the file
i = j = 0;

	for(int x=0; x<l_max-1; x++)
	{
			data->m_x[i] = values[x][0];
			data->m_y[i] = values[x][1];
			data->m_z[i] = values[x][2];
			data->a_x[i] = values[x][3];
			data->a_y[i] = values[x][4];
			data->a_z[i] = values[x][5];
			data->g_x[i] = values[x][6];
			data->g_y[i] = values[x][7];
			data->g_z[i] = values[x][8];

			i++;
		}
	}
