// For efficiency in compiling, this is the only file that uses the
// Eigen lirbary and the 'vector' type we made from it.

#include "vector.h"
#include "version.h"
#include "prog_options.h"
#include "minimu9.h"
#include "exceptions.h"
#include "pacer.h"
#include "MadgwickAHRS.h" //to run Madgwick algo
#include <iostream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>
#include <system_error>
#include <chrono>
#include <math.h>
#include <fstream>
#include <string.h>

void clean(char*buffer, FILE *fp)
{
	char *p = strchr(buffer,'\n');
	if(p!=NULL)
	{
		*p=0;
	}
	else
	{
		int c;
		while((c= fgetc(fp)) != '\n' && c!=EOF);
	}
}


// TODO: print warning if accelerometer magnitude is not close to 1 when starting up

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
float field_width = 6;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

std::ostream & operator << (std::ostream & os, const vector & vector)
{
  return os << FLOAT_FORMAT << vector(0) << ' '
            << FLOAT_FORMAT << vector(1) << ' '
            << FLOAT_FORMAT << vector(2);
}

std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
  return os << (vector)matrix.row(0) << ' '
            << (vector)matrix.row(1) << ' '
            << (vector)matrix.row(2);
}

std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
  return os << FLOAT_FORMAT << quat.w() << ' '
            << FLOAT_FORMAT << quat.x() << ' '
            << FLOAT_FORMAT << quat.y() << ' '
            << FLOAT_FORMAT << quat.z();
}

typedef void rotation_output_function(quaternion & rotation);

void output_quaternion(quaternion & rotation)
{
  std::cout << rotation;
}

void output_matrix(quaternion & rotation)
{
  std::cout << rotation.toRotationMatrix();
}

void output_euler(quaternion & rotation)
{
  vector euler =  (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0)
                        * (180 / M_PI));

  std::cout << euler;
  //~ euler[0]+= 21.23/1000 *(180/M_PI);   //This was added to compensate the local magnetic declination
  //~ std::cout << " XXXXXXXX " << euler[0];


}

void stream_raw_values(imu & imu)
{
  imu.enable();
  while(1)
  {
    imu.read_raw();
    printf("%7d %7d %7d  %7d %7d %7d  %7d %7d %7d\n",
           imu.m[0], imu.m[1], imu.m[2],
           imu.a[0], imu.a[1], imu.a[2],
           imu.g[0], imu.g[1], imu.g[2]
      );
    usleep(20*1000);
  }
}

matrix rotation_from_compass(const vector & acceleration, const vector & magnetic_field)
{
  vector down = -acceleration;
  vector east = down.cross(magnetic_field);
  vector north = east.cross(down);

  east.normalize();
  north.normalize();
  down.normalize();

  matrix r;
  r.row(0) = north;
  r.row(1) = east;
  r.row(2) = down;
  return r;
}

typedef void fuse_function(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field);

void fuse_compass_only(quaternion & rotation, float dt, const vector& angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  // Implicit conversion of rotation matrix to quaternion.
  rotation = rotation_from_compass(acceleration, magnetic_field);
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
void rotate(quaternion & rotation, const vector & w, float dt)
{
  // Multiply by first order approximation of the
  // quaternion representing this rotation.
  //std::cout << "QUATERNION 2 : " << rotation << endl;
  rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
  //std::cout << "QUATERNION 3 : " << rotation << endl;
  rotation.normalize();
  //std::cout << "QUATERNION 4 : " << rotation << endl;

  //std::cout << "QUATERNION FUSED =" << rotation << std::endl;
}

void fuse_gyro_only(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  rotate(rotation, angular_velocity, dt);
}

void fuse_default(quaternion & rotation, float dt, const vector & angular_velocity,
  const vector & acceleration, const vector & magnetic_field)
{
  vector correction = vector(0, 0, 0);

  if (fabs(acceleration.norm() - 1) <= 0.3)
  {
    // The magnetidude of acceleration is close to 1 g, so
    // it might be pointing up and we can do drift correction.

    const float correction_strength = 1;

    matrix rotation_compass = rotation_from_compass(acceleration, magnetic_field);
    matrix rotation_matrix = rotation.toRotationMatrix();
   // std::cout << "QUATERNION: " << rotation << std::endl;
    //std::cout << "ROTATION MAT :" ;
    //std::cout << rotation_matrix << std::endl ;

    correction = (
      rotation_compass.row(0).cross(rotation_matrix.row(0)) +
      rotation_compass.row(1).cross(rotation_matrix.row(1)) +
      rotation_compass.row(2).cross(rotation_matrix.row(2))
      ) * correction_strength;

  }

  rotate(rotation, angular_velocity + correction, dt);
}

void ahrs(imu & imu, fuse_function * fuse, rotation_output_function * output)
{
  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();

  // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(500000000);

  auto start = std::chrono::steady_clock::now();
  while(1)
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }
    
    vector acceleration = imu.read_acc();

    vector angular_velocity = imu.read_gyro();
    vector magnetic_field = imu.read_mag();
    
    float pressure_reg = imu.read_pressure_raw();
	float pressure = pressure_reg/4096.00;
	float altitude = (44330.8*(1-(pow((pressure)/1013.25,0.190263)))); //Compute actual altitude based on sea level

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

    output(rotation);
    std::cout << "  " << acceleration << "  " << magnetic_field << "       "  << pressure << "   "  << altitude << std::endl ;
    loop_pacer.pace();
  }
}

void ahrs_global(imu & imu, fuse_function * fuse, rotation_output_function * output, rotation_output_function * output2, rotation_output_function * output3)
{
  imu.load_calibration();
  imu.enable();
  imu.measure_offsets();


  // The quaternion that can convert a vector in body coordinates
  // to ground coordinates when it its changed to a matrix.
  quaternion rotation = quaternion::Identity();
 

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(20000000); // Default : 20000000


  auto start = std::chrono::steady_clock::now();
  while(1)
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }

    vector angular_velocity = imu.read_gyro_all();
    vector acceleration = imu.read_acc_all();
    vector magnetic_field = imu.read_mag_all();

    // MADGWICK ALGO VARIABLES
    //const float angular_x = angular_velocity(0);
    //const float angular_y = angular_velocity(1);
    //const float angular_z = angular_velocity(2);

    //const float acc_x = acceleration(0);
	//const float acc_y = acceleration(1);
	//const float acc_z = acceleration(2);

	//const float mag_x = magnetic_field(0);
	//const float mag_y = magnetic_field(1);
	//const float mag_z = magnetic_field(2);
	
	//float* madgwick_tmp = MadgwickAHRSupdate(angular_x, angular_y, angular_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z);
			
	//quaternion madgwick((const float)madgwick_tmp[0], (const float)madgwick_tmp[1], 
	//(const float)madgwick_tmp[2], (const float)madgwick_tmp[3]); //coeff are casted to match Eigen definition of quaternion's creator
	//*************************************//

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

	//std::cout << "OUTPUT1 : " ;
    output(rotation);
    //std::cout << madgwick_matrix  << endl;
    std::cout << "   " ;
    //std::cout << "Ori quat : " << " " ;
    output2(rotation);
    std::cout << "   " ;
    //std::cout << "OUTPUT3 : " ;
    output3(rotation);
    std::cout << "   ";
    std::cout << acceleration << "  " << magnetic_field << std::endl;
    //std::cout << "Mad quat : " << madgwick << endl;

    loop_pacer.pace();
  }
}

void ahrs_conversion(imu & imu, fuse_function *fuse, rotation_output_function *output, rotation_output_function * output2, rotation_output_function * output3, char* path)
{
   imu.load_calibration();
   imu.enable();
   pacer loop_pacer;
   loop_pacer.set_period_ns(20000000); // Default : 20000000

   int l_max=0;
   char buffer[MAX_CHAR_PER_LINE];
   DATA data;
   FILE * results=fopen(path,"rt");

	if(results)
	{
			while(!feof(results))
		{
			fgets(buffer,MAX_CHAR_PER_LINE,results);
			l_max++; //Counting the number of lines of the file
		}
	}
	fclose(results);

	data.m_x = (int32_t*)malloc(l_max*sizeof(float));
    data.m_y = (int32_t*)malloc(l_max*sizeof(float));
    data.m_z = (int32_t*)malloc(l_max*sizeof(float));
    data.a_x = (int32_t*)malloc(l_max*sizeof(float));
    data.a_y = (int32_t*)malloc(l_max*sizeof(float));
    data.a_z = (int32_t*)malloc(l_max*sizeof(float));
    data.g_x = (int32_t*)malloc(l_max*sizeof(float));
    data.g_y = (int32_t*)malloc(l_max*sizeof(float));
    data.g_z = (int32_t*)malloc(l_max*sizeof(float));

    imu.retrieve(path, &data, l_max);
	imu.measure_offsets_conv(data.g_x, data.g_y, data.g_z);
	quaternion rotation = quaternion::Identity();
	auto start = std::chrono::steady_clock::now();

        for(int i=0;i<l_max-1;i++)
		{
			auto last_start = start;
			start = std::chrono::steady_clock::now();
			std::chrono::nanoseconds duration = start - last_start;
		    float dt = duration.count() / 1e9;
	        if (dt < 0){ throw std::runtime_error("Time went backwards."); }
		 	//cout << data.m_x[i] << data.m_y[i] << data.m_z[i] << data.a_x[i] << data.a_y[i] << data.a_z[i] << data.g_x[i] << data.g_y[i] << data. g_z[i] << endl;

			vector angular_velocity = imu.read_gyro_conv(data.g_x[i], data.g_y[i], data.g_z[i]);
			vector acceleration = imu.read_acc_conv(data.a_x[i], data.a_y[i], data.a_z[i]);
			vector magnetic_field = imu.read_mag_conv(data.m_x[i], data.m_y[i], data.m_z[i]);

			//cout << "angular velocity = " << angular_velocity << "  "  << "acceleration= " << acceleration << "    " << "mag = " << magnetic_field << endl;

			fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

			//std::cout << "OUTPUT1 : " ;
			output(rotation);
			std::cout << "   ";
			//std::cout << "OUTPUT2 : " ;
			output2(rotation);
			std::cout << "   " ;
			//std::cout << "OUTPUT3 : " ;
			output3(rotation);
			std::cout << "   ";
			std::cout << acceleration << "   " <<magnetic_field << std::endl;
		    loop_pacer.pace();
		}
}

int main_with_exceptions(int argc, char **argv)
{
  prog_options options = get_prog_options(argc, argv);

  if(options.show_help)
  {
    print_command_line_options_desc();
    std::cout << "For more information, run: man minimu9-ahrs" << std::endl;
    return 0;
  }

  if (options.show_version)
  {
    std::cout << VERSION << std::endl;
    return 0;
  }

  // Decide what sensors we want to use.
  sensor_set set;
  set.mag = set.acc = set.gyro = set.pressure = true;

  minimu9::comm_config config = minimu9::auto_detect(options.i2c_bus_name);

  sensor_set missing = set - minimu9::config_sensor_set(config);
  if (missing)
  {
    if (missing.mag)
    {
      std::cerr << "Error: No magnetometer found." << std::endl;
    }
    if (missing.acc)
    {
      std::cerr << "Error: No accelerometer found." << std::endl;
    }
    if (missing.gyro)
    {
      std::cerr << "Error: No gyro found." << std::endl;
    }
    if (missing.pressure)
    {
      std::cerr << "Error: No barometer found." << std::endl;
    }
    std::cerr << "Error: Needed sensors are missing." << std::endl;
    return 1;
  }

  config = minimu9::disable_redundant_sensors(config, set);

  minimu9::handle imu;
  imu.open(config);

  rotation_output_function * output;

  // Figure out the output mode.
  if (options.output_mode == "matrix")
  {
    output = &output_matrix;
  }
  else if (options.output_mode == "quaternion")
  {
    output = &output_quaternion;
  }
  else if (options.output_mode == "euler")
  {
    field_width += 2;  // See comment above for field_width.
    output = &output_euler;
  }
  else
  {
    std::cerr << "Unknown output mode '" << options.output_mode << "'" << std::endl;
    return 1;
  }

  // Figure out the basic operating mode and start running.
  if (options.mode == "raw")
  {
    stream_raw_values(imu);
  }
  else if (options.mode == "all")
  {
	  ahrs_global(imu, &fuse_default, &output_matrix, &output_quaternion, &output_euler);
  }
  else if (options.mode == "conversion")
  {
	char path[200] = "";
	cout << " Please write down the ABSOLUTE path of the file to convert " << endl;
	cout << "ex : home/.../.../.../file.txt" << endl;
	cout << "PATH = " ;
	fgets(path, sizeof(path), stdin);
	clean(path,stdin);

	FILE* test_exist = fopen(path,"r+");

	while(test_exist == NULL)
	{
		cout << "The file doesn't exist or the path is wrong." << endl;
		cout << "Please try again !" << endl;
		cout << "PATH =  " ;
		fgets(path, sizeof(path), stdin);
		clean(path,stdin);
		test_exist = fopen(path,"r+");
	}
	ahrs_conversion(imu, &fuse_default, &output_matrix, &output_quaternion, &output_euler, path);

  }
  else if (options.mode == "gyro-only")
  {
    ahrs(imu, &fuse_gyro_only, output);
  }
  else if (options.mode == "compass-only")
  {
    ahrs(imu, &fuse_compass_only, output);
  }
  else if (options.mode == "normal")
  {
    ahrs(imu, &fuse_default, output);
  }
  else
  {
    std::cerr << "Unknown mode '" << options.mode << "'" << std::endl;
    return 1;
  }
  return 0;
}

int main(int argc, char ** argv)
{
  try
  {
    main_with_exceptions(argc, argv);
  }
  catch(const std::system_error & error)
  {
    std::string what = error.what();
    const std::error_code & code = error.code();
    std::cerr << "Error: " << what << " (" << code << ")" << std::endl;
    return 2;
  }
  catch(const std::exception & error)
  {
    std::cerr << "Error: " << error.what() << std::endl;
    return 9;
  }
}
