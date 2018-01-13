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
#include <string>


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
  rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
  rotation.normalize();
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
    //std::cout << "QUATERNION: " << rotation << std::endl;
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
  //quaternion madgwick = quaternion::Identity();

  // Set up a timer that expires every 20 ms.
  pacer loop_pacer;
  loop_pacer.set_period_ns(20000000);
  
  auto start = std::chrono::steady_clock::now();
  while(1)
  {
    auto last_start = start;
    start = std::chrono::steady_clock::now();
    std::chrono::nanoseconds duration = start - last_start;
    float dt = duration.count() / 1e9;
    if (dt < 0){ throw std::runtime_error("Time went backwards."); }

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();
    
    //const float angular_x = angular_velocity(0);
    //const float angular_y = angular_velocity(1);
    //const float angular_z = angular_velocity(2);
    //const float acc_x = acceleration(0);
	//const float acc_y = acceleration(1);
	//const float acc_z = acceleration(2);
	//const float mag_x = magnetic_field(0);
	//const float mag_y = magnetic_field(1);
	//const float mag_z = magnetic_field(2);
	
	//madgwick = MadgwickAHRSupdate(angular_x, angular_y, angular_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z);
		
    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);
    
    output(rotation);
    //std::cout << "  " << acceleration << "  " << magnetic_field << std::endl << std::endl;

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

    vector angular_velocity = imu.read_gyro();
    vector acceleration = imu.read_acc();
    vector magnetic_field = imu.read_mag();
           
    //ADD Madgwick algo here

    fuse(rotation, dt, angular_velocity, acceleration, magnetic_field);

	std::cout << "OUTPUT1 : " ;
    output(rotation);
    std::cout << "   ";
    std::cout << "OUTPUT2 : " ;
    output2(rotation);
    std::cout << "   " ;
    std::cout << "OUTPUT3 : " ;
    output3(rotation);
    std::cout << "   ";
    std::cout << acceleration << "   " <<magnetic_field << std::endl;
    
    loop_pacer.pace();
  }
}

void ahrs_conversion(imu & imu, fuse_function *fuse, rotation_output_function *output, rotation_output_function * output2, rotation_output_function * output3)
{
	imu.load_calibration();
  imu.enable();
   pacer loop_pacer;
  loop_pacer.set_period_ns(20000000); // Default : 20000000
  
  char path[] = "/home/robin/minimu9/minimu9-ahrs/results1bis.txt";
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
  set.mag = set.acc = set.gyro = true;

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
	ahrs_conversion(imu, &fuse_default, &output_matrix, &output_quaternion, &output_euler); 
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


//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================
//---------------------------------------------------------------------------------------------------
// Definitions

//~ #define sampleFreq	1660.0f		// sample frequency in Hz (was 512 Hz)
//~ #define betaDef		0.1f		// 2 * proportional gain

//~ //---------------------------------------------------------------------------------------------------
//~ // Variable definitions

//~ volatile float beta = betaDef;								// 2 * proportional gain (Kp)
//~ volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//~ //---------------------------------------------------------------------------------------------------
//~ // Function declarations

//~ float invSqrt(float x);

//~ //====================================================================================================
//~ // Functions

//~ //---------------------------------------------------------------------------------------------------
//~ // AHRS algorithm update

//~ quaternion MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	//~ float recipNorm;
	//~ float s0, s1, s2, s3;
	//~ float qDot1, qDot2, qDot3, qDot4;
	//~ float hx, hy;
	//~ float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	//~ quaternion quat = quaternion::Identity();

    //~ std::cout << "angular_x =" << gx << std::endl;


	//~ // Rate of change of quaternion from gyroscope
	//~ qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	//~ qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	//~ qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	//~ qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	//~ // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	//~ if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		//~ // Normalise accelerometer measurement
		//~ recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		//~ ax *= recipNorm;
		//~ ay *= recipNorm;
		//~ az *= recipNorm;   

		//~ // Normalise magnetometer measurement
		//~ recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		//~ mx *= recipNorm;
		//~ my *= recipNorm;
		//~ mz *= recipNorm;

		//~ // Auxiliary variables to avoid repeated arithmetic
		//~ _2q0mx = 2.0f * q0 * mx;
		//~ _2q0my = 2.0f * q0 * my;
		//~ _2q0mz = 2.0f * q0 * mz;
		//~ _2q1mx = 2.0f * q1 * mx;
		//~ _2q0 = 2.0f * q0;
		//~ _2q1 = 2.0f * q1;
		//~ _2q2 = 2.0f * q2;
		//~ _2q3 = 2.0f * q3;
		//~ _2q0q2 = 2.0f * q0 * q2;
		//~ _2q2q3 = 2.0f * q2 * q3;
		//~ q0q0 = q0 * q0;
		//~ q0q1 = q0 * q1;
		//~ q0q2 = q0 * q2;
		//~ q0q3 = q0 * q3;
		//~ q1q1 = q1 * q1;
		//~ q1q2 = q1 * q2;
		//~ q1q3 = q1 * q3;
		//~ q2q2 = q2 * q2;
		//~ q2q3 = q2 * q3;
		//~ q3q3 = q3 * q3;

		//~ // Reference direction of Earth's magnetic field
		//~ hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		//~ hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		//~ _2bx = sqrt(hx * hx + hy * hy);
		//~ _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		//~ _4bx = 2.0f * _2bx;
		//~ _4bz = 2.0f * _2bz;

		//~ // Gradient decent algorithm corrective step
		//~ s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//~ s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//~ s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//~ s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		//~ recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		//~ s0 *= recipNorm;
		//~ s1 *= recipNorm;
		//~ s2 *= recipNorm;
		//~ s3 *= recipNorm;

		//~ // Apply feedback step
		//~ qDot1 -= beta * s0;
		//~ qDot2 -= beta * s1;
		//~ qDot3 -= beta * s2;
		//~ qDot4 -= beta * s3;
	//~ }

	//~ // Integrate rate of change of quaternion to yield quaternion
	//~ q0 += qDot1 * (1.0f / sampleFreq);
	//~ q1 += qDot2 * (1.0f / sampleFreq);
	//~ q2 += qDot3 * (1.0f / sampleFreq);
	//~ q3 += qDot4 * (1.0f / sampleFreq);

	//~ // Normalise quaternion
	//~ recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	//~ q0 *= recipNorm;
	//~ q1 *= recipNorm;
	//~ q2 *= recipNorm;
	//~ q3 *= recipNorm;
	
	    //~ std::cout << "q0 =" << q0 << std::endl;

	
	//~ //quat = Eigen::Quaternionf(q0, q1, q2, q3);
	//~ return quat;
//~ }
//~ //---------------------------------------------------------------------------------------------------
//~ //END OF ALGORITHM
//~ //---------------------------------------------------------------------------------------------------
//~ // Fast inverse square-root
//~ // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//~ float invSqrt(float x) {
	//~ float halfx = 0.5f * x;
	//~ float y = x;
	//~ long i = *(long*)&y;
	//~ i = 0x5f3759df - (i>>1);
	//~ y = *(float*)&i;
	//~ y = y * (1.5f - (halfx * y * y));
	//~ return y;
//~ }
