// https://github.com/strangedev/Arduino-Quadcopter
// https://developer.mbed.org/users/onehorse/code/MPU9250AHRS/

/***************************************************
 *     d           c
 *       *       *
 *         * ^ *
 *           |
 *         * | *
 *       *       *
 *     a           b
 ***************************************************/

// 所有单位均为角度

extern "C"{
#include "flight_control.h"
#include "drivers/mpu9250/mpu9250.h"
#include "drivers/motor/motor.h"
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
}

#include "modules/pid/pid.h"

float yaw_set = 0, pitch_set = 0, roll_set = 0, velocity_set = 400.0f;
float yaw_set_last = 0, pitch_set_last = 0, roll_set_last = 0, velocity_set_last = 0;

/* PID Configuration */
#define SAMPLE_TIME 10

float pitch_Kp = 1;
float pitch_Ki = 0.6;
float pitch_Kd = 0.2;
//float pitch_Ki = 0;
//float pitch_Kd = 1;

float roll_Kp  = 1.2;
float roll_Ki  = 0.6;
float roll_Kd  = 0.2;
//float roll_Ki  = 5;
//float roll_Kd  = 1;

float yaw_Kp   = 1.2;
float yaw_Ki   = 0.6;
float yaw_Kd   = 0.2;
//float yaw_Ki   = 5;
//float yaw_Kd   = 1;

/* Flight Parameters */
#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define VELOCITY_MIN 0
#define VELOCITY_MAX 999
#define PID_PITCH_INFLUENCE 300
#define PID_ROLL_INFLUENCE 300
#define PID_YAW_INFLUENCE 300

#define _PI 3.14159265358979323846f

/* Motor control variables */
//int velocity;                       // global velocity
float bal_pitch = 0, bal_roll = 0;       // motor balances can vary between -100 & 100
float bal_yaw = 0;                     // throttle balance between axes -100:ac , +100:bd
int va, vb, vc, vd;                 //velocities

float ypr[3]     = {0.0f, 0.0f, 0.0f};       // yaw pitch roll values
float yprLast[3] = {0.0f, 0.0f, 0.0f};
float q[4]       = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0; // variables to hold latest sensor data values

float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magbias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

float deltat = 0.0f;             // integration interval for both filter schemes                              // used to calculate integration interval
float eInt[3] = {0.0f, 0.0f, 0.0f}; // vector to hold integral error for Mahony method

float GyroMeasError; // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
float beta;
float GyroMeasDrift; // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float zeta; // zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

float sum = 0;
uint32_t sumCount = 0;
int lastUpdate = 0, firstUpdate = 0, Now = 0; // used to calculate integration interval
int delt_t = 0; // used to control display output rate
timer_ticks_t count = 0;  // used to control display output rate

PID pitchReg(&ypr[1], &bal_pitch, &pitch_set, pitch_Kp, pitch_Ki, pitch_Kd, DIRECT);
PID rollReg(&ypr[2], &bal_roll, &roll_set, roll_Kp, roll_Ki, roll_Kd, DIRECT);
PID yawReg(&ypr[0], &bal_yaw, &yaw_set, yaw_Kp, yaw_Ki, yaw_Kd, DIRECT);

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz);

void initIMU(void) {
	float selfTest9250[6];

	/* Initialize motors */
	Motor_Init();
	trace_printf("Motor initialized.\n");

	/* Initialize related variables */
	GyroMeasError = _PI * (60.0f / 180.0f); // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
	GyroMeasDrift = _PI * (1.0f / 180.0f); // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	/* Initialize balancing */
	bal_pitch = 0;
	bal_roll = 0;
	bal_yaw = 0;

	/* Initialize PID */
	pitchReg.SetMode(AUTOMATIC);
	pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);

	rollReg.SetMode(AUTOMATIC);
	rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);

	yawReg.SetMode(AUTOMATIC);
	yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);
	trace_printf("PID initialized.\n");

	/* Detect whether MPU9250 is online */
	MPU9250_I2C_Init();
	resetMPU9250();

	if(MPU9250_I2C_ByteRead(MPU9250_ADDRESS, WHO_AM_I_MPU9250) != 0x71) {
		trace_printf("Could not find MPU9250!\n");
	}
	trace_printf("MPU9250 is online.\n");
	timer_sleep(100);

	/* MPU9250 self test and calibrate */
	MPU9250SelfTest(selfTest9250);
	trace_printf("x-axis self test: acceleration trim within : %6f %% of factory value\n", selfTest9250[0]);
	trace_printf("y-axis self test: acceleration trim within : %f %% of factory value\n", selfTest9250[1]);
	trace_printf("z-axis self test: acceleration trim within : %f %% of factory value\n", selfTest9250[2]);
	trace_printf("x-axis self test: gyration trim within : %f %% of factory value\n", selfTest9250[3]);
	trace_printf("y-axis self test: gyration trim within : %f %% of factory value\n", selfTest9250[4]);
	trace_printf("z-axis self test: gyration trim within : %f %% of factory value\n", selfTest9250[5]);
	timer_sleep(100);
	calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
	trace_printf("x gyro bias = %f\n", gyroBias[0]);
	trace_printf("y gyro bias = %f\n", gyroBias[1]);
	trace_printf("z gyro bias = %f\n", gyroBias[2]);
	trace_printf("x accel bias = %f\n", accelBias[0]);
	trace_printf("y accel bias = %f\n", accelBias[1]);
	trace_printf("z accel bias = %f\n", accelBias[2]);
	timer_sleep(100);

	/* Initialize MPU9250 and AK8963 */
	initMPU9250();
	trace_printf("MPU9250 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
	timer_sleep(500);
    initAK8963(magCalibration);
    trace_printf("AK8963 initialized for active data mode....\n"); // Initialize device for active mode read of magnetometer
    trace_printf("Accelerometer full-scale range = %f  g\n", 2.0f*(float)(1<<Ascale));
    trace_printf("Gyroscope full-scale range = %f  deg/s\n", 250.0f*(float)(1<<Gscale));
    if(Mscale == 0) trace_printf("Magnetometer resolution = 14  bits\n");
    if(Mscale == 1) trace_printf("Magnetometer resolution = 16  bits\n");
    if(Mmode == 2) trace_printf("Magnetometer ODR = 8 Hz\n");
    if(Mmode == 6) trace_printf("Magnetometer ODR = 100 Hz\n");
    timer_sleep(100);
    getAres(); // Get accelerometer sensitivity
    getGres(); // Get gyro sensitivity
    getMres(); // Get magnetometer sensitivity
    trace_printf("Accelerometer sensitivity is %f LSB/g \n", 1.0f/aRes);
    trace_printf("Gyroscope sensitivity is %f LSB/deg/s \n", 1.0f/gRes);
    trace_printf("Magnetometer sensitivity is %f LSB/G \n", 1.0f/mRes);
    magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
    magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
    magbias[2] = +125.;  // User environmental x-axis correction in milliGauss
}

void updateYPR(void) {
	int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];  // Stores the 16-bit signed magnetometer sensor output
	int16_t tempCount; // Stores the real internal chip temperature in degrees Celsius
	float temperature;

	// If intPin goes high, all data registers have new data
	if (MPU9250_I2C_ByteRead(MPU9250_ADDRESS, INT_STATUS) & 0x01) { // On interrupt, check if data ready interrupt
		readAccelData(accelCount);  // Read the x/y/z adc values
		// Now we'll calculate the accleration value into actual g's
		ax = (float) accelCount[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
		ay = (float) accelCount[1] * aRes - accelBias[1];
		az = (float) accelCount[2] * aRes - accelBias[2];

		readGyroData(gyroCount);  // Read the x/y/z adc values
		// Calculate the gyro value into actual degrees per second
		gx = (float) gyroCount[0] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
		gy = (float) gyroCount[1] * gRes - gyroBias[1];
		gz = (float) gyroCount[2] * gRes - gyroBias[2];

		readMagData(magCount);  // Read the x/y/z adc values
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float) magCount[0] * mRes * magCalibration[0] - magbias[0]; // get actual magnetometer value, this depends on scale being set
		my = (float) magCount[1] * mRes * magCalibration[1] - magbias[1];
		mz = (float) magCount[2] * mRes * magCalibration[2] - magbias[2];

		Now = timer_get_us();
		deltat = (float) ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
		lastUpdate = Now;

		sum += deltat;
		sumCount++;

		//MadgwickQuaternionUpdate(ax, ay, az, gx*_PI/180.0f, gy*_PI/180.0f, gz*_PI/180.0f,  my,  mx, mz);
		MahonyQuaternionUpdate(ax, ay, az, gx * _PI / 180.0f, gy * _PI / 180.0f, gz * _PI / 180.0f, my, mx, mz);

		// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
		// In this coordinate system, the positive z-axis is down toward Earth.
		// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
		// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
		// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
		// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
		// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
		// applied in the correct order which for this configuration is yaw, pitch, and then roll.
		// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
		ypr[0] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
				q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
//		ypr[0] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
//				q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
//		ypr[0] = atan2(2.0f * (q[1] * q[2] - q[0] * q[3]),
//				q[0] * q[0] + q[1] * q[1] - 1);
		ypr[1] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		ypr[2] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
				q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
		ypr[1] *= 180.0f / _PI;
		ypr[0] *= 180.0f / _PI;
		ypr[0] -= -5.08f; // Declination at Hefei, Anhui is 5 degrees and 5 minutes (negative) on 2015-08-08
		ypr[2] *= 180.0f / _PI;
	}

	// Serial print and/or display at 0.5 s rate independent of data rates
	delt_t = systemTime - count;
	if (delt_t > 1500) { // update LCD once per half-second independent of read rate

	    trace_printf("ax = %f", 1000 * ax);
	    trace_printf(" ay = %f", 1000 * ay);
	    trace_printf(" az = %f  mg\n", 1000 * az);

	    trace_printf("gx = %f", gx);
	    trace_printf(" gy = %f", gy);
	    trace_printf(" gz = %f  deg/s\n", gz);

	    trace_printf("gx = %f", mx);
	    trace_printf(" gy = %f", my);
	    trace_printf(" gz = %f  mG\n", mz);

	    tempCount = readTempData();  // Read the adc values
	    temperature = ((float) tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
	    trace_printf("temperature = %f  C\n", temperature);

	    trace_printf("q0 = %f\n", q[0]);
	    trace_printf("q1 = %f\n", q[1]);
	    trace_printf("q2 = %f\n", q[2]);
	    trace_printf("q3 = %f\n", q[3]);

	    trace_printf("Yaw, Pitch, Roll: %f %f %f\n", ypr[0], ypr[1], ypr[2]);
	    trace_printf("average rate = %f\n\n\n\r", (float) sumCount / sum);

	    count = systemTime;

	    if (count > 1 << 21) {
			systemTime = 0; // start the timer over again if ~30 minutes has passed
			count = 0;
			deltat = 0;
			lastUpdate = timer_get_us();
		}

		sum = 0;
		sumCount = 0;
	}
}

void computePID(void) {
	  if((pitch_set < PITCH_MIN) || (pitch_set > PITCH_MAX)) pitch_set = pitch_set_last;
	  if((roll_set < ROLL_MIN) || (roll_set > ROLL_MAX)) roll_set = roll_set_last;
	  if((yaw_set < YAW_MIN) || (yaw_set > YAW_MAX)) yaw_set = yaw_set_last;

	  pitch_set_last = pitch_set;
	  roll_set_last = roll_set;
	  pitch_set_last = pitch_set;

	  //////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////
	  // FOR TEST ONLY /////////////////////////////////////
	  //////////////////////////////////////////////////////
	  //////////////////////////////////////////////////////
	  ypr[0] = 0;

	  if(abs(ypr[0]-yprLast[0])>30) ypr[0] = yprLast[0];
	  if(abs(ypr[1]-yprLast[1])>30) ypr[1] = yprLast[1];
	  if(abs(ypr[2]-yprLast[2])>30) ypr[2] = yprLast[2];

	  yprLast[0] = ypr[0];
	  yprLast[1] = ypr[1];
	  yprLast[2] = ypr[2];

	  pitchReg.Compute();
	  rollReg.Compute();
	  yawReg.Compute();

	  //trace_printf("yprPID: %f, %f, %f\n", bal_yaw, bal_pitch, bal_roll);
}

void calculateVelocities(void) {

	if ((velocity_set < VELOCITY_MIN) || (velocity_set > VELOCITY_MAX))
		velocity_set = velocity_set_last;

	velocity_set_last = velocity_set;

//	va = ((abs(-100.0 + bal_roll) + abs( 100.0 + bal_pitch) + abs( 100.0 + bal_yaw)) / 300.0) * velocity_set;
//	vb = ((abs( 100.0 + bal_roll) + abs( 100.0 + bal_pitch) + abs(-100.0 + bal_yaw)) / 300.0) * velocity_set;
//	vc = ((abs( 100.0 + bal_roll) + abs(-100.0 + bal_pitch) + abs( 100.0 + bal_yaw)) / 300.0) * velocity_set;
//	vd = ((abs(-100.0 + bal_roll) + abs(-100.0 + bal_pitch) + abs(-100.0 + bal_yaw)) / 300.0) * velocity_set;

	va = (-1.0 * bal_roll) + (-1.0 * bal_pitch) + (-1.0 * bal_yaw) + velocity_set;
	vb = ( 1.0 * bal_roll) + (-1.0 * bal_pitch) + ( 1.0 * bal_yaw) + velocity_set;
	vc = ( 1.0 * bal_roll) + ( 1.0 * bal_pitch) + (-1.0 * bal_yaw) + velocity_set;
	vd = (-1.0 * bal_roll) + ( 1.0 * bal_pitch) + ( 1.0 * bal_yaw) + velocity_set;

	if(va < VELOCITY_MIN) { va = VELOCITY_MIN; }
	if(va > VELOCITY_MAX) { va = VELOCITY_MAX; }
	if(vb < VELOCITY_MIN) { vb = VELOCITY_MIN; }
	if(vb > VELOCITY_MAX) { vb = VELOCITY_MAX; }
	if(vc < VELOCITY_MIN) { vc = VELOCITY_MIN; }
	if(vc > VELOCITY_MAX) { vc = VELOCITY_MAX; }
	if(vd < VELOCITY_MIN) { vd = VELOCITY_MIN; }
	if(vd > VELOCITY_MAX) { vd = VELOCITY_MAX; }

	//trace_printf("speed: %d, %d, %d, %d\n", va, vb, vc, vd);
}

void updateMotors(void) {
	Motor1_SetSpeed(va);
	Motor2_SetSpeed(vb);
	Motor3_SetSpeed(vc);
	Motor4_SetSpeed(vd);
}

void errorHappened(void) {
	trace_printf("Error happened!\n");
	Motor1_SetSpeed(0);
	Motor2_SetSpeed(0);
	Motor3_SetSpeed(0);
	Motor4_SetSpeed(0);
	for(;;) {
		trace_printf("Error happened!\n");
	}
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3
			+ _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
			+ my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
			+ _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q2 * (2.0f * q1q2 + _2q3q4 - ay)
			- _2bz * q3
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q4 + _2bz * q2)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q3
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ _2bz * q4
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q3 + _2bz * q1)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q4 - _4bz * q2)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax)
			+ _2q4 * (2.0f * q1q2 + _2q3q4 - ay)
			- 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az)
			+ (-_4bx * q3 - _2bz * q1)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (_2bx * q2 + _2bz * q4)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ (_2bx * q1 - _4bz * q3)
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay)
			+ (-_4bx * q4 + _2bz * q2)
					* (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
			+ (-_2bx * q1 + _2bz * q3)
					* (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
			+ _2bx * q2
					* (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
	norm = 1.0f / norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);  // normalise quaternion
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
		float gz, float mx, float my, float mz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3]; // short name local variable for readability
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f)
		return; // handle NaN
	norm = 1.0f / norm;        // use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4)
			+ 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4)
			+ 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2)
			+ 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f) {
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	} else {
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}
