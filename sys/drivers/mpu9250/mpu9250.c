// https://developer.mbed.org/users/onehorse/code/MPU9250AHRS/

#include <drivers/mpu9250/mpu9250.h>

uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
uint8_t Gscale = GFS_250DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
uint8_t Mscale = MFS_16BITS; // MFS_14BITS or MFS_16BITS, 14-bit or 16-bit magnetometer resolution
uint8_t Mmode = 0x06; // Either 8 Hz 0x02) or 100 Hz (0x06) magnetometer data ODR
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

float magCalibration[3] = { 0, 0, 0 }; // Factory mag calibration

void getMres() {
	switch (Mscale) {
	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.0 * 4219.0 / 8190.0; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.0 * 4219.0 / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void getGres() {
	switch (Gscale) {
	// Possible gyro scales (and their register bit settings) are:
	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void getAres() {
	switch (Ascale) {
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
	// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}

void readAccelData(int16_t * destination) {
	uint8_t rawData[6];  // x/y/z accel register data stored here
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
	destination[0] = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
}

void readGyroData(int16_t * destination) {
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
	destination[0] = (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
	destination[1] = (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
	destination[2] = (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
}

void readMagData(int16_t * destination) {
	uint8_t rawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if (MPU9250_I2C_ByteRead(AK8963_ADDRESS, AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		MPU9250_I2C_BufferRead(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]); // Read the six raw data and ST2 registers sequentially into data array
		uint8_t c = rawData[6]; // End data read by reading ST2 register
		if (!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] =
					(int16_t) (((int16_t) rawData[1] << 8) | rawData[0]); // Turn the MSB and LSB into a signed 16-bit value
			destination[1] =
					(int16_t) (((int16_t) rawData[3] << 8) | rawData[2]); // Data stored as little Endian
			destination[2] =
					(int16_t) (((int16_t) rawData[5] << 8) | rawData[4]);
		}
	}
}

int16_t readTempData() {
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, TEMP_OUT_H, 2, &rawData[0]); // Read the two raw data registers sequentially into data array
	return (int16_t) (((int16_t) rawData[0]) << 8 | rawData[1]); // Turn the MSB and LSB into a 16-bit value
}

void resetMPU9250() {
	// reset device
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	MPU9250_Delay(100);
}

void initAK8963(float * destination) {
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	MPU9250_I2C_ByteWrite(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	MPU9250_Delay(10);
	MPU9250_I2C_ByteWrite(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	MPU9250_Delay(10);
	MPU9250_I2C_BufferRead(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]); // Read the x-, y-, and z-axis calibration values
	destination[0] = (float) (rawData[0] - 128) / 256.0f + 1.0f; // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float) (rawData[1] - 128) / 256.0f + 1.0f;
	destination[2] = (float) (rawData[2] - 128) / 256.0f + 1.0f;
	MPU9250_I2C_ByteWrite(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
	MPU9250_Delay(10);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	MPU9250_I2C_ByteWrite(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	MPU9250_Delay(10);
}

void initMPU9250() {
	// Initialize MPU9250 device
	// wake up device
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	MPU9250_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// get stable time source
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; the same rate set in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, GYRO_CONFIG);
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro

	// Set accelerometer configuration
	c = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, ACCEL_CONFIG);
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, ACCEL_CONFIG2);
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2) {
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

// reset device, reset all registers, clear gyro and accelerometer bias registers
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	MPU9250_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
	MPU9250_Delay(200);

// Configure device for bias calculation
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Turn on internal clock source
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, USER_CTRL, 0x00); // Disable FIFO and I2C master modes
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	MPU9250_Delay(15);

// Configure MPU9250 gyro and accelerometer for bias calculation
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, CONFIG, 0x01);   // Set low-pass filter to 188 Hz
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	MPU9250_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t) data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		MPU9250_I2C_BufferRead(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
		gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
		gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
		gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0] += (int32_t) gyro_temp[0];
		gyro_bias[1] += (int32_t) gyro_temp[1];
		gyro_bias[2] += (int32_t) gyro_temp[2];

	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0] /= (int32_t) packet_count;
	gyro_bias[1] /= (int32_t) packet_count;
	gyro_bias[2] /= (int32_t) packet_count;

	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t) accelsensitivity;
	}  // Remove gravity from the z-axis accelerometer bias calculation
	else {
		accel_bias[2] += (int32_t) accelsensitivity;
	}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

/// Push gyro biases to hardware registers
	/*  MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	 */
	dest1[0] = (float) gyro_bias[0] / (float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	dest1[1] = (float) gyro_bias[1] / (float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2] / (float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t) data[0] << 8) | data[1];
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t) data[0] << 8) | data[1];
	MPU9250_I2C_BufferRead(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t) data[0] << 8) | data[1];

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if (accel_bias_reg[ii] & mask)
			mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
	/*  MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	 MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	 */
// Output scaled accelerometer biases for manual subtraction in the main program
	dest2[0] = (float) accel_bias[0] / (float) accelsensitivity;
	dest2[1] = (float) accel_bias[1] / (float) accelsensitivity;
	dest2[2] = (float) accel_bias[2] / (float) accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

		MPU9250_I2C_BufferRead(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		MPU9250_I2C_BufferRead(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

// Configure the accelerometer for self-test
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	MPU9250_Delay(25); // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

		MPU9250_I2C_BufferRead(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);

		MPU9250_I2C_BufferRead(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t) (((int16_t) rawData[0] << 8) | rawData[1]); // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t) (((int16_t) rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t) (((int16_t) rawData[4] << 8) | rawData[5]);
	}

	for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
	MPU9250_I2C_ByteWrite(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
	MPU9250_Delay(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
	selfTest[4] = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
	selfTest[5] = MPU9250_I2C_ByteRead(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float) (2620 / 1 << FS)
			* (pow(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0 * ((float) (aSTAvg[i] - aAvg[i]))
				/ factoryTrim[i]; // Report percent differences
		destination[i + 3] = 100.0 * ((float) (gSTAvg[i] - gAvg[i]))
				/ factoryTrim[i + 3]; // Report percent differences
	}

}
