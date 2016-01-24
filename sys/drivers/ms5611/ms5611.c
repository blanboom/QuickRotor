// https://github.com/jarzebski/Arduino-MS5611
/*
MS5611.cpp - Class file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.
Version: 1.0.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl
This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <arm_math.h>
#include "ms5611.h"
#include "hal_ms5611.h"

uint16_t fc[6];
uint8_t ct;
uint8_t uosr;
int32_t TEMP2;
int64_t OFF2, SENS2;

void MS5611_Reset(void);
void MS5611_ReadPROM(void);
uint16_t MS5611_ReadRegister16(uint8_t reg);
uint32_t MS5611_ReadRegister24(uint8_t reg);

void MS5611_Begin(ms5611_osr_t osr)
{
    MS5611_Reset();

    MS5611_SetOversampling(osr);

    MS5611_Delay(100);

    MS5611_ReadPROM();
}

// Set oversampling value
void MS5611_SetOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611_GetOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void MS5611_Reset(void)
{
	MS5611_I2C_CommandWrite(MS5611_ADDRESS, MS5611_CMD_RESET);
}

void MS5611_ReadPROM(void)
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
	fc[offset] = MS5611_ReadRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t MS5611_ReadRawTemperature(void)
{
	MS5611_I2C_CommandWrite(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + uosr);

    MS5611_Delay(ct);

    return MS5611_ReadRegister24(MS5611_CMD_ADC_READ);
}

uint32_t MS5611_ReadRawPressure(void)
{
	MS5611_I2C_CommandWrite(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + uosr);

    MS5611_Delay(ct);

    return MS5611_ReadRegister24(MS5611_CMD_ADC_READ);
}

int32_t MS5611_ReadPressure(bool compensation)
{
    uint32_t D1 = MS5611_ReadRawPressure();

    uint32_t D2 = MS5611_ReadRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

	OFF2 = 0;
	SENS2 = 0;

	if (TEMP < 2000)
	{
	    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
    }

    uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double MS5611_ReadTemperature(bool compensation)
{
    uint32_t D2 = MS5611_ReadRawTemperature();
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation)
    {
	if (TEMP < 2000)
	{
	    TEMP2 = (dT * dT) / (2 << 30);
	}
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double MS5611_GetAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double MS5611_GetSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611_ReadRegister16(uint8_t reg)
{
	uint16_t value;
	uint8_t tmp[2];

	MS5611_I2C_BufferRead(MS5611_ADDRESS, reg, 2, tmp);

    value = tmp[0] << 8 | tmp[1];

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611_ReadRegister24(uint8_t reg)
{
    uint32_t value;
    uint8_t tmp[3];

    MS5611_I2C_BufferRead(MS5611_ADDRESS, reg, 2, tmp);

    value = ((int32_t)tmp[2] << 16) | ((int32_t)tmp[1] << 8) | tmp[0];

    return value;
}
