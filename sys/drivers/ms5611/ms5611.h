/*
MS5611.h - Header file for the MS5611 Barometric Pressure & Temperature Sensor Arduino Library.
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

#ifndef MS5611_h
#define MS5611_h

#include <stdbool.h>

#define MS5611_ADDRESS                (0x77<<1)

#define MS5611_CMD_ADC_READ           (0x00)
#define MS5611_CMD_RESET              (0x1E)
#define MS5611_CMD_CONV_D1            (0x40)
#define MS5611_CMD_CONV_D2            (0x50)
#define MS5611_CMD_READ_PROM          (0xA2)

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;

void MS5611_Begin(ms5611_osr_t);
uint32_t MS5611_ReadRawTemperature(void);
uint32_t MS5611_ReadRawPressure(void);
double MS5611_ReadTemperature(bool compensation);
int32_t MS5611_ReadPressure(bool compensation);
double MS5611_GetAltitude(double pressure, double seaLevelPressure); //seaLevelPressure = 101325
double MS5611_GetSeaLevel(double pressure, double altitude);
void MS5611_SetOversampling(ms5611_osr_t osr);
ms5611_osr_t MS5611_GetOversampling(void);



#endif
