# QuickRotor

PCB 四旋翼飞行器，大小在 10cm x 10cm 之内，方便以较低的成本进行制作。

四旋翼采用 STM32F103RET6，使用 [GNU ARM Eclipse](http://gnuarmeclipse.github.io) 进行开发。遥控器（地面站）采用 Arduino UNO, 开发环境为 PlatformIO.

该飞行器只是个人的兴趣项目，目前大部分程序已完成，但还需进一步的完善才能正常工作。

## Acknowledgements

该飞行器的制作使用了以下软硬件项目：

硬件：

- [Sparkfun Eagle Library](https://github.com/sparkfun/SparkFun-Eagle-Libraries)</br>Sparkfun 的 Eagle 封装库，包含大量元器件与模块
- [openenstm32hw](https://github.com/gbulmer/openstm32hw)</br>开源 STM32 开发板。本飞行器的原理图和 PCB 布局参考了这一项目
- [MPU-9250 Footprint](http://www.snapeda.com/parts/MPU-9250/InvenSense/view-part/)</br>来自 SnapEDA 的 MPU-9250 封装库
- [MS5611 Footprint](https://github.com/zrecommerce/freeIMU/blob/master/hardware/schematics/0.4.1/eagle/freeIMU/freeimu-components.lbr)</br>来自 FreeIMU 的 MS5611 封装库

软件：

- [Arduino-Quadcopter](https://github.com/strangedev/Arduino-Quadcopter)</br>一个简单的 Arduino 四旋翼飞行器
- [MPU9250AHRS](https://developer.mbed.org/users/onehorse/code/MPU9250AHRS/)</br>用于 mbed 的 MPU9250 驱动程序，能够获取 MPU9250 传感器中的数据，并通过相关算法进行姿态融合</br>原始代码主要用于 mbed, 在本项目中进行了修改，以便于在 STM32 中使用
- [arduino-nrf24l01](https://github.com/aaronds/arduino-nrf24l01)</br>用于 Arduino 的 nRF24L01 驱动程序</br>原始代码用于 Arduino, 经过移植以便于在 STM32 中使用。但在 IO 口、时钟、SPI 等的初始化上尚未完善，如果需要用于其他项目，请在 Mirf.h, Mirf.cpp, MirfHardwareSpiDriver.cpp 中，根据实际使用的 IO 引脚和 SPI 端口进行相应的修改
- [Arduino-MS5611](https://github.com/jarzebski/Arduino-MS5611)</br>用于 Arduino 的 MS5611 驱动程序</br>原始代码用于 Arduino, 经过移植以便于在 STM32 中使用
- [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library)</br>用于 Arduino 的 PID 算法库</br>原始代码用于 Arduino, 经过移植以便于在 STM32 中使用


