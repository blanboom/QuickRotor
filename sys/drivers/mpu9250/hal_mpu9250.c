#include "hal_mpu9250.h"

void MPU9250_I2C_Init(void) {
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable I2C and GPIO clocks */
    RCC_APB1PeriphClockCmd(MPU9250_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(MPU9250_I2C_RCC_Port, ENABLE);

    /* Configure I2C pins: SCL and SDA */
    GPIO_InitStructure.GPIO_Pin = MPU9250_I2C_SCL_Pin | MPU9250_I2C_SDA_Pin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(MPU9250_I2C_Port, &GPIO_InitStructure);
//    GPIO_InitStructure.GPIO_Pin = MPU9250_INT_PIN;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//    GPIO_Init(MPU9250_INT_PORT, &GPIO_InitStructure);

    I2C_DeInit(MPU9250_I2C);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0xA0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = MPU9250_I2C_Speed;

    /* Apply I2C configuration after enabling it */
    I2C_Init(MPU9250_I2C, &I2C_InitStructure);
    /* I2C Peripheral Enable */
    I2C_Cmd(MPU9250_I2C, ENABLE);
}


/**
 * @brief  Writes one byte to the  MPU9250.
 * @param  slaveAddr : slave address MPU9250_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU9250.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void MPU9250_I2C_ByteWrite(u8 slaveAddr, u8 writeAddr, u8 data)
{
    // ENTR_CRT_SECTION();

    /* Send START condition */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for write */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU9250's internal address to write to */
    I2C_SendData(MPU9250_I2C, writeAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(MPU9250_I2C, data);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(MPU9250_I2C, ENABLE);

    // EXT_CRT_SECTION();
}

/**
 * @brief  Reads a block of data from the MPU9250.
 * @param  slaveAddr  : slave address MPU9250_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer that receives the data read from the MPU9250.
 * @param  readAddr : MPU9250's internal address to read from.
 * @param  NumByteToRead : number of bytes to read from the MPU9250 ( NumByteToRead >1  only for the Mgnetometer readinf).
 * @return None
 */
void MPU9250_I2C_BufferRead(u8 slaveAddr, u8 readAddr, u16 NumByteToRead, u8* pBuffer)
{
    // ENTR_CRT_SECTION();

    /* While the bus is busy */
    while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for write */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(MPU9250_I2C, ENABLE);

    /* Send the MPU9250's internal address to write to */
    I2C_SendData(MPU9250_I2C, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for read */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the MPU9250 */
            *pBuffer = I2C_ReceiveData(MPU9250_I2C);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }

    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
    // EXT_CRT_SECTION();
}


char MPU9250_I2C_ByteRead(u8 slaveAddr, u8 readAddr)
{
	char tmp = 0;

    // ENTR_CRT_SECTION();

    /* While the bus is busy */
    while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for write */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(MPU9250_I2C, ENABLE);

    /* Send the MPU9250's internal address to write to */
    I2C_SendData(MPU9250_I2C, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for read */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
	for (;;) {
		/* Disable Acknowledgement */
		I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);

		/* Send STOP Condition */
		I2C_GenerateSTOP(MPU9250_I2C, ENABLE);

		/* Test on EV7 and clear it */
		if (I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
			/* Read a byte from the MPU9250 */
			tmp = I2C_ReceiveData(MPU9250_I2C);

			break;
		}
	}


    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
    // EXT_CRT_SECTION();

    return tmp;
}
