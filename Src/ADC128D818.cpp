extern "C"
{
#include "main.h"
}

#include "i2c.h"
#include "stm32f3xx_hal_i2c.h"
#include "ADC128D818.h"

//-------------------------------------------------------------------------------

TempI2C_ADC128D818::TempI2C_ADC128D818(I2C_HandleTypeDef * hi2c, uint8_t i2c_addr)
{
	// on STM32, I2C addresses have to be shifted 1 bit left to allow for hardware insertion of the r/w bit as the MSb
	uint8_t _i2cAddr = (i2c_addr << 1);
   
	int status;
	//status = HAL_I2C_Master_Receive(hi2c, (uint16_t)_i2cAddr<<1, &_shadow, 1, 10);
	status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)_i2cAddr, 3, 1000); 

	if (status != HAL_OK) {
		m_u16I2CAddr = 0; // does not answer
	} else {
		m_u16I2CAddr = _i2cAddr;
		m_hi2c = hi2c;
		bool isNotReady;
		isNotReady = getNotReady();
	}
	disabled_mask = 0;

	ref_v = 2.56f;
	ref_mode = INTERNAL_REF;
	op_mode = SINGLE_ENDED_WITH_TEMP;
	conv_mode = CONTINUOUS;
	initialize();
}
uint16_t TempI2C_ADC128D818::baseAddress(int nTh)
{
	uint8_t addresses[] = { 0x1D, 0x1E, 0x1F, 0x2D, 0x2E, 0x2F, 0x35, 0x36, 0x37 };
	return addresses[nTh];
}
void TempI2C_ADC128D818::initialize()
{
	bool isNotReady = getNotReady();
	while (isNotReady){
		HAL_Delay(35);
		isNotReady = getNotReady();
	}
	writeRegister(CONFIG_REG, 0);
	// Reset all
	writeRegister(CONFIG_REG, 1 << 7);

	HAL_Delay(100);

	writeRegister(ADV_CONFIG_REG, ref_mode | (op_mode << 1));

	writeRegister(CONV_RATE_REG, conv_mode);

	writeRegister(CHANNEL_DISABLE_REG, disabled_mask);

	// set start bit in configuration (interrupts disabled)
	writeRegister(CONFIG_REG, 1);
}
bool TempI2C_ADC128D818::getNotReady()
{
	BusyStatusRegister busyReg;
	readRegister(BUSY_STATUS_REG, &busyReg.mbyte, false);

	return (busyReg.mbits.busy);
}



//-------------------------------------------------------------------------------
float TempI2C_ADC128D818::getTemp(uint8_t channel)
{
	return m_fTemp[channel];
}
float TempI2C_ADC128D818::acquireTemp(uint8_t channel, bool bIT)
{
	int status;
	if (bIT) {
		m_tempRegister[channel].mTempX = 0;
	}
	status = readRegister(TEMPERATURE_REG_BASE + channel, &m_tempRegister[channel].mdata[0], bIT);
	if (bIT) {
		currentChannel = channel; // remember for DMA
		return 0.0f;
	} else {
		storeTemp(channel);
		return getTemp(channel);
	}
}
void TempI2C_ADC128D818::storeTemp(uint8_t channel)
{
	uint8_t data; // we have to swap
	data = m_tempRegister[channel].mdata[0];
	m_tempRegister[channel].mdata[0] = m_tempRegister[channel].mdata[1];
	m_tempRegister[channel].mdata[1] = data;
	m_fTemp[channel] = m_tempRegister[channel].mTempS / 256.0F;
}
void TempI2C_ADC128D818::storeTempDMA(){
	storeTemp(currentChannel);
}
//-------------------------------------------------------------------------------
TempI2C_ADC128D818 * pSensorIT;
unsigned short TempI2C_ADC128D818::readRegister(uint16_t reg, uint8_t * ptrData, bool bIT)
{
	unsigned short retVal = 0;
	uint16_t dataSize;

	if (m_u16I2CAddr) {
		int status;
		if (reg >= 0x20 || reg <= 0x27) {
			// 16 bits read temperature
			dataSize = 2;
		} else {
			dataSize = 1;
		}

		if (bIT) {
			// using DMA and Interrupt
			pSensorIT = this;
			status = HAL_I2C_Mem_Read_DMA(m_hi2c, m_u16I2CAddr, reg, I2C_MEMADD_SIZE_8BIT, ptrData, dataSize);
			retVal = status;  
		} else {
			status = HAL_I2C_Mem_Read(m_hi2c, m_u16I2CAddr, reg, I2C_MEMADD_SIZE_8BIT, ptrData, dataSize, 1000);
			retVal = status; 
		}

	}
	return retVal;
}
//-------------------------------------------------------------------------------
void TempI2C_ADC128D818::writeRegister(uint16_t reg, unsigned newValue)
{
	int status = HAL_ERROR;
	uint16_t dataSize;

	if (m_u16I2CAddr) {
		uint8_t data[3];

		data[0] = newValue & 0xFF;
		if (reg < 0x20 || reg > 0x27) {
			// 8 bits  
			dataSize = 1;
		} else {
			dataSize = 2;
			data[1] = (newValue & 0xFF00) >> 8;
		}
		status = HAL_I2C_Mem_Write(m_hi2c, m_u16I2CAddr, reg, I2C_MEMADD_SIZE_8BIT, &data[0], dataSize, 1000);
	}
	//return ((status == HAL_OK)); // HAL_OK is 0 as well
}





void TempI2C_ADC128D818::setShutdown(bool newShutdown)
{
	DeepShutdownRegister reg;

	reg.mbits.deepShutdownEnable = newShutdown;
	writeRegister(DEEP_SHUTDOWN_REG, reg.mbyte);
}


void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if (pSensorIT) {
		pSensorIT->storeTempDMA();
	}
}