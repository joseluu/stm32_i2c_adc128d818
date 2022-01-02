#pragma once
#ifndef __ADC128D818_H
#define __ADC128D818_H


#include <inttypes.h>
#include "i2c.h"

//#define CONFIG_REG    0x00
//#define CONV_RATE_REG 0x07
//#define CHANNEL_DISABLE_REG 0x08
//#define ADV_CONFIG_REG 0x0B
//#define BUSY_STATUS_REG 0x0C

#define READ_REG_BASE 0x20
#define LIMIT_REG_BASE 0x2A

#define TEMP_REGISTER 0x27

#define START_BIT 0
#define INIT_BIT 7

#define EXT_REF_ENABLE 0
#define MODE_SELECT_1 1
#define MODE_SELECT_2 2

#define BUSY_BIT 0
#define NOT_READY_BIT 1

enum reference_mode_t {
	INTERNAL_REF = 0,
	EXTERNAL_REF = 1
};

enum conv_mode_t {
	LOW_POWER,
	CONTINUOUS,
	ONE_SHOT
};

enum operation_mode_t {
	SINGLE_ENDED_WITH_TEMP = 0,
	SINGLE_ENDED           = 1,
	DIFFERENTIAL           = 2,
	MIXED                  = 3
};

class TempI2C_ADC128D818 {
	
public: 
	
	typedef enum { comparator_mode = 0, interrupt_mode } ThermostatMode;
	typedef enum { one_samples = 0, two_samples, four_samples, six_samples } ThermostatFaultTolerance;
	typedef enum { active_low = 0, active_high } OSPolarity;

	TempI2C_ADC128D818(I2C_HandleTypeDef * hi2c, uint8_t i2c_addr);

	bool getNotReady();

	// Temperature and temperature ranges in degrees centigrade
	void setReference(float ref_voltage);
	void setReferenceMode(reference_mode_t mode);
	void setOperationMode(operation_mode_t mode);
	void setDisabledMask(uint8_t disabled_mask);
	void setConversionMode(conv_mode_t mode);
	void begin(void);  
	uint8_t conversions_done(void);
	uint16_t read(uint8_t channel);
	float readConverted(uint8_t channel);
	float readTemperatureConverted(void);
  
	bool isActive();
      

private:

	typedef enum  { CONFIG_REG = 0, 
			INT_STATUS_REG,
			INT_MASK_REG,
			CONV_RATE_REG = 7,
			CHANNEL_DISABLE_REG,
			ADV_CONFIG_REG =0x0B,
			BUSY_STATUS_REG = 0x0C
			 } ADC128D818Register;
	typedef union {
		struct {
			uint8_t start : 1;
			uint8_t intEnable : 1;
			uint8_t reserved : 1;
			uint8_t intClear : 1;
			uint8_t reserved2 : 3;
			uint8_t initialization : 1;
		} mbits;
		uint8_t mbyte;
	} ConfigurationRegister;
	typedef union {
		struct {
			uint8_t conversionRate : 1;
			uint8_t reserved : 7;
		} mbits;
		uint8_t mbyte;
	} ConversionRateRegister;

	typedef union {
		struct {
			uint8_t oneShot : 1;
			uint8_t reserved : 7;
		} mbits;
		uint8_t mbyte;
	} OneShotRegister;
	typedef union {
		struct {
			uint8_t deepShutdownEnable : 1;
			uint8_t reserved : 7;
		} mbits;
		uint8_t mbyte;
	} DeepShutdownRegister;

	typedef union {
		struct {
			uint8_t externalReferenceEnable : 1;
			uint8_t modeSelect : 2;
			uint8_t reserved : 5;
		} mbits;
		uint8_t mbyte;
	} AdvancedConfigurationRegister;

	typedef union  {
		struct {
			uint8_t busy : 1;
			uint8_t notReady : 1;
			uint8_t reserved : 6;
		} mbits;
		uint8_t mbyte;
	} BusyStatusRegister;

	uint8_t disabled_mask;
	float ref_v;

	I2C_HandleTypeDef * m_hi2c;
	bool i2cInitialized;
	uint16_t m_u16I2CAddr;

	unsigned short i2cError;
      
	reference_mode_t ref_mode;
	operation_mode_t op_mode;
	conv_mode_t conv_mode;

	unsigned short getReg(ADC128D818Register reg, uint8_t * ptrData, bool bIT);
	void setReg(ADC128D818Register reg, unsigned newValue);

	void initI2c();
	void setRegisterAddress(uint8_t reg_addr);
	void setRegister(uint8_t reg_addr, uint8_t value);
	uint8_t readCurrentRegister8();

	
};


#endif
