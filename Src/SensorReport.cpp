extern "C"
{
#include "main.h"
}

#include "i2c.h"
#include <gpio.h>
#include <tim.h>
#include "stm32f3xx_hal_i2c.h"
#include "SensorReport.h"
#include "ADC128D818.h"
#include <string.h>

#ifdef USE_SERIAL
#include "Serial.h"

#define SERIAL_BUFFER_SIZE 50
static char bufferOutConsole[SERIAL_BUFFER_SIZE];
static char bufferInConsole[SERIAL_BUFFER_SIZE];

Serial * pSerial;
#endif

void sendSerial(const char* message)
{
	pSerial->output.puts(message);
}

#define SENSOR_COUNT 1
I2C_ADC128D818 * sensors[SENSOR_COUNT];
#define CHANNEL_COUNT 8 // should be in object


char * my_itoa(int n, int maxVal = 100000)
{
	static char message[11];
	int leadingZeros = 0;
	bool negative = false;
	char d;
	int i = 0;
	char nextC = 0;
	message[0] = 0;
	if (n < 0) {
		negative = true;
		n = -n;
	}
	while (maxVal > 9 && (d = n / maxVal) == 0) {
		leadingZeros++;
		maxVal /= 10;
	}
	while (i < leadingZeros - 1) {
		message[i++] = ' ';
	}
	if (negative != 0) {
		message[i++] = '-';
	} else {
		message[i++] = ' ';
	}
	while (maxVal != 0) {
		d = '0' + n / maxVal;
		message[i++] = d;
		n = n % maxVal;
		maxVal = maxVal / 10;
	}
	message[i + 1] = 0;
	return &message[0];
}

void scan()
{
	int returnStatus;
	uint8_t totalDevicesFound = 0;
	for (uint8_t s = 0; s <= 0x7F; s++) {
		returnStatus = 0;
		returnStatus = HAL_I2C_IsDeviceReady(&hi2c1, s, 2, 80);    // 2 retries, 80ms timeout
		if(returnStatus != HAL_OK)
		{
			if (hi2c1.ErrorCode != HAL_I2C_ERROR_TIMEOUT) {
				 // bus problem
				pSerial->output.puts("I2C bus problem\r\n");
				return;
			}
		}
		else
		{
			pSerial->output.puts("Found device at address - ");
			pSerial->output.puts(my_itoa(s,10000));
			pSerial->output.puts("\r\n");

			totalDevicesFound++;
		}
	}
	if (!totalDevicesFound){
		pSerial->output.puts("No I2c devices found\r\n");
	}
}



void initializeSensorReport()
{

#ifdef USE_SERIAL
	pSerial = createSerial(&huart2, bufferInConsole, SERIAL_BUFFER_SIZE, bufferOutConsole, SERIAL_BUFFER_SIZE);
	pSerial->input.initialize();
	pSerial->output.puts("\r\nReady\r\n");
#endif
	//scan();
	for (int i=0; i< SENSOR_COUNT;i++){
		sensors[i] = new I2C_ADC128D818(&hi2c1,I2C_ADC128D818::baseAddress(i));
		if (! sensors[i]->isActive()) {
#ifdef USE_SERIAL
			pSerial->output.puts("Sensor not responding: ");
			pSerial->output.puts(my_itoa(i));
			pSerial->output.puts("\r\n");
#endif
		}
	}
}

float mcp9700Convert(float measurement)
{
	// or any sensor with slope 10mV/dC like TMP235-Q1
	float refTemp = 13.0;
	float refV = 0.6;
	float slope = 0.010;

	float temp;
	temp = refTemp +  (measurement - refV) / slope;
	return temp;
}

float mcp9701Convert(float measurement)
{
	// or any sensor with slope 19.5mV/dC like TMP236-Q1
	float refTemp = 13.0;
	float refV = 0.6;
	float slope = 0.0195;

	float temp;
	temp = refTemp +  (measurement - refV) / slope;
	return temp;
}

static float voltage;
static float temp;
void doReport() {



	pSerial->output.puts("mV \t");
	for (int i = 0; i < SENSOR_COUNT; i++) {
		for (int j = 0; j < CHANNEL_COUNT; j++) {
			voltage = sensors[i]->getVoltage(j);
			pSerial->output.puts(my_itoa((int)(voltage*1000.0)));
			pSerial->output.puts("\t");
		}
		pSerial->output.puts("\r\ntemp \t");
		for (int j = 0; j < CHANNEL_COUNT; j++) {
			temp = sensors[i]->getVoltage(j);
			pSerial->output.puts(my_itoa((int)mcp9701Convert(temp)));
			pSerial->output.puts("\t");
		}
	}
	pSerial->output.puts("\r\n");

}

static int which;
static int which_channel;
void doAcquisitionStep()
{
	doLedToggle();
	sensors[which++]->acquireVoltage(which_channel,false);
	if (which >= SENSOR_COUNT){
		which = 0;
		which_channel++;
		which_channel %= CHANNEL_COUNT;
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim1) {
		doAcquisitionStep();
	} else {
		Error_Handler();
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	doAcquisitionStep();
}