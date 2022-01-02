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

#define SENSOR_COUNT 8
TempI2C_ADC128D818 * sensors[SENSOR_COUNT];


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
		pSerial->output.puts("No devices found\r\n");
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
		sensors[i] = new TempI2C_ADC128D818(&hi2c1,TempI2C_ADC128D818::baseAddress + i);
	}
}


static float temp;
void doReport() {



	pSerial->output.puts("temp \t");
	for (int i = 0; i < SENSOR_COUNT; i++) {
		temp = sensors[i]->getTemp();
		pSerial->output.puts(my_itoa((int)temp));
		pSerial->output.puts("\t");
	}
	pSerial->output.puts("\r");

}

static int which;
void doAcquisitionStep()
{
	doLedToggle();
	sensors[which++]->acquireTemp(true);
	if (which >= SENSOR_COUNT){
		which = 0;
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