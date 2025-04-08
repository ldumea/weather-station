/**
 * @file RAK4631-DeepSleep-LoRaWan.ino
 * @author Bernd Giesecke (bernd.giesecke@rakwireless.com)
 * @brief LoRaWan deep sleep example
 * Device goes into sleep after successful OTAA/ABP network join.
 * Wake up every SLEEP_TIME seconds. Set time in main.h 
 * @version 0.1
 * @date 2020-09-05
 * 
 * @copyright Copyright (c) 2020
 * 
 * @note RAK4631 GPIO mapping to nRF52840 GPIO ports
   RAK4631    <->  nRF52840
   WB_IO1     <->  P0.17 (GPIO 17)
   WB_IO2     <->  P1.02 (GPIO 34)
   WB_IO3     <->  P0.21 (GPIO 21)
   WB_IO4     <->  P0.04 (GPIO 4)
   WB_IO5     <->  P0.09 (GPIO 9)
   WB_IO6     <->  P0.10 (GPIO 10)
   WB_SW1     <->  P0.01 (GPIO 1)
   WB_A0      <->  P0.04/AIN2 (AnalogIn A2)
   WB_A1      <->  P0.31/AIN7 (AnalogIn A7)
 */
#include "main.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h> // Click to install library: http://librarymanager/All#Adafruit_BME680
#include "UVlight_LTR390.h" //Click here to get the library: http://librarymanager/All#RAK12019_LTR390

Adafruit_BME680 bme;
UVlight_LTR390 ltr;
// Might need adjustments
#define SEALEVELPRESSURE_HPA (1013.25)
float lux = 0.00;
float als = 0.00;
float uvi = 0.00;
float uvs = 0.00;
void sensor_init()
{
  
  Wire.end();
  delay(100);
  Wire.begin();
  ltr = UVlight_LTR390();
  if (!ltr.init())
	{
		Serial.println("Couldn't find LTR sensor!");
		// while (1)
		// 	delay(10);
	}
	Serial.println("Found LTR390 sensor!");
  
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    return;
  } else {
    Serial.println("Found BME680 sensor!");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_16X);
  bme.setHumidityOversampling(BME680_OS_8X);
  bme.setPressureOversampling(BME680_OS_8X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms



  //if set LTR390_MODE_ALS,get ambient light data, if set LTR390_MODE_UVS,get ultraviolet light data.
	// ltr.setMode(LTR390_MODE_ALS); //LTR390_MODE_UVS
	ltr.setMode(LTR390_MODE_UVS); //
	if (ltr.getMode() == LTR390_MODE_ALS)
	{
		Serial.println("In ALS mode");
	}
	else
	{
		Serial.println("In UVS mode");
	}

	ltr.setGain(LTR390_GAIN_18);
	Serial.print("Gain : ");
	switch (ltr.getGain())
	{
	case LTR390_GAIN_1:
		Serial.println(1);
		break;
	case LTR390_GAIN_3:
		Serial.println(3);
		break;
	case LTR390_GAIN_6:
		Serial.println(6);
		break;
	case LTR390_GAIN_9:
		Serial.println(9);
		break;
	case LTR390_GAIN_18:
		Serial.println(18);
		break;
	default:
		Serial.println("Failed to set gain");
		break;
	}
	ltr.setResolution(LTR390_RESOLUTION_20BIT);
	Serial.print("Integration Time (ms): ");
	switch (ltr.getResolution())
	{
	case LTR390_RESOLUTION_13BIT:
		Serial.println(13);
		break;
	case LTR390_RESOLUTION_16BIT:
		Serial.println(16);
		break;
	case LTR390_RESOLUTION_17BIT:
		Serial.println(17);
		break;
	case LTR390_RESOLUTION_18BIT:
		Serial.println(18);
		break;
	case LTR390_RESOLUTION_19BIT:
		Serial.println(19);
		break;
	case LTR390_RESOLUTION_20BIT:
		Serial.println(20);
		break;
	default:
		Serial.println("Failed to set Integration Time");
		break;
	}
	delay(200);

	ltr.setThresholds(100, 1000); //Set the interrupt output threshold range for lower and upper.
	if (ltr.getMode() == LTR390_MODE_ALS)
	{
		ltr.configInterrupt(true, LTR390_MODE_ALS); //Configure the interrupt based on the thresholds in setThresholds()
	}
	else
	{
		ltr.configInterrupt(true, LTR390_MODE_UVS);
	}

}

void bme680_get()
{
  Serial.println();
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  // if (ltr.newDataAvailable())
	// {
	// 	if (ltr.getMode() == LTR390_MODE_ALS)
	// 	{
	// 		Serial.printf("Lux Data:%0.2f-----Als Data:%d\r\n", ltr.getLUX(), ltr.readALS()); //calculate the lux
	// 	}
	// 	else
	// 	{
	// 		Serial.printf("Uvi Data:%0.2f-----Uvs Data:%d\r\n", ltr.getUVI(), ltr.readUVS());
	// 	}
	// }
  
}

/** Semaphore used by events to wake up loop task */
SemaphoreHandle_t taskEvent = NULL;

/** Timer to wakeup task frequently and send message */
SoftwareTimer taskWakeupTimer;

/** Buffer for received LoRaWan data */
uint8_t rcvdLoRaData[256];
/** Length of received data */
uint8_t rcvdDataLen = 0;

/**
 * @brief Flag for the event type
 * -1 => no event
 * 0 => LoRaWan data received
 * 1 => Timer wakeup
 * 2 => tbd
 * ...
 */
uint8_t eventType = -1;

/**
 * @brief Timer event that wakes up the loop task frequently
 * 
 * @param unused 
 */
void periodicWakeup(TimerHandle_t unused)
{
	// Switch on blue LED to show we are awake
	digitalWrite(LED_BUILTIN, HIGH);
	eventType = 1;
	// Give the semaphore, so the loop task will wake up
	xSemaphoreGiveFromISR(taskEvent, pdFALSE);
}

/**
 * @brief Arduino setup function. Called once after power-up or reset
 * 
 */
void setup(void)
{
  Serial.begin(115200);
  // while (!Serial); 
  Serial.println('Start program');

	// Create the LoRaWan event semaphore
	taskEvent = xSemaphoreCreateBinary();
	// Initialize semaphore
	xSemaphoreGive(taskEvent);

	// Initialize the built in LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

	// Initialize the connection status LED
	pinMode(LED_CONN, OUTPUT);
	digitalWrite(LED_CONN, HIGH);

  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
  delay(100);

#ifndef MAX_SAVE
	// Initialize Serial for debug output
	

	time_t timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		}
		else
		{
			break;
		}
	}
#endif
  Serial.println("Start sensor init");
  sensor_init();
	digitalWrite(LED_BUILTIN, LOW);

#ifndef MAX_SAVE
	Serial.println("=====================================");
	Serial.println("RAK4631 LoRaWan Deep Sleep Test");
	Serial.println("=====================================");
#endif

	// Initialize LoRaWan and start join request
	int8_t loraInitResult = initLoRaWan();

#ifndef MAX_SAVE
	if (loraInitResult != 0)
	{
		switch (loraInitResult)
		{
		case -1:
			Serial.println("SX126x init failed");
			break;
		case -2:
			Serial.println("LoRaWan init failed");
			break;
		case -3:
			Serial.println("Subband init error");
			break;
		case -4:
			Serial.println("LoRa Task init error");
			break;
		default:
			Serial.println("LoRa init unknown error");
			break;
		}

		// Without working LoRa we just stop here
		while (1)
		{
			Serial.println("Nothing I can do, just loving you");
			delay(5000);
		}
	}
	Serial.println("LoRaWan init success");
#endif

	// Take the semaphore so the loop will go to sleep until an event happens
	xSemaphoreTake(taskEvent, 10);
}

/**
 * @brief Arduino loop task. Called in a loop from the FreeRTOS task handler
 * 
 */
void loop(void)
{
  
	// Switch off blue LED to show we go to sleep
	digitalWrite(LED_BUILTIN, LOW);

	// Sleep until we are woken up by an event
	if (xSemaphoreTake(taskEvent, portMAX_DELAY) == pdTRUE)
	{
		// Switch on blue LED to show we are awake
		digitalWrite(LED_BUILTIN, HIGH);
		delay(500); // Only so we can see the blue LED

		// Check the wake up reason
		switch (eventType)
		{
		case 0: // Wakeup reason is package downlink arrived
#ifndef MAX_SAVE
			Serial.println("Received package over LoRaWan");
#endif
			if (rcvdLoRaData[0] > 0x1F)
			{
#ifndef MAX_SAVE
				Serial.printf("%s\n", (char *)rcvdLoRaData);
#endif
			}
			else
			{
#ifndef MAX_SAVE
				for (int idx = 0; idx < rcvdDataLen; idx++)
				{
					Serial.printf("%X ", rcvdLoRaData[idx]);
				}
				Serial.println("");
#endif
			}

			break;
		case 1: // Wakeup reason is timer
#ifndef MAX_SAVE
			Serial.println("Timer wakeup");
#endif
			/// \todo read sensor or whatever you need to do frequently
      if (! bme.performReading())
      {
        Serial.println("Failed to perform reading :(");
      }
      bme680_get();
      if (ltr.newDataAvailable())
      {
        if (ltr.getMode() == LTR390_MODE_ALS)
        {
          Serial.printf("Lux Data:%0.2f-----Als Data:%d\r\n", ltr.getLUX(), ltr.readALS()); //calculate the lux

          lux = ltr.getLUX()!=0?ltr.getLUX():lux;
          als = ltr.readALS()!=0?ltr.getLUX():als;
        }
        else
        {
          Serial.printf("Uvi Data:%0.2f-----Uvs Data:%d\r\n", ltr.getUVI(), ltr.readUVS());
          
          uvi = ltr.getUVI()!=0?ltr.getUVI():uvi;
          uvs = ltr.readUVS()!=0?ltr.readUVS():uvs;
        }
      } else {
        Serial.println("Failed to read UV sensor :(");
      }
			// Send the data package
			if (sendLoRaFrame(bme.temperature, bme.humidity, bme.pressure, bme.gas_resistance, lux, als, uvi, uvs))
			{
#ifndef MAX_SAVE
				Serial.println("LoRaWan package sent successfully");
#endif
			}
			else
			{
#ifndef MAX_SAVE
				Serial.println("LoRaWan package send failed");
				/// \todo maybe you need to retry here?
#endif
			}

			break;
		default:
#ifndef MAX_SAVE
			Serial.println("This should never happen ;-)");
#endif
			break;
		}
   digitalWrite(LED_BUILTIN, LOW);
		// Go back to sleep
		xSemaphoreTake(taskEvent, 10);
	}
}
