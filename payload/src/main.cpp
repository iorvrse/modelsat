#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include <CRC.h>
#include <time.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include "uRTCLib.h"

#define RXD2 16
#define TXD2 17

#define MPU_INT	13

#define GPS_BAUD 9600

#define FILTER_DISK1	32
#define FILTER_DISK2	26

#define FILTER_RED_ANGLE		45
#define FILTER_GREEN_ANGLE		90
#define FILTER_BLUE_ANGLE		135
#define FILTER_CLEAR_ANGLE		0

typedef struct
{
	uint32_t packet_number;
	uint8_t state;
	uint8_t error_code;
	uint32_t mission_time;
	uint32_t pressure_payload;
	uint32_t pressure_container;
	int16_t descent_rate;
	int16_t temperature;
	uint8_t volt;
	float gps_latitude;
	float gps_longitude;
	int16_t gps_altitude;
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
	uint8_t lnln[4];
	int16_t iot1;
	int16_t iot2;
	uint32_t team_id;
} packet_t;

typedef enum : uint8_t
{
    CMD_TEAM_ID = 0,
    CMD_TELEM_ON,
    CMD_TELEM_OFF,
    CMD_RELEASE,
    CMD_CAL,
    CMD_FILTER,
	CMD_MISSION_TIME,
    WIFI_RSSI = 249,
    GCS_LOCATION_DATA,
    PAYLOAD_PING_DATA,
    PAYLOAD_TELEMETRY_DATA,
	CONTAINER_DATA,
	IOT1_STATION_DATA,
	IOT2_STATION_DATA
} message_type_t;

typedef enum : uint8_t
{
    FILTER_CODE_M = 'M',
    FILTER_CODE_F = 'F',
    FILTER_CODE_N = 'N',
    FILTER_CODE_R = 'R',
    FILTER_CODE_G = 'G',
    FILTER_CODE_B = 'B',
    FILTER_CODE_P = 'P',
    FILTER_CODE_Y = 'Y',
    FILTER_CODE_C = 'C'
} filter_code_t;

typedef enum : uint8_t
{
	READY_TO_FLIGHT = 0,
	ASCENT,
	SATELLITE_DESCENT,
	RELEASE,
	PAYLOAD_DESCENT,
	RECOVERY
} state_t;

packet_t packet = {
	.team_id = 632419
};
state_t state;
filter_code_t filter_code;
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
MPU6050 mpu;

Adafruit_BMP280 bmp;
Servo releaseServo, filterServo1, filterServo2;
uRTCLib rtc(0x68);
Preferences preferences;
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xF9, 0x97, 0xCA};
char filterColorChar[9] = {'M', 'F', 'N', 'R', 'G', 'B', 'P', 'Y', 'C'};
float altitude_payload, altitude_container, temp_altitude, ref_altitude;
bool telemetry_on;
uint8_t limitSwitch;

// MPU control/status vars
volatile bool mpuInterrupt = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

TaskHandle_t bmpTaskHandle;
TaskHandle_t voltTaskHandle;
TaskHandle_t telemetryTaskHandle;
TaskHandle_t descentRateTaskHandle;
TaskHandle_t gpsTaskHandle;
TaskHandle_t memoryTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t stateTaskHandle;
TaskHandle_t errorCodeTaskHandle;
TaskHandle_t missionTimeTaskHandle;
TaskHandle_t cameraFilterTaskHandle;

void bmpTask(void *pvParameters);
void gpsTask(void *pvParameters);
void voltageTask(void *pvParameters);
void descentRateTask(void *pvParameters);
void imuTask(void *pvParameters);
void telemetryTask(void *pvParameters);
void memoryTask(void *pvParameters);
void stateTask(void *pvParameters);
void errorCodeTask(void *pvParameters);
void missionTimeTask(void *pvParameters);
void cameraFilterTask(void *pvParameters);

void filter_mechanism()
{
	filter_code = (filter_code_t)packet.lnln[0];
	switch (filter_code)
	{
		case FILTER_CODE_M:
			filterServo1.write(FILTER_RED_ANGLE);
			filterServo2.write(FILTER_RED_ANGLE);
			log_n("Filter set to M");
			break;
			
		case FILTER_CODE_F:
			filterServo1.write(FILTER_GREEN_ANGLE);
			filterServo2.write(FILTER_GREEN_ANGLE);
			log_n("Filter set to F");
			break;

		case FILTER_CODE_N:
			filterServo1.write(FILTER_BLUE_ANGLE);
			filterServo2.write(FILTER_BLUE_ANGLE);
			log_n("Filter set to N");
			break;
			
		case FILTER_CODE_R:
			filterServo1.write(FILTER_CLEAR_ANGLE);
			filterServo2.write(FILTER_RED_ANGLE);
			log_n("Filter set to R");
			break;
			
		case FILTER_CODE_G:
			filterServo1.write(FILTER_CLEAR_ANGLE);
			filterServo2.write(FILTER_GREEN_ANGLE);
			log_n("Filter set to G");
			break;

		case FILTER_CODE_B:
			filterServo1.write(FILTER_CLEAR_ANGLE);
			filterServo2.write(FILTER_BLUE_ANGLE);
			log_n("Filter set to B");
			break;

		case FILTER_CODE_P:
			filterServo1.write(FILTER_RED_ANGLE);
			filterServo2.write(FILTER_RED_ANGLE);
			log_n("Filter set to P");
			break;
			
		case FILTER_CODE_Y:
			filterServo1.write(FILTER_RED_ANGLE);
			filterServo2.write(FILTER_GREEN_ANGLE);
			log_n("Filter set to Y");
			break;
			
		case FILTER_CODE_C:
			filterServo1.write(FILTER_BLUE_ANGLE);
			filterServo2.write(FILTER_GREEN_ANGLE);
			log_n("Filter set to C");
			break;
			
		default:
			filterServo1.write(FILTER_CLEAR_ANGLE);
			filterServo2.write(FILTER_CLEAR_ANGLE);
			log_n("Filter set to default (no filter)");
			break;
	}
}
	
void dmpDataReady()
{
	mpuInterrupt = true;
}
	
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	if (status == ESP_NOW_SEND_SUCCESS)
		log_n("Data sent successfully");
	else
		log_n("Failed to send data");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *payload, int length)
{
	if (payload[0] == 0x7E)
	{
		if (calcCRC8(payload, length - 1, 213) == payload[length - 1])
		{
			switch(payload[2])
			{
				case CMD_TEAM_ID:
					memcpy(&packet.team_id, &payload[3], sizeof(uint32_t));
					preferences.putUInt("TEAM_ID", packet.team_id);
					log_n("Team ID written: %d", packet.team_id);
					break;
					
				case CMD_TELEM_ON:
					if(eTaskGetState(telemetryTaskHandle) != eRunning)
					{
						vTaskResume(telemetryTaskHandle);
					}
					preferences.putBool("TELEMETRY_ON", true);
					log_n("Telemetry On");
					break;
		
				case CMD_TELEM_OFF:
					if(eTaskGetState(telemetryTaskHandle) != eSuspended)
					{
						vTaskSuspend(telemetryTaskHandle);
					}
					preferences.putBool("TELEMETRY_ON", false);
					log_n("Telemetry Off");
					break;
					
				case CMD_RELEASE:
					// TODO: Add release mechanism
					log_n("Release");
					break;
		
				case CMD_CAL:
					packet.packet_number = 0;
					packet.state = READY_TO_FLIGHT;
					preferences.putUInt("STATE", packet.state);
					preferences.putUInt("PACKET_NUMBER", packet.packet_number);
					preferences.putFloat("REF_ALTITUDE", altitude_payload);
					log_n("Calibration");
					break;
					
				case CMD_FILTER:
					memcpy(&packet.lnln, &payload[3], sizeof(packet.lnln));
					log_n("Filter Command: %c %d %c %d", packet.lnln[0], packet.lnln[1], packet.lnln[2], packet.lnln[3]);
					vTaskResume(cameraFilterTaskHandle);
					break;

				case CMD_MISSION_TIME:
					memcpy(&packet.mission_time, &payload[3], sizeof(packet.mission_time));
					log_n("Mission Time: %d", packet.mission_time);
					// TODO: Add setting mission time in RTC
					break;
					
				case CONTAINER_DATA:
					memcpy(&packet.pressure_container, &payload[3], sizeof(packet.pressure_container));
					limitSwitch = payload[length - 2];
					log_n("Container Data: Pressure: %d", packet.pressure_container);
					break;
					
				case IOT1_STATION_DATA:
					memcpy(&packet.iot1, &payload[3], sizeof(packet.iot1));
					log_n("IoT1 Station Data: Temperature: %d", packet.iot1);
					break;
		
				case IOT2_STATION_DATA:
					memcpy(&packet.iot2, &payload[3], sizeof(packet.iot2));
					log_n("IoT2 Station Data: Temperature: %d", packet.iot2);
					break;
		
				default:
					break;
			}
		}
	}
}

void setup()
{
	Serial.begin(115200);

	preferences.begin("PAYLOAD");

    packet.packet_number = preferences.getUInt("PACKET_NUMBER", 0);
    packet.team_id = preferences.getUInt("TEAM_ID", 0);
    telemetry_on = preferences.getBool("TELEMETRY_ON", false);
	state = (state_t)preferences.getUInt("STATE", READY_TO_FLIGHT);
	ref_altitude = preferences.getFloat("REF_ALTITUDE", 0);

	if (!bmp.begin(0x76))
	{
		log_n("BMP280 not found!");
		// return;
	}
	
	if (!mpu.testConnection())
	{
		log_n("MPU6050 not connected!");
		// return;
	}
	
	// mpu.initialize();
	// devStatus = mpu.dmpInitialize();

	// mpu.setXGyroOffset(51);
	// mpu.setYGyroOffset(8);
	// mpu.setZGyroOffset(21);
	// mpu.setXAccelOffset(1150);
	// mpu.setYAccelOffset(-50);
	// mpu.setZAccelOffset(1060);

	// if (devStatus == 0)
	// {
	// 	mpu.CalibrateAccel(6);
	// 	mpu.CalibrateGyro(6);
	// 	mpu.PrintActiveOffsets();
	// 	mpu.setDMPEnabled(true);
	// 	attachInterrupt(MPU_INT, dmpDataReady, RISING);
	// 	mpuIntStatus = mpu.getIntStatus();
	// 	dmpReady = true;
	// 	packetSize = mpu.dmpGetFIFOPacketSize();
	// }

	gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

	if(!SD.begin())
	{
		log_n("Card Mount Failed");
		// return;
	}

	const char *path = "/log.txt";
	File file;
	if (SD.exists(path))
        file = SD.open(path, FILE_APPEND);
    else
        file = SD.open(path, FILE_WRITE);

	if(!file)
	{
		log_n("Failed to create/open file");
		// return;
	}
	file.close();

	URTCLIB_WIRE.begin();

	// releaseServo.attach(4);
	filterServo1.attach(FILTER_DISK1);
	filterServo2.attach(FILTER_DISK2);

	WiFi.mode(WIFI_STA);
	WiFi.setTxPower(WIFI_POWER_19_5dBm);
    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

	if(esp_now_init() != ESP_OK)
	{
        log_n("Error initializing ESP-NOW");
        // return;
    }
	
	esp_now_peer_info_t peerInfo = {};
	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;  
	peerInfo.encrypt = false;
	
	if(esp_now_add_peer(&peerInfo) != ESP_OK)
	{
		log_n("Failed to add peer");
        // return;
    }
	
	esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

	xTaskCreate(bmpTask, "BMP280 Task", 2048, NULL, 1, &bmpTaskHandle);
	xTaskCreate(voltageTask, "Voltage Task", 2048, NULL, 1, &voltTaskHandle);
	xTaskCreate(descentRateTask, "Descent Rate Task", 2048, NULL, 1, &descentRateTaskHandle);
	xTaskCreate(telemetryTask, "Telemetry Task", 8192, NULL, 1, &telemetryTaskHandle);
	xTaskCreate(gpsTask, "GPS Task", 4096, NULL, 1, &gpsTaskHandle);
	xTaskCreate(memoryTask, "Memory Task", 8192, NULL, 1, &memoryTaskHandle);
	// xTaskCreate(imuTask, "IMU Task", 8192, NULL, 1, &imuTaskHandle);
	xTaskCreate(stateTask, "State Task", 2048, NULL, 1, &stateTaskHandle);
	xTaskCreate(errorCodeTask, "Error Code Task", 2048, NULL, 1, &errorCodeTaskHandle);
	xTaskCreate(missionTimeTask, "Mission Time Task", 2048, NULL, 1, &missionTimeTaskHandle);
	xTaskCreate(cameraFilterTask, "Camera Filter Task", 2048, NULL, 1, &cameraFilterTaskHandle);
}

void loop()
{
}

void bmpTask(void *pvParameters)
{
	while (1)
	{
		packet.temperature = (int16_t)(bmp.readTemperature() * 100);
		packet.pressure_payload = (uint32_t)(bmp.readPressure() * 100);
  		altitude_payload = 44330 * (1.0 - pow((packet.pressure_payload / 100.0 / 100,0) / 1013.25, 0.190295)) - ref_altitude;
  		altitude_container = 44330 * (1.0 - pow((packet.pressure_container / 100.0 / 100.0) / 1013.25, 0.190295)) - ref_altitude;

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

void gpsTask(void *pvParameters)
{
	while (1)
	{
		while (gpsSerial.available() > 0)
		{
			gps.encode(gpsSerial.read());
			if (gps.location.isUpdated())
			{
				packet.gps_latitude = gps.location.lat();
				packet.gps_longitude = gps.location.lng();
				packet.gps_altitude = (int16_t)(gps.altitude.meters() * 10);
			}
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void voltageTask(void *pvParameters)
{
	#define FILTER_LENGTH	5

	float window[FILTER_LENGTH] = {0};
	uint8_t ind_ = 0;
	float sum = 0.0;
	uint8_t counter_change = 0;
	float v = 0.0;
	float v_prev = 0.0;

	while (1)
	{
		sum = 0;
		v = analogRead(36) * (2.8 / 3474.5) * 3.0; // r1 = 20k, r2 = 10k, Vmax = 2.8V, Batt = 8.4V

		if (fabs(v - v_prev) >= 0.0005)
		{
			counter_change++;
			if (counter_change >= 5)
			{
				counter_change = 0;
				window[ind_] = v;
				for (int i = 0; i < FILTER_LENGTH; ++i)
				{
					sum += window[ind_];
				}
				ind_ = (ind_ + 1) % FILTER_LENGTH;
				float value = (int)((sum / FILTER_LENGTH) * 100 + 0.5);
				v_prev = (float)value / 100.0;
			}
		}
		packet.volt = (uint8_t)(v_prev * 10.0);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void descentRateTask(void *pvParameters)
{
	while (1)
	{
		// Calculate descent rate
		if (altitude_payload < temp_altitude)
		{
			packet.descent_rate = (int16_t) ((temp_altitude - altitude_payload) / 0.5) * 100;
		}
		else
		{
			packet.descent_rate = 0;
		}

		temp_altitude = altitude_payload;
		
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void imuTask(void *pvParameters)
{
	// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	while (1)
	{
		if (!dmpReady) return;

  		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			packet.yaw = (int16_t)(ypr[0] * 180 / M_PI * 100);
			packet.pitch = (int16_t)(ypr[1] * 180 / M_PI * 100);
			packet.roll = (int16_t)(ypr[2] * 180 / M_PI * 100);
		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

void telemetryTask(void *pvParameters)
{
	if (telemetry_on == false)
	{
		vTaskSuspend(NULL);
	}
	
	while (1)
	{
		packet.packet_number++;
		preferences.putUInt("PACKET_NUMBER", packet.packet_number);

		uint8_t payload_buffer[sizeof(packet_t) + sizeof(message_type_t) + 3];
		payload_buffer[0] = 0x7E; // Start byte
		payload_buffer[1] = sizeof(payload_buffer); // Length
		payload_buffer[2] = PAYLOAD_TELEMETRY_DATA; // Message type
		memcpy(&payload_buffer[3], &packet, sizeof(packet_t));
		payload_buffer[sizeof(payload_buffer) - 1] = calcCRC8(payload_buffer, sizeof(payload_buffer) - 1, 213); // CRC8

		esp_now_send(broadcastAddress, payload_buffer, sizeof(payload_buffer));

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void memoryTask(void *pvParameters)
{
	File file;
	const char *path = "/log.txt";
	char packet_str[256];
	
	while (1)
	{
		sprintf(packet_str, "PN:%d,S:%d,E:%d,MT:%d,P1:%.2f,P2:%.2f,A1:%.2f,A2:%.2f,AD:%.2f,DR:%.2f,T:%.2f,V:%.1f,LAT:%.4f,LON:%.4f,GA:%.4f,P:%.2f,R:%.2f,Y:%.2f,IOT1:%.2f,IOT2:%.2f,ID:%d\r\n",
            packet.packet_number,
			packet.state,
			packet.error_code,
			packet.mission_time,
            (float)packet.pressure_payload / 100.0,
            (float)packet.pressure_container / 100.0,
			altitude_payload,
			altitude_container,
			altitude_payload - altitude_container,
            (float)packet.descent_rate / 100.0,
            (float)packet.temperature / 100.0,
            (float)packet.volt / 10.0,
			packet.gps_latitude,
			packet.gps_longitude,
			packet.gps_altitude / 10.0,
			(float)packet.pitch / 100.0,
			(float)packet.roll / 100.0,
			(float)packet.yaw / 100.0,
            (float)packet.iot1 / 100.0,
            (float)packet.iot2 / 100.0,
            packet.team_id
		);

		log_n("Logging data: %s", packet_str);

		file = SD.open(path, FILE_APPEND);
		if(!file.print(packet_str))
		{
			log_n("Failed to write file");
			return;
		}
		file.close();
		log_n("Data logged to SD card");

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void stateTask(void *pvParameters)
{
	uint8_t invalid_descent = 0;
	uint8_t validation_counter = 0;

	while (1)
	{
		if (altitude_payload - temp_altitude > 10)
		{
			invalid_descent = 1;
		}
		
		switch (state)
		{
			case READY_TO_FLIGHT:
				if (altitude_payload > 50)
				{
					state = ASCENT;
					preferences.putUInt("STATE", state);
					log_n("State changed to ASCENT");
				}
				break;
				
			case ASCENT:
				if (altitude_payload < temp_altitude && !invalid_descent)
				{
					validation_counter++;
					if (validation_counter >= 5)
					{
						validation_counter = 0;
						state = SATELLITE_DESCENT;
						preferences.putUInt("STATE", state);
						log_n("State changed to SATELLITE_DESCENT");
					}
				}
				break;

			case SATELLITE_DESCENT:
				if (altitude_payload <= 450 && limitSwitch && !invalid_descent)
				{
					// TODO: Add servo mechanism to release payload
					// releaseServo.write(90);
					if (altitude_payload <= 410)
					{
						state = RELEASE;
						preferences.putUInt("STATE", state);
						log_n("State changed to RELEASE");
					}
				}
				break;
				
			case RELEASE:
				if (altitude_payload <= 390 && !invalid_descent)
				{
					state = PAYLOAD_DESCENT;
					preferences.putUInt("STATE", state);
					log_n("State changed to PAYLOAD_DESCENT");
				}
				break;
				
			case PAYLOAD_DESCENT:
				if (altitude_payload < 10 && !invalid_descent)
				{
					state = RECOVERY;
					preferences.putUInt("STATE", state);
					log_n("State changed to RECOVERY");
				}
				break;

			case RECOVERY:
				break;
			
			default:
				break;
		}

		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
}

void errorCodeTask(void *pvParameters)
{
	while (1)
	{

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void missionTimeTask(void *pvParameters)
{
	struct tm t;
	
	while (1)
	{
		rtc.refresh();

        t.tm_year = rtc.year() + 2000 - 1900; // rtc.year() returns e.g. 24 for 2024
        t.tm_mon  = rtc.month(); - 1;          // tm_mon is 0-11
        t.tm_mday = rtc.day();
        t.tm_hour = rtc.hour();
        t.tm_min  = rtc.minute();
        t.tm_sec  = rtc.second();
        t.tm_isdst = 0;

		time_t epoch = mktime(&t);
		packet.mission_time = (uint32_t)epoch;

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void cameraFilterTask(void *pvParameters)
{
	while (1)
	{
		filter_mechanism();
		vTaskDelay(packet.lnln[1]  / portTICK_PERIOD_MS);
		filter_mechanism();
		vTaskDelay(packet.lnln[3]  / portTICK_PERIOD_MS);

		filterServo1.write(FILTER_CLEAR_ANGLE);
		filterServo2.write(FILTER_CLEAR_ANGLE);

		vTaskSuspend(NULL);
	}
}
