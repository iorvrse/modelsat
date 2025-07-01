#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

#define RXD2 16
#define TXD2 17

#define GPS_BAUD 9600

// // The TinyGPS++ object
TinyGPSPlus gps;

// // Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);

// typedef struct
// {
//   uint8_t year;        // 0-99
//   uint8_t month;       // 1-12
//   uint8_t day;         // 1-31
//   uint8_t hour;        // 0-23
//   uint8_t minute;      // 0-59
//   uint8_t second;      // 0-59
// } mission_time_t;

typedef struct
{
	uint16_t packet_number;
//   uint8_t state;
//   char error_code[6];
	// mission_time_t mission_time;
	float pressure1;
	float pressure2;
	float altitude1;
	float altitude2;
	float altitude_difference;
	float descent_rate;
	float temperature;
	float volt;
	float gps_latitude;
	float gps_longitude;
	float gps_altitude;
	float pitch;
	float roll;
	float yaw;
//   char lnln[4];
	float iot1;
	float iot2;
	char team_id[7];
} packet_t;

typedef struct
{
	uint8_t id;
	float temperature;
} iot_station_msg_t;

typedef struct
{
	uint8_t id;
	float pressure;
	float altitude;
} container_msg_t;

packet_t packet = {
	.team_id = "632419"
};
char packet_str[256];

// MAC address for GCS
uint8_t broadcastAddress[] = {0xC0, 0x49, 0xEF, 0xF9, 0x97, 0xCA};

Adafruit_BMP280 bmp;

esp_now_peer_info_t peerInfo;

void initFile(fs::FS &fs, const char * path)
{
	Serial.printf("Writing file: %s\n", path);

	File file;
	if (fs.exists(path))
        file = fs.open(path, FILE_APPEND);
    else
        file = fs.open(path, FILE_WRITE);

	if(!file)
	{
		Serial.println("Failed to initiate file");
		return;
	}

	file.close();
}

void updateFile(fs::FS &fs, const char * path, const char * message)
{
	File file;
	file = fs.open(path, FILE_APPEND);

	if(!file.print(message))
	{
		Serial.println("Failed to write file");
		return;
	}

	file.close();
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
	if (len == sizeof(iot_station_msg_t))
	{
		iot_station_msg_t iot_station_data;
		memcpy(&iot_station_data, incomingData, sizeof(iot_station_msg_t));
		if (iot_station_data.id == 1)
		{
			packet.iot1 = iot_station_data.temperature;
		}
		else if (iot_station_data.id == 2)
		{
			packet.iot2 = iot_station_data.temperature;
		}
	}
	else if (len == sizeof(container_msg_t))
	{
		container_msg_t container_data;
		memcpy(&container_data, incomingData, sizeof(container_data));
		packet.pressure2 = container_data.pressure;
		packet.altitude2 = container_data.altitude;
		packet.altitude_difference = packet.altitude1 - packet.altitude2;
	}
	else
	{
		Serial.println("Data tidak dikenali");
	}
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	Serial.print("\r\nLast Packet Send Status:\t");
	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

TaskHandle_t bmpTaskHandle;
TaskHandle_t voltTaskHandle;
TaskHandle_t telemetryTaskHandle;
TaskHandle_t descentRateTaskHandle;
TaskHandle_t gpsTaskHandle;
TaskHandle_t memoryTaskHandle;
TaskHandle_t imuTaskHandle;
// TaskHandle_t controlTaskHandle;
// TaskHandle_t errorCodeTaskHandle;

void bmpTask(void *pvParameters)
{
	while (1)
	{
		// Read temperature and pressure from BMP280
		packet.temperature = bmp.readTemperature();
		packet.pressure1 = bmp.readPressure() / 100.0F; // Convert to hPa
		packet.altitude1 = bmp.readAltitude(1013.25); // Assuming sea level pressure is 1013.25 hPa

		vTaskDelay(200 / portTICK_PERIOD_MS); // Delay for 2 seconds
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
				packet.gps_altitude = gps.altitude.meters();
				// packet.mission_time.year = gps.date.year();
				// packet.mission_time.month = gps.date.month();
				// packet.mission_time.day = gps.date.day();
				// packet.mission_time.hour = gps.time.hour();
				// packet.mission_time.minute = gps.time.minute();
				// packet.mission_time.second = gps.time.second();
			}
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 100 ms
	}
}

void voltageTask(void *pvParameters)
{
	const int FILTER_LENGTH = 5; // Length of the filter window
	float window[FILTER_LENGTH] = {0}; // Circular buffer for voltage readings
	int ind_ = 0; // Index for the circular buffer
	float sum = 0.0; // Sum of the voltage readings in the window
	int counter_change = 0; // Counter for changes in voltage
	float v = 0.0; // Current voltage reading

	while (1)
	{
		sum = 0;
		v = analogRead(36) * (2.8 / 3474.5) * 3.0; // r1 = 20k, r2 = 10k, Vmax = 2.8V, Batt = 8.4V

		if (fabs(v - packet.volt) >= 0.0005)
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
				packet.volt = (float)value / 100.0;
			}
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

void descentRateTask(void *pvParameters)
{
	float temp_altitude = 0;
	while (1)
	{
		// Calculate descent rate (replace with actual logic)
		if (packet.altitude1 < temp_altitude)
		{
			packet.descent_rate = (temp_altitude - packet.altitude1) / 0.5; // Descent rate in meters per second
		}
		else
		{
			packet.descent_rate = 0.0; // No descent
		}
		temp_altitude = packet.altitude1; // Update altitude for next calculation
		
    	vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 1 second
	}
}

void imuTask(void *pvParameters)
{
	while (1)
	{
		while (fifoCount < packetSize)
		{
			fifoCount = mpu.getFIFOCount();
		}

		if (fifoCount == 1024)
		{
		
			mpu.resetFIFO();
			Serial.println(F("FIFO overflow!"));	
		}
		else
		{
			if (fifoCount % packetSize != 0)
			{
				mpu.resetFIFO();
			}
			else
			{
				while (fifoCount >= packetSize)
				{
					mpu.getFIFOBytes(fifoBuffer,packetSize);
					fifoCount -= packetSize;				
				}    
				
				mpu.dmpGetQuaternion(&q,fifoBuffer);
				mpu.dmpGetGravity(&gravity,&q);
				mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
				packet.pitch = ypr[1] * 180 / M_PI; // Convert to degrees
				packet.roll = ypr[2] * 180 / M_PI; // Convert to degrees
				packet.yaw = ypr[0] * 180 / M_PI; // Convert to degrees
			}
		}
		vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100 ms
	}
}

// void errorCodeTask(void *pvParameters)
// {
//   while (1)
//   {
//     if (packet.state <= 2 && packet.descent_rate <= 14)
//         packet.error_code[0] = '0';
//     else
//         packet.error_code[0] = '1';
		
//     if (packet.state >= 3 && packet.descent_rate <= 8)
//         packet.error_code[1] = '0';
//     else
//         packet.error_code[1] = '1';
				
//     if (packet.state >= 3 && packet.descent_rate <= 8)
//         packet.error_code[2] = '0';
//     else
//         packet.error_code[2] = '1';
//   }
// }

void telemetryTask(void *pvParameters)
{
	while (1)
	{
		packet.packet_number++;
//     // Send telemetry data
		esp_now_send(broadcastAddress, (uint8_t *)&packet, sizeof(packet));
		sprintf(packet_str, "pn:%d, P1:%.2f, P2:%.2f, alt1:%.2f, alt2:%.2f, altDif:%.2f, desrt: %.2f, T:%.2f, V:%.1f, lat:%.4f, lon:%.4f, galt:%.4f, pc:%.2f, rl:%.2f, yw:%.2f, iot1:%.2f, iot2:%.2f, t:%s\r\n",
            packet.packet_number,
            packet.pressure1,
            packet.pressure2,
            packet.altitude1,
            packet.altitude2,
            packet.altitude_difference,
            packet.descent_rate,
            packet.temperature,
            packet.volt,
			packet.gps_latitude,
			packet.gps_longitude,
			packet.gps_altitude,
			packet.pitch,
			packet.roll,
			packet.yaw,
            packet.iot1,
            packet.iot2,
            packet.team_id
        );
		Serial.print(packet_str);

    	vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
	}
}

void memoryTask(void *pvParameters)
{
	while (1)
	{
		updateFile(SD, "/log.txt", packet_str);
		Serial.println("Data logged to SD card");

		vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
	}
}

void setup()
{
	Serial.begin(115200);
	WiFi.mode(WIFI_STA);

	if (!bmp.begin(0x76))
	{
		Serial.println("BMP280 tidak ditemukan!");
		while (1);
	}

	gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

	if (!mpu.testConnection()) {
		Serial.println("MPU6050 tidak terhubung!");
		while (1);
	}
	
	mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-1343);
    mpu.setYAccelOffset(-1155);
    mpu.setZAccelOffset(1033);
    mpu.setXGyroOffset(19);
    mpu.setYGyroOffset(-27);
    mpu.setZGyroOffset(16);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

	if(!SD.begin())
	{
		Serial.println("Card Mount Failed");
		return;
	}
	
	initFile(SD, "/log.txt");

	if (esp_now_init() != ESP_OK) {
		Serial.println("ESP-NOW Init gagal");
		return;
	}

	esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
	
	// Register peer
	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;  
	peerInfo.encrypt = false;
	
	// Add peer        
	if (esp_now_add_peer(&peerInfo) != ESP_OK){
		Serial.println("Failed to add peer");
		return;
	}

	xTaskCreate(bmpTask, "BMP280 Task", 2048, NULL, 1, &bmpTaskHandle);
	xTaskCreate(voltageTask, "Voltage Task", 2048, NULL, 1, &voltTaskHandle);
	xTaskCreate(descentRateTask, "Descent Rate Task", 2048, NULL, 1, &descentRateTaskHandle);
	xTaskCreate(telemetryTask, "Telemetry Task", 8192, NULL, 1, &telemetryTaskHandle);
	xTaskCreate(gpsTask, "GPS Task", 4096, NULL, 1, &gpsTaskHandle);
	xTaskCreate(memoryTask, "Memory Task", 8192, NULL, 1, &memoryTaskHandle);
	xTaskCreate(imuTask, "IMU Task", 4096, NULL, 1, &imuTaskHandle);
	// xTaskCreate(errorCodeTask, "Error Code Task", 2048, NULL, 1, NULL);
}

void loop()
{
}
