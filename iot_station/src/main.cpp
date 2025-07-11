#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <CRC.h>

#define BMP280_SAMPLE_TIME_MS		100
#define TELEMETRY_SAMPLE_TIME_MS 	200

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

Adafruit_BMP280 bmp;
int16_t temperature = 0;

uint8_t broadcastAddress[] = {0xC8, 0x2E, 0x18, 0x8D, 0x74, 0xB8};

TaskHandle_t bmpTaskHandle;
TaskHandle_t telemetryTaskHandle;

void bmpTask(void *parameter);
void telemetryTask(void *parameter);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
	Serial.print("Status pengiriman: ");
	Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Berhasil" : "Gagal");
}

void setup()
{
	Serial.begin(115200);

	if (!bmp.begin(0x76))
	{
		log_n("BMP280 not found!");
		return;
	}

	WiFi.mode(WIFI_STA);
	WiFi.setTxPower(WIFI_POWER_19_5dBm);
	esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

	if (esp_now_init() != ESP_OK)
	{
		log_n("ESP-NOW init failed!");
		return;
	}

	esp_now_peer_info_t peerInfo = {};
	memcpy(peerInfo.peer_addr, broadcastAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;
	
	if (esp_now_add_peer(&peerInfo) != ESP_OK)
	{
		log_n("Failed to add peer!");
		return;
	}

	esp_now_register_send_cb(OnDataSent);
	
	xTaskCreate(bmpTask, "BMP280 Task", 2048, NULL, 1, &bmpTaskHandle);
	xTaskCreate(telemetryTask, "Telemetry Task", 2048, NULL, 1, &telemetryTaskHandle);
}

void loop()
{
}

void bmpTask(void *parameter)
{
	while (1)
	{
		temperature = (int16_t)(bmp.readTemperature() * 100);

		vTaskDelay(BMP280_SAMPLE_TIME_MS / portTICK_PERIOD_MS);
	}
}

void telemetryTask(void *parameter)
{
	while (1)
	{
		uint8_t iot_station_buffer[sizeof(message_type_t) + sizeof(int16_t) + 3];
		iot_station_buffer[0] = 0x7E; // Start byte
		iot_station_buffer[1] = sizeof(iot_station_buffer); // Length
		iot_station_buffer[2] = IOT1_STATION_DATA; // Message type
		memcpy(&iot_station_buffer[3], &temperature, sizeof(int16_t));
		iot_station_buffer[sizeof(iot_station_buffer) - 1] = calcCRC8(iot_station_buffer, sizeof(iot_station_buffer) - 1, 213); // CRC8

		esp_now_send(broadcastAddress, (uint8_t *) &iot_station_buffer, sizeof(iot_station_buffer));
		log_n("Temperature: %d", temperature);

		vTaskDelay(TELEMETRY_SAMPLE_TIME_MS / portTICK_PERIOD_MS);
	}
}