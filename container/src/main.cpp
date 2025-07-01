#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <esp_now.h>
#include <WiFi.h>

#define READ_SENSOR_SAMPLING_MS 100
#define SEND_DATA_SAMPLING_MS   200

typedef struct
{
  uint8_t id;
  float pressure;
  float altitude;
} container_msg_t;

container_msg_t dataToSend;

Adafruit_BMP280 bmp;

uint8_t broadcastAddress[] = {0xC8, 0x2E, 0x18, 0x8D, 0x74, 0xB8};

TaskHandle_t taskReadSensorHandler;
TaskHandle_t taskSendDataHandler;

void TaskReadSensor(void *pvParameters)
{
  while (1)
  {
    dataToSend.pressure = bmp.readPressure() / 100.0F; // dalam hPa
    dataToSend.altitude = bmp.readAltitude(1013.25);   // asumsikan tekanan permukaan laut 1013.25 hPa
    vTaskDelay(READ_SENSOR_SAMPLING_MS / portTICK_PERIOD_MS);
  }
}

void TaskSendData(void *pvParameters)
{
  while (1)
  {
    dataToSend.id = 3;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));
    
    if (result == ESP_OK)
    {
      Serial.println("Sent with success");
    }
    else
    {
      Serial.println("Error sending the data");
    }
    
    Serial.print("Dikirim: ");
    Serial.print(dataToSend.id);
    Serial.print(" | Tekanan: ");
    Serial.print(dataToSend.pressure);
    Serial.print(" | Ketinggian: ");
    Serial.println(dataToSend.altitude);
    
    vTaskDelay(SEND_DATA_SAMPLING_MS / portTICK_PERIOD_MS);
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("Status pengiriman: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Berhasil" : "Gagal");
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

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init gagal");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Gagal menambahkan peer");
    return;
  }

  xTaskCreate(TaskReadSensor, "TaskReadSensor", 2048, NULL, 1, &taskReadSensorHandler);
  xTaskCreate(TaskSendData, "TaskSendData", 2048, NULL, 1, &taskSendDataHandler);
}

void loop()
{
  // dataToSend.id = 3;
  // dataToSend.pressure = bmp.readPressure() / 100.0F; // dalam hPa
  // dataToSend.altitude = bmp.readAltitude(1013.25);   // asumsikan tekanan permukaan laut 1013.25 hPa

  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &dataToSend, sizeof(dataToSend));
  
  // if (result == ESP_OK)
  // {
  //   Serial.println("Sent with success");
  // }
  // else
  // {
  //   Serial.println("Error sending the data");
  // }
  
  // Serial.print("Dikirim: ");
  // Serial.print(dataToSend.id);
  // Serial.print(" | Tekanan: ");
  // Serial.print(dataToSend.pressure);
  // Serial.print(" | Ketinggian: ");
  // Serial.println(dataToSend.altitude);    
}
