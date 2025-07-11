#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <CRC.h>
#include <MicroNMEA.h>

uint8_t fcpaMacAddress[6] = {0xC8, 0x2E, 0x18, 0x8D, 0x74, 0xB8};
esp_now_peer_info_t peerInfo;

// 802.11 Packet Structure
typedef struct
{
	unsigned frame_ctrl: 16;
	unsigned duration_id: 16;
	uint8_t addr1[6]; /* receiver address */
	uint8_t addr2[6]; /* sender address */
	uint8_t addr3[6]; /* filtering address */
	unsigned sequence_ctrl: 16;
	uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
	wifi_ieee80211_mac_hdr_t hdr;
	uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

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

// Telemetry Data
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

typedef struct
{
	float gcsLat;
	float gcsLon;
	int16_t gcsAlt;
	uint8_t satNum;
} GPSTelemetryData;

packet_t payloadTelemetryData;
GPSTelemetryData gpsTelemetryData;

// GCS Hardware Specific
int8_t rssi;

// GPS Data
uint8_t nmeaBuffer[128];

TaskHandle_t RSSITask_Handle;
TaskHandle_t GPSTask_Handle;

MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void SendUARTData(message_type_t type, const uint8_t* payload, int length) {
	// Build Communication Frame
	uint8_t frame[length + 4];
	if (payload == NULL)
	{
		frame[0] = 0x7E;
		frame[1] = 0;
		frame[2] = type;
	}
	else
	{
		frame[0] = 0x7E;
		frame[1] = length;
		frame[2] = type;
		for (uint i = 0; i < length; i++)
		{
			frame[i + 3] = payload[i];
		}
	}
	frame[sizeof(frame) - 1] = calcCRC8(frame, sizeof(frame) - 1, 213);

	// Send Command
	Serial.write(frame, sizeof(frame));
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status)
{
}

void onDataRecv(const uint8_t* mac, const uint8_t* payload, int length)
{
	if (payload[0] == 0x7E)
	{
		if (calcCRC8(payload, length - 1, 213) == payload[length - 1])
		{
			switch(payload[2])
			{
				case PAYLOAD_TELEMETRY_DATA:
					memcpy(&payloadTelemetryData, &payload[3], sizeof(packet_t));
					SendUARTData(PAYLOAD_TELEMETRY_DATA, (uint8_t*)&payloadTelemetryData, sizeof(payloadTelemetryData));
					break;
				default:
					break;
			}
		}
	}
}

void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
	// All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
	if (type != WIFI_PKT_MGMT)
	{
		return;
	}

	const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
	const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
	const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

	if(((hdr->frame_ctrl & 0xFF) == 0xD0) && memcmp(hdr->addr2, fcpaMacAddress, 6) == 0)
	{
		rssi = ppkt->rx_ctrl.rssi;
	}
}

void GPSTask(void *pvParameters)
{
	for(;;)
	{
		if(Serial2.available())
		{
			byte data = Serial2.read();
			if(nmea.process(data))
			{
				long alt;

				gpsTelemetryData.gcsLat = nmea.getLatitude() / 1000000.f;
				gpsTelemetryData.gcsLon = nmea.getLongitude() / 1000000.f;

				if(nmea.getAltitude(alt))
				{
					gpsTelemetryData.gcsAlt = alt / 10.f;
				}

				gpsTelemetryData.satNum = nmea.getNumSatellites();

				SendUARTData(GCS_LOCATION_DATA, (uint8_t*)&gpsTelemetryData, sizeof(gpsTelemetryData));
			}
		}
	}
}

void setup()
{
	Serial.begin(115200);
	Serial2.begin(115200);

	xTaskCreatePinnedToCore(GPSTask, "GPS Task", 4096, NULL, 1, &GPSTask_Handle, 1);
	
	WiFi.mode(WIFI_STA);
	WiFi.setTxPower(WIFI_POWER_19_5dBm);
	esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);

	esp_wifi_set_promiscuous(true);
	esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);

	if(esp_now_init() != ESP_OK)
	{
		log_n("Error initializing ESP-NOW");
		return;
	}

	esp_now_register_send_cb(onDataSent);

	memcpy(peerInfo.peer_addr, fcpaMacAddress, 6);
	peerInfo.channel = 0;
	peerInfo.encrypt = false;

	if(esp_now_add_peer(&peerInfo) != ESP_OK)
	{
		log_n("Failed to add peer");
		return;
	}

	esp_now_register_recv_cb(onDataRecv);

	log_n("ESP-NOW Started with this parameter:");
	log_n("Protocol: Wi-Fi Long Range");
	log_n("TX Power: 19,5 dBm");
}

void loop()
{
	if(Serial.available())
	{
		if(Serial.read() == 0x7E)
		{
			uint8_t payloadLength = Serial.read();
			uint8_t payload[payloadLength + 2];
			
			Serial.readBytes(payload, payloadLength + 2);
			
			uint8_t wholeData[sizeof(payload) + 2];
			wholeData[0] = 0x7E;
			wholeData[1] = payloadLength;
			memcpy(&wholeData[2], payload, payloadLength + 2);

			if(payload[payloadLength + 1] == calcCRC8(wholeData, sizeof(wholeData) - 1, 213))
			{
				esp_now_send(fcpaMacAddress, payload, sizeof(payload) - 1);
			}
		}
	}
}