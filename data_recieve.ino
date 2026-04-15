#include <SPI.h>
#include <LoRa.h>
#include <BluetoothSerial.h>

// Pins for the Receiver (matches your first snippet)
#define CS    4
#define RST   15
#define DIO0  2

BluetoothSerial SerialBT;

// Internal 3D structures for integers
struct int16_t3d {
  int16_t x, y, z;
};

// UPDATED: Matches the Transmitter's packed structure and data types
struct __attribute__((packed)) SensorData {
  unsigned long now;          
  unsigned long UT_seconds;   
  int16_t AHT_temp, AHT_hum;  
  int16_t BMP_temp, BMP_pres; 
  int16_t3d gyro;             
  int16_t3d accel;            
  int16_t gtemp;              
  int16_t3d mag;              
  int16_t volt;               
  uint16_t pm1_0, pm2_5, pm10_0; 
  uint16_t p03um, p05um, p10um;  
  float lat, lon;             
  uint16_t altitude;          
  int q2G;                    
  uint8_t flags;              
};

const char* labels[] = {
  "now[ms]", "UT[s]",
  "AHT_temp[C]", "AHT_hum",
  "BMP_temp[C]", "BMP_pres",
  "gx", "gy", "gz",
  "ax[m/s2]", "ay[m/s2]", "az[m/s2]",
  "gtemp[C]",
  "magx[uT]", "magy[uT]", "magz[uT]",
  "voltage",
  "pm1_0", "pm2_5", "pm10_0",
  "p03um", "p05um", "p10um",
  "lat", "lon", "altitude", "2Grssi", "flags", "rssi", "snr", "malformed"
};

int expected_payload_size;
SensorData data;
char row[2048];

void setup() {
  Serial.begin(115200);
  SerialBT.begin("AuroraData", false); 

  LoRa.setPins(CS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // Size = StartByte(1) + Struct + EndByte(1)
  expected_payload_size = sizeof(SensorData) + 2; 
  
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);

  Serial.println("LoRa Receiver Ready. Matching updated transmitter struct...");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    std::vector<uint8_t> payload;
    while (LoRa.available()) {
      payload.push_back(LoRa.read());
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    // Check size and Framing bytes (0xAA ... 0xBB)
    if (payload.size() != (size_t)expected_payload_size || payload[0] != 0xAA || payload[payload.size()-1] != 0xBB) {
      generateErrorRow(row, sizeof(row), payload, expected_payload_size, rssi, snr);
    } else {
      memcpy(&data, &payload[1], sizeof(SensorData));
      dataToJson(data, row, sizeof(row), snr, rssi, false);
    }

    SerialBT.println(row);
    Serial.println(row);
  }
}

void dataToJson(SensorData d, char buffer[], int len, float snr, int rssi, bool err) {
  int i = 0;
  snprintf(buffer, len,
    "{\"%s\":%lu,\"%s\":%lu,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%ld,"
    "\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,"
    "\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%u,"
    "\"%s\":%u,\"%s\":%u,\"%s\":%u,\"%s\":%u,\"%s\":%u,\"%s\":%.9f,\"%s\":%.9f,"
    "\"%s\":%u,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%.2f,\"%s\":%d}",
    labels[i++], d.now, labels[i++], d.UT_seconds,
    labels[i++], (float)d.AHT_temp/100.0, labels[i++], (float)d.AHT_hum/100.0,
    labels[i++], (float)d.BMP_temp/100.0, labels[i++], (long)d.BMP_pres*10,
    labels[i++], (float)d.gyro.x/100.0, labels[i++], (float)d.gyro.y/100.0, labels[i++], (float)d.gyro.z/100.0,
    labels[i++], (float)d.accel.x/1000.0, labels[i++], (float)d.accel.y/1000.0, labels[i++], (float)d.accel.z/1000.0,
    labels[i++], (float)d.gtemp/100.0,
    labels[i++], (float)d.mag.x/100.0, labels[i++], (float)d.mag.y/100.0, labels[i++], (float)d.mag.z/100.0,
    labels[i++], (float)d.volt/1000.0,
    labels[i++], d.pm1_0, labels[i++], d.pm2_5, labels[i++], d.pm10_0,
    labels[i++], d.p03um, labels[i++], d.p05um, labels[i++], d.p10um,
    labels[i++], d.lat, labels[i++], d.lon, labels[i++], d.altitude,
    labels[i++], d.q2G, labels[i++], d.flags, labels[i++], rssi, labels[i++], snr, labels[i++], err);
}

void generateErrorRow(char buffer[], int len, std::vector<uint8_t> payload, int expected, int rssi, float snr) {
  char hexBuffer[512] = ""; 
  char hexByte[4];
  for (size_t i = 0; i < payload.size() && i < 120; i++) {
    snprintf(hexByte, sizeof(hexByte), "%02X", payload[i]);
    strcat(hexBuffer, hexByte);
  }
  snprintf(buffer, len,
    "{\"malformed\":1,\"len\":%d,\"expected\":%d,\"rssi\":%d,\"snr\":%.2f,\"raw\":\"%s\"}",
    (int)payload.size(), expected, rssi, snr, hexBuffer);
}#include <SPI.h>
#include <LoRa.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

#define CS    4
#define RST   15
#define DIO0  2

BluetoothSerial SerialBT;

// Added packed attribute to prevent memory gaps during memcpy
struct __attribute__((packed)) float3d {
  float x, y, z;
};

struct __attribute__((packed)) SensorData {
  float lon, lat, altitude;
  unsigned long now;
  unsigned int UT_seconds;
  float AHT_temp, AHT_hum;
  float BMP_temp, BMP_pres;
  float3d accel;
  float3d gyro;
  float gtemp;
  float3d mag;
  float volt;
  unsigned int pm1_0, pm2_5, pm10_0;
  unsigned int p03um, p05um, p10um;
  int q2G;
};

const char* labels[] = {
  "now[ms]", "UT[s]",
  "AHT_tmp[C]", "AHT_hum",
  "BMP_temp[C]", "BMP_pres",
  "gx", "gy", "gz",
  "ax[m/s2]", "ay[m/s2]", "az[m/s2]",
  "gtemp[C]",
  "magx[uT]", "magy[uT]", "magz[uT]",
  "voltage",
  "pm1_0", "pm2_5", "pm10_0",
  "p03um", "p05um", "p10um",
  "lat", "lon", "altitude", "2Grssi", "rssi", "snr"
};

int normal_size;
SensorData data;
int rssi;
float snr;
char row[4096];

void setup() {
  Serial.begin(115200);
  // Changed to false so your phone can discover the ESP32
  SerialBT.begin("AuroraData", false); 

  LoRa.setPins(CS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  normal_size = sizeof(SensorData) + 2; 
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);

  Serial.println("LoRa receiver ready");
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize > 0) {
    std::vector<uint8_t> payload;
    while (LoRa.available()) {
      payload.push_back(LoRa.read());
    }
    rssi = LoRa.packetRssi();
    snr = LoRa.packetSnr();

    if (payload.size() < (size_t)normal_size) {
      generateErrorRow(row, sizeof(row), payload, normal_size, rssi, snr);
      SerialBT.println(row);
      Serial.println(row);
      return;
    }

    if (payload[0] != 0xAA || payload[normal_size - 1] != 0xBB) {
      generateErrorRow(row, sizeof(row), payload, normal_size, rssi, snr);
      SerialBT.println(row);
      Serial.println(row);
      return;
    }

    memcpy(&data, &payload[1], sizeof(SensorData));

    dataToJson(data, row, sizeof(row), snr, rssi);
    SerialBT.println(row);
    Serial.println(row);
  }
}

void dataToJson(SensorData data, char buffer[], int len, float snr, int rssi) {
  int i = 0;
  // Fixed: Number of format specifiers now matches the 29 labels/variables
  snprintf(buffer, len,
    "{\"%s\":%lu,\"%s\":%u,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,"
    "\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,"
    "\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%u,"
    "\"%s\":%u,\"%s\":%u,\"%s\":%u,\"%s\":%u,\"%s\":%u,\"%s\":%.9f,\"%s\":%.9f,"
    "\"%s\":%.0f,\"%s\":%d,\"%s\":%d,\"%s\":%.2f}",
    labels[i++], data.now, labels[i++], data.UT_seconds,
    labels[i++], data.AHT_temp, labels[i++], data.AHT_hum,
    labels[i++], data.BMP_temp, labels[i++], data.BMP_pres,
    labels[i++], data.gyro.x, labels[i++], data.gyro.y, labels[i++], data.gyro.z,
    labels[i++], data.accel.x, labels[i++], data.accel.y, labels[i++], data.accel.z,
    labels[i++], data.gtemp,
    labels[i++], data.mag.x, labels[i++], data.mag.y, labels[i++], data.mag.z,
    labels[i++], data.volt,
    labels[i++], data.pm1_0, labels[i++], data.pm2_5, labels[i++], data.pm10_0,
    labels[i++], data.p03um, labels[i++], data.p05um, labels[i++], data.p10um,
    labels[i++], data.lat, labels[i++], data.lon, labels[i++], data.altitude,
    labels[i++], data.q2G, labels[i++], rssi, labels[i++], snr);
}

void generateErrorRow(char buffer[], int len, std::vector<uint8_t> payload, int normal_size, int rssi, float snr) {
  snprintf(buffer, len,
    "{\"error\":\"malformed_packet\",\"len\":%d,\"expected\":%d,\"rssi\":%d,\"snr\":%.3f}",
    (int)payload.size(), normal_size, rssi, snr);
}
