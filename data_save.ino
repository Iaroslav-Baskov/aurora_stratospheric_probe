#include "FS.h"
#include "SD.h"
#include "GY521.h"
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <BMP280.h>
#include <Adafruit_AHTX0.h>
#include <Wire.h>

#define CS_PIN    4
#define RST       14
#define DIO0      27

#define RXD1      25
#define TXD1      26
#define RXD2      16
#define TXD2      17

#define SD_CS     5
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23

#define HOT_PIN   32
#define TEST_PIN  35

#define QMC5883L_ADDRESS 0x0D
#define SCALE_FACTOR 1.5

GY521 sensor(0x68);
BMP280 bmp280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;

String labels[]={
"now[ms]","UT[s]",
"AHT_tmp[C]","AHT_hum",
"BMP_temp[C]","BMP_pres",
"ax","ay","az",
"gtemp",
"magx","magy","magz",
"volt",
"pm1_0","pm2_5","pm10_0",
"p03um","p05um","p10um",
"lat","lon","altitude",
  };

struct SensorData {
  unsigned long now;
  unsigned int UT_seconds;
  float AHT_temp, AHT_hum;
  float BMP_temp, BMP_pres;
  float ax, ay, az;
  float gx, gy, gz;
  float gtemp;
  float magx, magy, magz;
  float volt;
  unsigned int pm1_0, pm2_5, pm10_0;
  unsigned int p03um, p05um, p10um;
  float lon, lat, altitude;
};

SensorData data;

char row[2048];
char date[32];
int unitime;
uint8_t payload[106];
uint8_t pmsBuffer[32];


unsigned long lastSMS = 0;
unsigned long lastLoRa = 0;


const unsigned long smsInterval = 60000;
unsigned long loraInterval=0;

const char* number = "+359892777567";

float offsetX = 328.3;
float offsetY = 235.7;
float offsetZ = 83.3;

int power = 0;

void generatePayload() {
  payload[0] = 0xAA;  // стартовый байт

  memcpy(&payload[1], &data, sizeof(SensorData));

  payload[1 + sizeof(SensorData)] = 0xBB;  // конечный байт
}


unsigned int timeOnAir_ms(uint8_t sf, float bw, uint8_t cr, int len, bool header, bool crc) { // calculate time-on-air in milliseconds for LoRa packet 
  double ts = (double)(1 << sf) / bw * 1000; // symbol duration in ms 
  double pl = len + (header ? 0 : 4); 
  double nPayload = 8 + std::max( ceil((8 * pl - 4 * sf + 28 + 16 * crc - 20) / (double)(4 * (sf - 2))) * (cr + 4), 0.0 ); 
  double tOnAir = (12.25 + nPayload) * ts; 
  return (unsigned int)ceil(tOnAir); 
}

void sendSMS() {
  Serial2.println("AT+CMGF=1");
  delay(200);
  Serial2.print("AT+CMGS=\"");
  Serial2.print(number);
  Serial2.println("\"");
  delay(200);
  dataToJson(data,row,sizeof(row));
  Serial2.print(row);
  Serial2.write(26);
}

void getMag(float &x, float &y, float &z) {
  int16_t mx, my, mz;

  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.requestFrom(QMC5883L_ADDRESS, 6);

  if (Wire.available() == 6) {
    mx = Wire.read() | (Wire.read() << 8);
    my = Wire.read() | (Wire.read() << 8);
    mz = Wire.read() | (Wire.read() << 8);

    x = (mx - offsetX) * SCALE_FACTOR;
    y = (my - offsetY) * SCALE_FACTOR;
    z = (mz - offsetZ) * SCALE_FACTOR;
  }
}
bool readPMSFrame(Stream &serial, uint8_t *buffer) { // Чтение 32 байт из PMS 
  if (serial.available() >=2) { 
    if (serial.read() == 0x42 && serial.read() == 0x4D) {
      buffer[0] = 0x42; buffer[1] = 0x4D; 
      for (int i = 2; i < 32; i++) { 
        if (serial.available() == 0) return false;
          buffer[i] = serial.read();} 
      return true;
    }
  } 
return false; }
void dataToJson(SensorData data,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{%s:%d,%s:%d,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.0f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.4f,%s:%d,%s:%d,%s:%d,%s:%d,%s:%d,%s:%d,%s:%.9f,%s:%.9f,%s:%.0f}\n",
    labels[i++],data.now,labels[i++],data.UT_seconds,
    labels[i++],data.AHT_temp,labels[i++],data.AHT_hum,
    labels[i++],data.BMP_temp,labels[i++],data.BMP_pres,
    labels[i++],data.ax,labels[i++],data.ay,labels[i++],data.az,
    labels[i++],data.gtemp,
    labels[i++],data.magx,labels[i++],data.magy,labels[i++],data.magz,
    labels[i++],data.volt,
    labels[i++],data.pm1_0,labels[i++], data.pm2_5,labels[i++], data.pm10_0,
    labels[i++],data.p03um, labels[i++],data.p05um, labels[i++],data.p10um,
    labels[i++],data.lat,
    labels[i++],data.lon,
    labels[i++],data.altitude,
    labels[i++]);
}
void dataToCsv(SensorData data,char buffer[],int len){
snprintf(buffer, len,
    "%d,%d,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.0f\n",
    data.now,data.UT_seconds,
    data.AHT_temp,data.AHT_hum,
    data.BMP_temp,data.BMP_pres,
    data.ax,data.ay,data.az,
    data.gtemp,
    data.magx,data.magy,data.magz,
    data.volt,
    data.pm1_0, data.pm2_5, data.pm10_0,
    data.p03um, data.p05um, data.p10um,
    data.lat,
    data.lon,
    data.altitude
  );
}
void setup() {
  Serial.begin(9600);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Wire.begin();
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(0x09);
  Wire.write(0b00011101); // OSR=512, 8G, 200Hz, continuous
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();
  
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);

  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  pinMode(HOT_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  digitalWrite(CS_PIN, HIGH);
  digitalWrite(SD_CS, HIGH);

  SD.begin(SD_CS);
  File file = SD.open("/data.csv");
  if (!file.available()) {
    file.close();
    file = SD.open("/data.csv");
    if(file){
      for(int i=0;i<sizeof(labels)/sizeof(labels[0]);i++){
        file.print(labels[i]);
        if(i<i<sizeof(labels)/sizeof(labels[0])-1)file.print(",");
        else file.print("/n");
        
      }
      file.close();}
  }

  LoRa.setPins(CS_PIN, RST, DIO0);
  LoRa.begin(433E6);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);
  LoRa.setTxPower(20);

  loraInterval=timeOnAir_ms(12, 62.5E3, 8, sizeof(payload), true, true);
  Serial.print("loraInterval:");
  Serial.println(loraInterval);

  aht.begin();
  bmp280.begin();
  sensor.wakeup();
}

void loop() {
  if(Serial1.available()){
    while (Serial1.available()) {
      gps.encode(Serial1.read());
    }
  
    if (gps.location.isValid()) {
      data.lat = gps.location.lat();
      data.lon = gps.location.lng();
    }
  
    if (gps.altitude.isValid())
      data.altitude = gps.altitude.meters();
    data.UT_seconds=gps.time.second()+gps.time.minute()*60+gps.time.hour()*3600;
  }
  if (readPMSFrame(Serial, pmsBuffer)) { 
    Serial.println("OK");
    data.pm1_0 = (pmsBuffer[10] << 8) | pmsBuffer[11];
    data.pm2_5 = (pmsBuffer[12] << 8) | pmsBuffer[13];
    data.pm10_0 = (pmsBuffer[14] << 8) | pmsBuffer[15];
    data.p03um = (pmsBuffer[16] << 8) | pmsBuffer[17];
    data.p05um = (pmsBuffer[18] << 8) | pmsBuffer[19];
    data.p10um = (pmsBuffer[20] << 8) | pmsBuffer[21];
  }
    
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

  data.AHT_temp = temp.temperature;
  data.AHT_hum = humidity.relative_humidity;

  data.BMP_temp = bmp280.getTemperature();
  data.BMP_pres = bmp280.getPressure();

  data.volt =analogRead(TEST_PIN)*3.10/4095+0.97;

  sensor.read();

  data.ax = sensor.getAccelX();
  data.ay = sensor.getAccelY();
  data.az = sensor.getAccelZ();

  data.gx = sensor.getGyroX();
  data.gy = sensor.getGyroY();
  data.gz = sensor.getGyroZ();

  data.gtemp = sensor.getTemperature() - 20;
  
  if (data.gtemp < 40) { 
    if (data.volt > 3.15) 
      {if (power <= 250) power += 5;
      else if (power >= 10) power -= 10;}
    else power = 0;
  }
  data.now=millis();
  getMag(data.magx, data.magy, data.magz);
  
  dataToCsv(data,row,sizeof(row));


  File file = SD.open("/data.csv", FILE_APPEND);
  if (file) {
    file.print(row);
    file.close();
  }else{
    SD.begin(SD_CS);
  }

  if (millis() - lastSMS > smsInterval) {
    lastSMS = millis();
    sendSMS();
  }

  if (millis() - lastLoRa > loraInterval) {
    lastLoRa = millis();

    generatePayload();
    LoRa.beginPacket();
    LoRa.write(payload, 1 + sizeof(SensorData) + 1);
    LoRa.endPacket(true);
  }

  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
  delay(10);
  
}
