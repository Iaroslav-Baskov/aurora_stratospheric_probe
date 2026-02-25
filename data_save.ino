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

const char* labels[]={
  "now[ms]","UT[s]",
  "AHT_tmp[C]","AHT_hum",
  "BMP_temp[C]","BMP_pres",
  "gx","gy","gz",
  "ax","ay","az",
  "gtemp",
  "magx","magy","magz",
  "volt",
  "pm1_0","pm2_5","pm10_0",
  "p03um","p05um","p10um",
  "lat","lon","altitude",
};


struct float3d{
  float x,y,z;};
struct int16_t3d{
  int16_t x,y,z;};
  
struct SensorData {
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
  float lon, lat, altitude;
};
struct device{
  bool OK;
  unsigned long last;
  unsigned long last_alive;
  unsigned long timeout;
};
struct SystemInfo {
  device SD,LoRa,SMS,pms,AHT,BMP,gyro,mag,GPS;
};
struct sensor1d{
  float offset;
  float scale;
  float tempOffset;
  float t0;
};
struct sensor3d{
  float3d offset;
  float3d scale;
  float3d tempOffset;
  float t0;
};
struct calibrationInfo{
  sensor1d AHT_temp, AHT_hum;
  sensor1d BMP_temp, BMP_pres;
  sensor1d volt;
  sensor1d gtemp;
  sensor1d accel;
  sensor3d gyro;
  sensor3d mag;
};

SensorData data;
calibrationInfo calibrator;
SystemInfo check;

char row[2048];
int unitime;
uint8_t payload[sizeof(SensorData) + 2];
uint8_t pmsBuffer[32];


const unsigned long smsInterval = 60000;
unsigned long loraInterval=0;

const char* number = "+359892777567";

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

float calibrate(sensor1d dataset, float value,float t=-300){
  if(t!=-300)return (value-dataset.offset-dataset.tempOffset*(t-dataset.t0))*dataset.scale;
  return (value-dataset.offset)*dataset.scale;
}
int calibrate(sensor1d dataset, int value,float t=-300){
  if(t!=-300)return (int)((value-dataset.offset-dataset.tempOffset*(t-dataset.t0))*dataset.scale);
  return (int)((value-dataset.offset)*dataset.scale);
}
float3d calibrate(sensor3d dataset, float3d value,float t=-300){
  float3d v;
  if (t!=-300){
    v.x=(value.x-dataset.offset.x-dataset.tempOffset.x*(t-dataset.t0))*dataset.scale.x;
    v.y=(value.y-dataset.offset.y-dataset.tempOffset.y*(t-dataset.t0))*dataset.scale.y;
    v.z=(value.z-dataset.offset.z-dataset.tempOffset.z*(t-dataset.t0))*dataset.scale.z;}
  else{
    v.x=(value.x-dataset.offset.x)*dataset.scale.x;
    v.y=(value.y-dataset.offset.y)*dataset.scale.y;
    v.z=(value.z-dataset.offset.z)*dataset.scale.z;
  }
  return v;
}
float3d calibrate(sensor3d dataset, int16_t3d value,float t=-300){
  float3d v;
  if (t!=-300){
    v.x=((float)value.x-dataset.offset.x-dataset.tempOffset.x*(t-dataset.t0))*dataset.scale.x;
    v.y=((float)value.y-dataset.offset.y-dataset.tempOffset.y*(t-dataset.t0))*dataset.scale.y;
    v.z=((float)value.z-dataset.offset.z-dataset.tempOffset.z*(t-dataset.t0))*dataset.scale.z;}
  else{
    v.x=((float)value.x-dataset.offset.x)*dataset.scale.x;
    v.y=((float)value.y-dataset.offset.y)*dataset.scale.y;
    v.z=((float)value.z-dataset.offset.z)*dataset.scale.z;
  }
  return v;
}
void getMag(float3d &output,sensor3d calibrator,float t) {
  if(check.mag.OK){
    check.mag.last = millis();
    int16_t3d result;
  
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(QMC5883L_ADDRESS, 6);
  
    if (Wire.available() >= 6) {
      result.x = Wire.read() | (Wire.read() << 8);
      result.y = Wire.read() | (Wire.read() << 8);
      result.z = Wire.read() | (Wire.read() << 8);
      output=calibrate(calibrator,result,t=t);
   }
  }
}
void checkI2CDevices() {
    check.gyro.OK = i2cDevicePresent(0x68);
    if(check.gyro.OK)check.gyro.last=millis();
    check.BMP.OK = i2cDevicePresent(0x76);
    if(check.BMP.OK)check.BMP.last=millis();
    check.AHT.OK = i2cDevicePresent(0x38);
    if(check.AHT.OK)check.AHT.last=millis();
    check.mag.OK = i2cDevicePresent(QMC5883L_ADDRESS);
    if(check.mag.OK)check.mag.last=millis();
}

bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}
bool readPMSFrame(Stream &serial, uint8_t *buffer) { // Чтение 32 байт из PMS 
  if (serial.available() >=2) { 
    if (serial.read() == 0x42 && serial.read() == 0x4D) {
      buffer[0] = 0x42; buffer[1] = 0x4D; 
      for (int i = 2; i < 32; i++) { 
        if (serial.available() == 0) return false;
          buffer[i] = serial.read();} 
        data.pm1_0 = (buffer[10] << 8) | buffer[11];
        data.pm2_5 = (buffer[12] << 8) | buffer[13];
        data.pm10_0 = (buffer[14] << 8) | buffer[15];
        data.p03um = (buffer[16] << 8) | buffer[17];
        data.p05um = (buffer[18] << 8) | buffer[19];
        data.p10um = (buffer[20] << 8) | buffer[21];
        check.pms.OK=true;
        check.pms.last=millis();
        return true;
    }
  }
  data.pm1_0 = 0;
  data.pm2_5 = 0;
  data.pm10_0 =0;
  data.p03um = 0;
  data.p05um = 0;
  data.p10um = 0; 
  if(millis()-check.pms.last>check.pms.timeout){
    
    check.pms.last=millis();
    check.pms.OK=false;
    pmsConnect();
  }
  return false; }

void dataToJson(SensorData data,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{%s:%d,%s:%d,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.0f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.4f,%s:%d,%s:%d,%s:%d,%s:%d,%s:%d,%s:%d,%s:%.9f,%s:%.9f,%s:%.0f}\n",
    labels[i++],data.now,labels[i++],data.UT_seconds,
    labels[i++],data.AHT_temp,labels[i++],data.AHT_hum,
    labels[i++],data.BMP_temp,labels[i++],data.BMP_pres,
    labels[i++],data.gyro.x,labels[i++],data.gyro.y,labels[i++],data.gyro.z,
    labels[i++],data.accel.x,labels[i++],data.accel.y,labels[i++],data.accel.z,
    labels[i++],data.gtemp,
    labels[i++],data.mag.x,labels[i++],data.mag.y,labels[i++],data.mag.z,
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
    "%d,%d,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.4f,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.0f\n",
    data.now,data.UT_seconds,
    data.AHT_temp,data.AHT_hum,
    data.BMP_temp,data.BMP_pres,
    data.gyro.x,data.gyro.y,data.gyro.z,
    data.accel.x,data.accel.y,data.accel.z,
    data.gtemp,
    data.mag.x,data.mag.y,data.mag.z,
    data.volt,
    data.pm1_0, data.pm2_5, data.pm10_0,
    data.p03um, data.p05um, data.p10um,
    data.lat,
    data.lon,
    data.altitude
  );
}
void sdConnect(){
  Serial.println("sd connect");
  check.SD.OK=SD.begin(SD_CS);
  if(check.SD.OK){
    check.SD.last=millis();
    File file = SD.open("/data.csv");
    if (!file) {
      file.close();
      file = SD.open("/data.csv");
      if(file){
        for(int i=0;i<sizeof(labels)/sizeof(labels[0]);i++){
          file.print(labels[i]);
          if(i<sizeof(labels)/sizeof(labels[0])-1)file.print(",");
          else file.print("\n");
        }
        file.close();}
    }
  }
}

void LoRaConnect(){
  Serial.println("lora connect");
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);
  LoRa.sleep();
  check.LoRa.OK=LoRa.begin(433E6);
  if(check.LoRa.OK){
    check.LoRa.last=millis();
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setCodingRate4(8);
    LoRa.setTxPower(20);}
}
void ahtConnect(){
  Serial.println("aht connect");
  check.AHT.OK=aht.begin();
  if(check.AHT.OK)check.AHT.last=millis();
}
void bmpConnect(){
  Serial.println("bmp connect");
  check.BMP.OK=bmp280.begin();
  if(check.BMP.OK)check.BMP.last=millis();
}
void accelConnect(){
  
  Serial.println("accel connect");
  check.gyro.OK=sensor.wakeup();
  if(check.gyro.OK) check.gyro.last=millis();
}
void magnConnect(){
  Serial.println("magn connect");
  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(0x09);
  Wire.write(0b00011101); // OSR=512, 8G, 200Hz, continuous
  Wire.endTransmission();

  Wire.beginTransmission(QMC5883L_ADDRESS);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();

  delay(50);
  check.mag.OK = i2cDevicePresent(QMC5883L_ADDRESS);
  if(check.mag.OK)check.mag.last=millis();
}
void startI2CDevices(){
  delay(500);
  Wire.begin();
  ahtConnect();
  bmpConnect();
  accelConnect();
  magnConnect();
}
void pmsConnect(){
  Serial.end();
  delay(500);
  Serial.begin(9600);
  check.pms.last=millis();
  Serial.println("pms connect");
}
void gpsConnect(){
  
  Serial.println("gps connect");
  Serial1.end();
  delay(500);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  check.GPS.last=millis();
}
bool readGPSFrame(Stream &serial){
  if(serial.available()){
    while (serial.available()) {
      gps.encode(serial.read());
      check.GPS.last=millis();
      check.GPS.OK=true;}
    if (gps.location.isValid()) {
      data.lat = gps.location.lat();
      data.lon = gps.location.lng();}
    if (gps.altitude.isValid()){
      data.altitude = gps.altitude.meters();}
    if (gps.time.isValid()){
      data.UT_seconds=gps.time.second()+gps.time.minute()*60+gps.time.hour()*3600;
    }
  }
  if(millis()-check.GPS.last>check.GPS.timeout){
    check.GPS.last=millis();
    check.GPS.OK=false;
    gpsConnect();
  }
  return check.GPS.OK;
}
void smsConnect(){
  Serial.println("sms connect");
  Serial2.end();
  delay(500);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  checkSMS(Serial2);
}
bool checkSMS(Stream &serial){
  serial.println("AT");
  delay(100);
  if(serial.available()>=3){
    if(serial.read()=='O' && serial.read()=='K' && serial.read()=='\n'){
      check.SMS.last=millis();
      check.SMS.OK=true;
      return true;}
  }
  check.SMS.OK=false;
  return false;
}
void sendSMS(Stream &serial) {
  if(checkSMS(serial)){
      serial.println("AT+CMGF=1");
      delay(200);
      serial.print("AT+CMGS=\"");
      serial.print(number);
      serial.println("\"");
      delay(200);
      dataToJson(data,row,sizeof(row));
      serial.print(row);
      serial.write(26);
  }else smsConnect();
}

void setup() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  startI2CDevices();
  
  pinMode(HOT_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  LoRa.setPins(CS_PIN, RST, DIO0);
  check.SMS.timeout=5000;
  check.GPS.timeout=5000;
  check.pms.timeout=5000;
  check.AHT.timeout=1000;
  check.BMP.timeout=1000;
  check.gyro.timeout=1000;
  check.mag.timeout=1000;

  calibrator.mag.offset.x=0;
  calibrator.mag.offset.y=0;
  calibrator.mag.offset.z=0;
  calibrator.mag.scale.x=1.5;
  calibrator.mag.scale.y=1.5;
  calibrator.mag.scale.z=1.5;
  loraInterval=timeOnAir_ms(12, 62.5E3, 8, sizeof(payload), true, true);
  
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(SD_CS, HIGH);
  
  gpsConnect();
  pmsConnect();
  smsConnect();
  sdConnect();
  LoRaConnect();
}

void loop() {
  readGPSFrame(Serial1);
  readPMSFrame(Serial, pmsBuffer);
  
  checkI2CDevices();
  if(!(check.AHT.OK || check.BMP.OK || check.gyro.OK || check.mag.OK)){
    startI2CDevices();
  }
  if(check.gyro.OK){
    if(sensor.read()){
      data.accel.x = sensor.getAccelX();
      data.accel.y = sensor.getAccelY();
      data.accel.z = sensor.getAccelZ();
    
      data.gyro.x = sensor.getGyroX();
      data.gyro.y = sensor.getGyroY();
      data.gyro.z = sensor.getGyroZ();
  
      data.gtemp = calibrate(calibrator.gtemp,sensor.getTemperature());
      check.gyro.last=millis();}
  }
  else if (millis()-check.gyro.last>check.gyro.timeout){
    accelConnect();
  }
  if(check.AHT.OK){
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    data.AHT_temp = temp.temperature;
    data.AHT_hum = humidity.relative_humidity;
  }
  else if (millis()-check.AHT.last>check.AHT.timeout){
    ahtConnect();
  }
  if(check.BMP.OK){
    data.BMP_temp = bmp280.getTemperature();
    data.BMP_pres = bmp280.getPressure();
  }
  else if (millis()-check.BMP.last>check.BMP.timeout){
    bmpConnect();
  }
  if(check.mag.OK){
    getMag(data.mag,calibrator.mag,data.gtemp);}
  else if (millis()-check.mag.last>check.mag.timeout){
    magnConnect();
  }

  data.volt =analogRead(TEST_PIN)*3.10/4095+0.97;
  
  
  if (data.gtemp < 40) { 
    if (data.volt > 3.15) 
      {if (power <= 250) power += 5;
      else if (power >= 10) power -= 10;}
    else power = 0;
  }
  
  data.now=millis();
  
  
  dataToCsv(data,row,sizeof(row));
  Serial.print(row);
  if(check.SD.OK){
    File file = SD.open("/data.csv", FILE_APPEND);
    if (file) {
      check.SD.last=millis();
      file.print(row);
      file.close();
      check.SD.last=millis();
    }else{
      check.SD.OK=false;
    }
  }else sdConnect();

  if (millis() - check.SMS.last> smsInterval) {
    sendSMS(Serial2);
  }

  if (millis() - check.LoRa.last > loraInterval) {
    generatePayload();
    LoRa.beginPacket();
    LoRa.write(payload, 1 + sizeof(SensorData) + 1);
    check.LoRa.OK=LoRa.endPacket(true);
    if(check.LoRa.OK)check.LoRa.last=millis();
  }
  if(!check.LoRa.OK && millis()-check.LoRa.last>check.LoRa.timeout){
    LoRaConnect();
  }
}
