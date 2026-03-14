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

#define SD_CS     33
#define SCK_PIN   18
#define MISO_PIN  19
#define MOSI_PIN  23

#define HOT_PIN   32
#define TEST_PIN  35

#define QMC5883L_ADDRESS 0x1E

GY521 sensor(0x68);
BMP280 bmp280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;

const char* labels[]={
  "now[ms]","UT[s]",
  "AHT_tmp[C]","AHT_hum",
  "BMP_temp[C]","BMP_pres",
  "gx","gy","gz",
  "ax[m/s2]","ay[m/s2]","az[m/s2]",
  "gtemp[C]",
  "magx[G]","magy[G]","magz[G]",
  "voltage",
  "pm1_0","pm2_5","pm10_0",
  "p03um","p05um","p10um",
  "lat","lon","altitude","2G",
};


struct float3d{
  float x,y,z;};
struct int16_t3d{
  int16_t x,y,z;};
  
struct SensorData {
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

struct device{
  bool OK=false;
  unsigned long last=0;
  unsigned long timeout=1000;
};
struct heatDevice{
  bool isOn=false;
  unsigned long last=0;
  float targetTemp=0;
  float tempThreshold=0;
  float minVoltage=0;
  float voltThreshold;
};
struct SystemInfo {
  device SD,LoRa,SMS,pms,AHT,BMP,gyro,mag,GPS;
  heatDevice termo;
};
struct sensor1d{
  float offset=0;
  float scale=1;
  float tempOffset=0;
  float t0=0;
};
struct sensor3d{
  float3d offset={0,0,0};
  float3d scale={1,1,1};
  float3d tempOffset={0,0,0};
  float t0=0;
};
struct calibrationInfo{
  sensor1d volt;
  sensor1d q2G;
  sensor1d gtemp;
  sensor3d accel;
  sensor3d gyro;
  sensor3d mag;
};

SensorData data;
calibrationInfo calibrator;
SystemInfo check;

char row[2048];
uint8_t payload[sizeof(SensorData) + 2];
uint8_t pmsBuffer[32];

const char* number = "+359892777567";

unsigned int timeOnAir_ms(uint8_t sf, float bw, uint8_t cr, int len, bool header, bool crc) { // calculate time-on-air in milliseconds for LoRa packet 
  double ts = (double)(1 << sf) / bw * 1000; // symbol duration in ms 
  double pl = len + (header ? 0 : 4); 
  double nPayload = 8 + std::max( ceil((8 * pl - 4 * sf + 28 + 16 * crc - 20) / (double)(4 * (sf - 2))) * (cr + 4), 0.0 ); 
  double tOnAir = (12.25 + nPayload) * ts; 
  return (unsigned int)ceil(tOnAir); 
}
void generatePayload(uint8_t *payload) {
  payload[0] = 0xAA;  // стартовый байт

  memcpy(&payload[1], &data, sizeof(SensorData));

  payload[1 + sizeof(SensorData)] = 0xBB;  // конечный байт
}

float calibrate(sensor1d dataset, float value,float t=-300){
  if(t!=-300)return (value-dataset.offset-dataset.tempOffset*(t-dataset.t0))*dataset.scale;
  return (value-dataset.offset)*dataset.scale;
}
float calibrate(sensor1d dataset, int value,float t=-300){
  if(t!=-300)return (int)((value-dataset.offset-dataset.tempOffset*(t-dataset.t0))*dataset.scale);
  return ((value-dataset.offset)*dataset.scale);
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
bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}
void checkI2CDevices() {
    check.gyro.OK = check.gyro.OK && i2cDevicePresent(0x68);
    check.BMP.OK = check.BMP.OK && i2cDevicePresent(0x77);
    check.AHT.OK =check.AHT.OK && i2cDevicePresent(0x38);
    check.mag.OK =check.mag.OK && i2cDevicePresent(QMC5883L_ADDRESS);
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
  if(millis()-check.pms.last>check.pms.timeout){
    
    check.pms.last=millis();
    check.pms.OK=false;
    pmsConnect();
  }
  return false; }

void dataToJson(SensorData data,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{\"%s\":%d,\"%s\":%d,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.0f,"
    "\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,"
    "\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%d,"
    "\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%.9f,\"%s\":%.9f,"
    "\"%s\":%.0f,\"%s\":%d}\n",
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
    labels[i++],data.q2G,
    labels[i++]);
}
void dataToCsv(SensorData data,char buffer[],int len){
  snprintf(buffer, len,
    "%d,%d,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.0f,%d\n",
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
    data.altitude,
    data.q2G
  );
}
void sdConnect(){
  Serial.println("sd connect");
  digitalWrite(SD_CS, LOW);
  check.SD.OK=SD.begin(SD_CS);
  if(check.SD.OK){
    check.SD.last=millis();
    if (!SD.exists("/data.csv")) {
      File file = SD.open("/data.csv",FILE_WRITE);
      if(file){
        for(int i=0;i<sizeof(labels)/sizeof(labels[0]);i++){
          file.print(labels[i]);
          if(i<sizeof(labels)/sizeof(labels[0])-1)file.print(",");
          else file.print("\n");
        }
        file.close();}
    }
  }
  digitalWrite(SD_CS, HIGH);
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
void LoRaCheck(){
  uint8_t version = 0;
  
  // Start SPI manual transaction
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(SD_CS, HIGH);
  digitalWrite(CS_PIN, LOW); // Select LoRa
  
  // Send 0x42 (Version Register). 
  // In SPI, the 8th bit is 0 for Read, 1 for Write.
  SPI.transfer(0x42 & 0x7F); 
  version = SPI.transfer(0x00); // Read the result
  
  digitalWrite(CS_PIN, HIGH); // Deselect LoRa
  SPI.endTransaction();
  if(version!=0x12){
  check.LoRa.OK=false;}}
void ahtConnect(){
  Serial.println("aht connect");
  check.AHT.last=millis();
  if(i2cDevicePresent(0x38))check.AHT.OK=aht.begin();
  else Serial.println("aht not found");
}
void bmpConnect(){
  Serial.println("bmp connect");
  check.BMP.last=millis();
  if(i2cDevicePresent(0x77))check.BMP.OK=bmp280.begin()==0;
  else Serial.println("bmp not found");
}
void accelConnect(){
  Serial.println("accel connect");
  check.gyro.last=millis();
  if(i2cDevicePresent(0x68)){
    check.gyro.OK=sensor.wakeup();
    if(check.gyro.OK) {
      sensor.setAccelSensitivity(0);  //  2g
      sensor.setGyroSensitivity(0);   //  250 degrees/s
      sensor.setThrottle();
    }
  }else Serial.println("accel not found");
}
void magnConnect(){
  Serial.println("magn connect");
  check.mag.last=millis();
  
  check.mag.OK = i2cDevicePresent(QMC5883L_ADDRESS);
  if(check.mag.OK){
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x09);
    Wire.write(0b00011101); // OSR=512, 8G, 200Hz, continuous
    Wire.endTransmission();
  
    Wire.beginTransmission(QMC5883L_ADDRESS);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();
  }
  else Serial.println("magn no found");
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
  delay(50);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  delay(100);
  checkSMS(Serial2);
}
bool checkSMS(Stream &serial){
  while(serial.available())serial.read();
  serial.println("AT");
  delay(50);
  check.SMS.last=millis();
    if(serial.readString().indexOf("OK")!=-1){
      check.SMS.OK=true;
      return true;}
  
  check.SMS.OK=false;
  return false;
}
int get2gQuality(Stream &serial){
  while(serial.available())serial.read();
  serial.println("AT+CSQ");
  delay(50);
  String output=serial.readString();
  int start=output.indexOf(": ");
  output=output.substring(start+2,start+4);
  if(output[1]==','){
    output=output.substring(0,1);
  }
  return (int) calibrate(calibrator.q2G,(int)output.toInt());
}
void sendSMS(Stream &serial) {
  if(checkSMS(serial)){
      data.q2G=get2gQuality(serial);
      serial.println("AT+CMGF=1");
      delay(50);
      serial.print("AT+CMGS=\"");
      serial.print(number);
      serial.println("\"");
      delay(50);
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
  check.LoRa.timeout=timeOnAir_ms(12, 62.5E3, 8, sizeof(payload), true, true);
  check.SMS.timeout=5000;
  check.GPS.timeout=5000;
  check.pms.timeout=5000;
  check.AHT.timeout=1000;
  check.BMP.timeout=1000;
  check.gyro.timeout=1000;
  check.mag.timeout=1000;
  
  calibrator.gyro.tempOffset.x=-0.021586792358359117;
  calibrator.gyro.offset.x=2.0293925221537843;
  calibrator.gyro.tempOffset.y=-0.0203030523388205;
  calibrator.gyro.offset.y=2.0367279043462965;
  calibrator.gyro.tempOffset.z=-0.005832454539831707;
  calibrator.gyro.offset.z=-0.2910172807975372;
  calibrator.accel.offset.x=0.03764483705436618;
  calibrator.accel.scale.x=9.89667167880515;
  calibrator.accel.offset.y=0.0039877343303261386;
  calibrator.accel.scale.y=9.91839443251234;
  calibrator.accel.offset.z=-0.0007502589765589077;
  calibrator.accel.scale.z=9.620605996593481;
  calibrator.mag.offset.x=499.22560443025856;
  calibrator.mag.scale.x=0.00033468981790177587;
  calibrator.mag.offset.y=406.10800404767446;
  calibrator.mag.scale.y=0.00032792114858669303;
  calibrator.mag.offset.z=254.62126909439348;
  calibrator.mag.scale.z=0.0003375361890145562;
  calibrator.gtemp.offset=21.892719725589476;
  calibrator.gtemp.scale=0.8899545931264473;

  calibrator.volt.scale=3.10/4095;
  calibrator.volt.offset=-0.97*4095/3.10;
  
  calibrator.q2G.scale=2;
  calibrator.q2G.offset=113/2;
  check.termo.targetTemp=30;
  check.termo.tempThreshold=2;
  check.termo.minVoltage=3.1;
  check.termo.voltThreshold=0.2;
  
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
      sensor.read();
      data.accel.x = sensor.getAccelX();
      data.accel.y = sensor.getAccelY();
      data.accel.z = sensor.getAccelZ();
      data.gyro.x = sensor.getGyroX();
      data.gyro.y = sensor.getGyroY();
      data.gyro.z = sensor.getGyroZ();
  
      data.gtemp = calibrate(calibrator.gtemp,sensor.getTemperature());
      
      data.accel=calibrate(calibrator.accel,data.accel,data.gtemp);
      data.gyro=calibrate(calibrator.gyro,data.gyro,data.gtemp);
      check.gyro.last=millis();
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

  data.volt =calibrate(calibrator.volt,analogRead(TEST_PIN));
  
  
  if (data.gtemp < check.termo.targetTemp-check.termo.tempThreshold && data.volt > check.termo.minVoltage+check.termo.voltThreshold)digitalWrite(HOT_PIN,HIGH);
  else if(data.gtemp > check.termo.targetTemp || data.volt < check.termo.minVoltage)  digitalWrite(HOT_PIN,LOW);
  
  data.now=millis();
  
  
  dataToCsv(data,row,sizeof(row));
  Serial.print(row);
  if(check.SD.OK){
    digitalWrite(SD_CS, LOW); // Ensure SD is deselected
    digitalWrite(CS_PIN, HIGH);
    File file = SD.open("/data.csv", FILE_APPEND);
    if (file) {
      check.SD.last=millis();
      file.print(row);
      file.close();
      check.SD.last=millis();
    }else{
      check.SD.OK=false;
    }
    digitalWrite(SD_CS, HIGH);
  }else sdConnect();

  if (millis() - check.SMS.last> check.SMS.timeout) {
    sendSMS(Serial2);
  }
  if(millis() - check.LoRa.last > check.LoRa.timeout){
  LoRaCheck();
  if (check.LoRa.OK) {
    digitalWrite(SD_CS, HIGH);
    digitalWrite(CS_PIN, LOW);
    generatePayload(payload);
    LoRa.beginPacket();
    LoRa.write(payload, 1 + sizeof(SensorData) + 1);
    LoRa.endPacket(true);
    digitalWrite(CS_PIN, HIGH);
  }
  else{
    LoRaConnect();
  }
  check.LoRa.last=millis();}
}
