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

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

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


Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
GY521 sensor(0x68);
BMP280 bmp280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;

const char* labels[]={
  "now[ms]","UT[s]",
  "AHT_temp[C]","AHT_hum",
  "BMP_temp[C]","BMP_pres",
  "gx","gy","gz",
  "ax[m/s2]","ay[m/s2]","az[m/s2]",
  "gtemp[C]",
  "magx[uT]","magy[uT]","magz[uT]",
  "voltage", 
  "pm1_0","pm2_5","pm10_0",
  "p03um","p05um","p10um",
  "lat","lon","altitude","2G","flags"
};

const uint8_t GPSsettings[]={
  0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 
  0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
};

struct float3d{
  float x,y,z;};
struct int16_t3d{
  int16_t x,y,z;};
  
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

struct device{
  uint8_t flagpos=0;
  bool OK=false;
  unsigned long last=0;
  unsigned long timeout=1000;
};
struct heatDevice{
  uint8_t flagpos=0;
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
  sensor1d x;
  sensor1d y;
  sensor1d z;
  
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

int calibrate(sensor1d dataset, float value,float multiplier=1,float t=-300){
  if(t!=-300)return (value-dataset.offset-dataset.tempOffset*(t-dataset.t0))*dataset.scale;
  return (int)((value-dataset.offset)*dataset.scale*multiplier);
}
int calibrate(sensor1d dataset, int value,float multiplier=1,float t=-300){
  if(t!=-300)return (int)((value-dataset.offset-dataset.tempOffset*(t-dataset.t0))*dataset.scale);
  return (int)((value-dataset.offset)*dataset.scale*multiplier);
}
bool i2cDevicePresent(uint8_t address) {
    Wire.beginTransmission(address);
    return (Wire.endTransmission() == 0);
}
void checkI2CDevices() {
    check.gyro.OK = check.gyro.OK && i2cDevicePresent(0x68);
    check.BMP.OK = check.BMP.OK && i2cDevicePresent(0x77);
    check.AHT.OK =check.AHT.OK && i2cDevicePresent(0x38);
    check.mag.OK =check.mag.OK && i2cDevicePresent(0x1E);
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
    "{\"%s\":%d,\"%s\":%d,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%lu,"
    "\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,"
    "\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%d,"
    "\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%.9f,\"%s\":%.9f,"
    "\"%s\":%d,\"%s\":%d,\"%s\":%d}\n",
    labels[i++],data.now,labels[i++],data.UT_seconds,
    labels[i++],(float)data.AHT_temp/100,labels[i++],(float)data.AHT_hum/100,
    labels[i++],(float)data.BMP_temp/100,labels[i++],(long)data.BMP_pres*10,
    labels[i++],(float)data.gyro.x/100,labels[i++],(float)data.gyro.y/100,labels[i++],(float)data.gyro.z/100,
    labels[i++],(float)data.accel.x/1000,labels[i++],(float)data.accel.y/1000,labels[i++],(float)data.accel.z/1000,
    labels[i++],(float)data.gtemp/100,
    labels[i++],(float)data.mag.x/100,labels[i++],(float)data.mag.y/100,labels[i++],(float)data.mag.z/100,
    labels[i++],(float)data.volt/1000,
    labels[i++],data.pm1_0,labels[i++], data.pm2_5,labels[i++], data.pm10_0,
    labels[i++],data.p03um, labels[i++],data.p05um, labels[i++],data.p10um,
    labels[i++],data.lat,
    labels[i++],data.lon,
    labels[i++],data.altitude,
    labels[i++],data.q2G,
    labels[i++],data.flags);
}
void dataToCsv(SensorData data, char buffer[], int len) {
    snprintf(buffer, len,
    "%u,%u,%.2f,%.2f,%.2f,%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,%u,%u,%.9f,%.9f,%d,%d,%d\n",
    data.now, 
    data.UT_seconds,
    (float)data.AHT_temp / 100.0, 
    (float)data.AHT_hum / 100.0,
    (float)data.BMP_temp / 100.0, 
    (long)data.BMP_pres*10,
    (float)data.gyro.x / 100.0, 
    (float)data.gyro.y / 100.0, 
    (float)data.gyro.z / 100.0,
    (float)data.accel.x / 1000.0, 
    (float)data.accel.y / 1000.0, 
    (float)data.accel.z / 1000.0,
    (float)data.gtemp / 100.0,
    (float)data.mag.x / 100.0, 
    (float)data.mag.y / 100.0, 
    (float)data.mag.z / 100.0,
    (float)data.volt / 1000.0,
    data.pm1_0, 
    data.pm2_5, 
    data.pm10_0,
    data.p03um, 
    data.p05um, 
    data.p10um,
    data.lat,
    data.lon,
    data.altitude,
    data.q2G,
    data.flags
  );
}
void sdConnect(){
  Serial.println("sd connect");
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
}

void LoRaConnect(){
  Serial.println("lora connect");
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);
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
  digitalWrite(CS_PIN, LOW);
  // Send 0x42 (Version Register).
  // In SPI, the 8th bit is 0 for Read, 1 for Write.
  SPI.transfer(0x42 & 0x7F); 
  version = SPI.transfer(0x00); // Read the result
  
  digitalWrite(CS_PIN, HIGH);
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
void magnConnect() {
  Serial.println("HMC5883L connect...");
  check.mag.last = millis();
  check.mag.OK =mag.begin();
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
  delay(100);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  check.GPS.last=millis();
  delay(50);
  Serial1.write(GPSsettings,sizeof(GPSsettings));
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
      data.altitude = (uint16_t)gps.altitude.meters();}
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
  if(start>=0){
  output=output.substring(start+2,start+4);
  if(output[1]==','){
    output=output.substring(0,1);
  }
  return (int) calibrate(calibrator.q2G,(int)output.toInt());}
  return 0;
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

  LoRa.setPins(CS_PIN, RST, DIO0);
  check.LoRa.timeout=timeOnAir_ms(12, 62.5E3, 8-4, sizeof(payload), true, true);
  check.SMS.timeout=5000;
  check.GPS.timeout=5000;
  check.pms.timeout=5000;
  check.AHT.timeout=1000;
  check.BMP.timeout=1000;
  check.gyro.timeout=1000;
  check.mag.timeout=1000;
  
  calibrator.gyro.x.offset=2.336;
  calibrator.gyro.y.offset=2.351;
  calibrator.gyro.z.offset=-0.221;
  calibrator.accel.x.offset=0.035378469830884045;
  calibrator.accel.x.scale=10.01711296392382;
  calibrator.accel.y.offset=-0.010919070720751357;
  calibrator.accel.y.scale=9.854182783125843;
  calibrator.accel.z.offset=-0.0060115734257293755;
  calibrator.accel.z.scale=9.569241972040562;
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
      data.accel.x = calibrate(calibrator.accel.x,sensor.getAccelX(),1000);
      data.accel.y = calibrate(calibrator.accel.y,sensor.getAccelY(),1000);
      data.accel.z = calibrate(calibrator.accel.z,sensor.getAccelZ(),1000);
      data.gyro.x = calibrate(calibrator.gyro.x,sensor.getGyroX(),100);
      data.gyro.y = calibrate(calibrator.gyro.y,sensor.getGyroY(),100);
      data.gyro.z = calibrate(calibrator.gyro.z,sensor.getGyroZ(),100);
  
      data.gtemp = calibrate(calibrator.gtemp,sensor.getTemperature(),100);
      
      check.gyro.last=millis();
  }
  else if (millis()-check.gyro.last>check.gyro.timeout){
    accelConnect();
  }
  if(check.AHT.OK){
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    data.AHT_temp = temp.temperature*100;
    data.AHT_hum = humidity.relative_humidity*100;
  }
  else if (millis()-check.AHT.last>check.AHT.timeout){
    ahtConnect();
  }
  if(check.BMP.OK){
    data.BMP_temp = bmp280.getTemperature()*100;
    data.BMP_pres = bmp280.getPressure()/10;
  }
  else if (millis()-check.BMP.last>check.BMP.timeout){
    bmpConnect();
  }
  if(check.mag.OK){
      sensors_event_t event; 
      if (mag.getEvent(&event)) {
        data.mag.x=calibrate(calibrator.mag.x,event.magnetic.x,100);
        data.mag.y=calibrate(calibrator.mag.y,event.magnetic.y,100);
        data.mag.z=calibrate(calibrator.mag.z,event.magnetic.z,100);}
      else {
        check.mag.OK = false; // Mark as failed if the library returns false
      }
      check.mag.last=millis();
  }
  else if (millis()-check.mag.last>check.mag.timeout){
    magnConnect();
  }

  data.volt =calibrate(calibrator.volt,analogRead(TEST_PIN),1000);
  
  
//  if (data.gtemp < check.termo.targetTemp-check.termo.tempThreshold && data.volt/1000 > check.termo.minVoltage+check.termo.voltThreshold)
//  { check.termo.isOn=true;
//    digitalWrite(HOT_PIN,HIGH);}
//  else if(data.gtemp > check.termo.targetTemp || data.volt/1000 < check.termo.minVoltage){
//    check.termo.isOn=false;
//    digitalWrite(HOT_PIN,LOW);}
  
  
  data.now=millis();
  if(data.now%2000>1000){
    check.termo.isOn=true;
    digitalWrite(HOT_PIN,HIGH);
  }
  else{
    check.termo.isOn=false;
    digitalWrite(HOT_PIN,LOW);
  }
  data.flags=(data.flags & ~(1 << check.termo.flagpos)) | (uint8_t(check.termo.isOn) << check.termo.flagpos);
  
  dataToCsv(data,row,sizeof(row));
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
  dataToJson(data,row,sizeof(row));
  Serial.print(row);
  if (millis() - check.SMS.last> check.SMS.timeout) {
    sendSMS(Serial2);
  }
  if(millis() - check.LoRa.last > check.LoRa.timeout){
  LoRaCheck();
  if (check.LoRa.OK) {
    generatePayload(payload);
    LoRa.beginPacket();
    LoRa.write(payload, 1 + sizeof(SensorData) + 1);
    LoRa.endPacket(true);
  }
  else{
    LoRaConnect();
  }
  check.LoRa.last=millis();}
}
