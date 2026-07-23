#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <LoRa.h>
#include <BMP280.h>
#include <GY521.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPSPlus.h>
#include <esp_task_wdt.h>
#include "RS-FEC.h"

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
#define CAM_PIN   13

#define SF 12
#define CR 4
#define FREETIME 10
#define BW 62500.0
#define REG_MODEM_CONFIG3  0x26

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
GY521 sensor(0x68);
BMP280 bmp280;
Adafruit_AHTX0 aht;
TinyGPSPlus gps;

const char* labels[]={
  "now[ms]",
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
  
struct __attribute__((packed)) ScientificData {
  unsigned long now;
  int16_t AHT_temp, AHT_hum;
  int16_t BMP_temp;
  uint16_t BMP_pres;
  uint16_t pm1_0, pm2_5, pm10_0; 
  uint16_t p03um, p05um, p10um;
  float lat, lon;
  uint16_t altitude;
};

struct __attribute__((packed)) TechnicalData {
  unsigned long now;
  int16_t3d gyro;
  int16_t3d accel;
  int16_t gtemp; 
  int16_t3d mag;
  int16_t volt;
  uint8_t q2G;
  uint8_t flags;
};


unsigned int timeOnAir_ms(uint8_t sf, float bw, uint8_t cr, int len, bool header, bool crc,int preamlen=8) { // calculate time-on-air in milliseconds for LoRa packet 
  double ts = (double)(1 << sf) / bw * 1000; // symbol duration in ms 
  double pl = len + (header ? 0 : 4); 
  double nPayload = 8 + std::max( ceil((8 * pl - 4 * sf + 28 + 16 * crc - 20) / (double)(4 * (sf - 2))) * (cr + 4), 0.0 ); 
  double tOnAir = (4.25 +preamlen+ nPayload) * ts; 
  return (unsigned int)ceil(tOnAir); 
}

template<typename T,uint8_t parity>
struct sender{
  T *sentData;
  uint8_t dataLen;
  uint8_t reg;
  unsigned long timeout;
  uint8_t quenue=0;
  uint8_t maxQuenue;
  RS::ReedSolomon<sizeof(T),parity> rs;
  uint8_t par=parity;
  uint8_t payload[sizeof(T)+parity+1];
  sender(T *data, uint8_t r,
  uint8_t maxQ): 
    sentData(data),
    dataLen(sizeof(T)),
    reg(r),
    timeout(timeOnAir_ms(SF, BW, CR-4, parity+sizeof(T)+1, true, true, 62) * FREETIME),
    maxQuenue(maxQ){}
};

struct device{
  uint8_t flagpos=0;
  bool OK=false;
  bool busy=false;
  unsigned long last=0;
  unsigned long timeout=1000;
};
struct heatDevice{
  uint8_t flagpos=0;
  
  bool isOn=false;
  unsigned long last=0;
  unsigned long lastSwitch=0;
  
  unsigned long timeout=1000;
  unsigned long workTime=0;
  unsigned long sleepTime=0;
  
  float targetTemp=0;
  float tempThreshold=0;
  
  float minVoltage=0;
  float voltThreshold=0;
};
struct SystemInfo {
  device SD,LoRa,SMS,pms,AHT,BMP,gyro,mag,GPS;
  heatDevice termo;
  heatDevice cam;
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

ScientificData allData;
TechnicalData additData;

sender<ScientificData, 8> sciSender(&allData, 0x01, 0);
sender<TechnicalData, 8> additSender(&additData, 0x08, 4);

calibrationInfo calibrator;
SystemInfo check;

char row[2048];
uint8_t pmsBuffer[32];

//const char* number = "+359892777567";
const char* number = "+359877914275";

uint8_t changeBit(uint8_t flags,uint8_t flagpos,bool valu){
  return (flags & ~(1 << flagpos)) | (uint8_t(valu) << flagpos);
}


uint8_t payload[sizeof(ScientificData)];

template<typename T,uint8_t parity>
void generatePayload(sender<T, parity> &Sender);

template<typename T,uint8_t parity>
void generatePayload(sender<T, parity> &Sender){
  
  uint8_t inputWorkBuffer[Sender.dataLen];
  
  // 2. Clear the buffer safely
  memset(inputWorkBuffer, 0, Sender.dataLen);
  
  // 3. Copy the raw data from your struct pointer (*sentData) into the working buffer
  memcpy(inputWorkBuffer, Sender.sentData, Sender.dataLen);

  // 4. Encode using the Reed-Solomon instance inside that specific sender.
  // This calculates the error-correction parity bytes and writes everything into Sender.payload
  Sender.rs.Encode(inputWorkBuffer, &Sender.payload[1]);
  Sender.payload[0]=Sender.reg;
  // Optional: Debug print to verify total packet size (Data bytes + Parity bytes)
  
  Serial.print("Payload generated. Total size: ");
  Serial.println(Sender.dataLen + Sender.par);
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
    additData.flags=changeBit(additData.flags,check.gyro.flagpos,check.gyro.OK);
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
        allData.pm1_0 = (buffer[10] << 8) | buffer[11];
        allData.pm2_5 = (buffer[12] << 8) | buffer[13];
        allData.pm10_0 = (buffer[14] << 8) | buffer[15];
        allData.p03um = (buffer[16] << 8) | buffer[17];
        allData.p05um = (buffer[18] << 8) | buffer[19];
        allData.p10um = (buffer[20] << 8) | buffer[21];
        check.pms.OK=true;
        additData.flags=changeBit(additData.flags,check.pms.flagpos,check.pms.OK);
        check.pms.last=millis();
        return true;
    }
  }
  if(millis()-check.pms.last>check.pms.timeout){
    check.pms.last=millis();
    check.pms.OK=false;
    additData.flags=changeBit(additData.flags,check.pms.flagpos,check.pms.OK);
    pmsConnect();
  }
  return false; 
}

void dataToJson(ScientificData allData, TechnicalData additData,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{\"%s\":%u,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%lu,"
    "\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%.3f,\"%s\":%.3f,"
    "\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%d,"
    "\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%d,\"%s\":%.9f,\"%s\":%.9f,"
    "\"%s\":%d,\"%s\":%d,\"%s\":%d}\n",
    labels[i++],allData.now,
    labels[i++],(float)allData.AHT_temp/100,labels[i++],(float)allData.AHT_hum/100,
    labels[i++],(float)allData.BMP_temp/100,labels[i++],(long)allData.BMP_pres*10,
    labels[i++],(float)additData.gyro.x/100,labels[i++],(float)additData.gyro.y/100,labels[i++],(float)additData.gyro.z/100,
    labels[i++],(float)additData.accel.x/1000,labels[i++],(float)additData.accel.y/1000,labels[i++],(float)additData.accel.z/1000,
    labels[i++],(float)additData.gtemp/100,
    labels[i++],(float)additData.mag.x/100,labels[i++],(float)additData.mag.y/100,labels[i++],(float)additData.mag.z/100,
    labels[i++],(float)additData.volt/1000,
    labels[i++],allData.pm1_0,labels[i++], allData.pm2_5,labels[i++], allData.pm10_0,
    labels[i++],allData.p03um, labels[i++],allData.p05um, labels[i++],allData.p10um,
    labels[i++],allData.lat,
    labels[i++],allData.lon,
    labels[i++],allData.altitude,
    labels[i++],additData.q2G,
    labels[i++],additData.flags);
}
void dataToSMS(ScientificData allData, TechnicalData additData,char buffer[],int len){
  snprintf(buffer, len,
    "{\"%s\":%u,\"%s\":%.2f,\"%s\":%.3f,\"%s\":%.7f,\"%s\":%.7f,\"%s\":%d,\"%s\":%d,\"%s\":%d,\n",
    labels[0],allData.now,
    labels[11],(float)additData.gtemp/100,
    labels[15],(float)additData.volt/1000,
    labels[22],allData.lat,
    labels[23],allData.lon,
    labels[24],allData.altitude,
    labels[25],additData.q2G,
    labels[26],additData.flags);
}
void dataToCsv(ScientificData allData, TechnicalData additData, char buffer[], int len) {
    snprintf(buffer, len,
    "%u,%.2f,%.2f,%.2f,%lu,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.3f,%u,%u,%u,%u,%u,%u,%.9f,%.9f,%d,%d,%d\n",
    allData.now, 
    (float)allData.AHT_temp / 100.0, 
    (float)allData.AHT_hum / 100.0,
    (float)allData.BMP_temp / 100.0, 
    (long)allData.BMP_pres*10,
    (float)additData.gyro.x / 100.0, 
    (float)additData.gyro.y / 100.0, 
    (float)additData.gyro.z / 100.0,
    (float)additData.accel.x / 1000.0, 
    (float)additData.accel.y / 1000.0, 
    (float)additData.accel.z / 1000.0,
    (float)additData.gtemp / 100.0,
    (float)additData.mag.x / 100.0, 
    (float)additData.mag.y / 100.0, 
    (float)additData.mag.z / 100.0,
    (float)additData.volt / 1000.0,
    allData.pm1_0, 
    allData.pm2_5, 
    allData.pm10_0,
    allData.p03um, 
    allData.p05um, 
    allData.p10um,
    allData.lat,
    allData.lon,
    allData.altitude,
    additData.q2G,
    additData.flags
  );
}

void sdConnect(){
  check.SD.OK=SD.begin(SD_CS);
  additData.flags=changeBit(additData.flags,check.SD.flagpos,check.SD.OK);
  if(check.SD.OK){
    check.SD.last=millis();
    if (!SD.exists("/allData.csv")) {
      File file = SD.open("/allData.csv",FILE_WRITE);
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

// -------------------------------------------------------------
// ИСПРАВЛЕНО: LoRaConnect и LoRaCheck
// -------------------------------------------------------------
uint8_t readRegister(uint8_t address) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address & 0x7F);
  uint8_t value = SPI.transfer(0);
  digitalWrite(CS_PIN, HIGH);
  return value;
}
void writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}
void LoRaConnect(){
  digitalWrite(CS_PIN, HIGH); // Гарантируем отключение SPI-устройства перед сбросом
  
  digitalWrite(RST, LOW);
  delay(10);
  digitalWrite(RST, HIGH);
  delay(10); // ВАЖНО: Даем время LoRa-модулю на аппаратную инициализацию
  
  check.LoRa.OK = LoRa.begin(433E6);
  additData.flags=changeBit(additData.flags,check.LoRa.flagpos,check.LoRa.OK);
  
  if(check.LoRa.OK){
    check.LoRa.last = millis();
    LoRa.setSpreadingFactor(SF);
    LoRa.setSignalBandwidth(BW);
    LoRa.setCodingRate4(CR);
    LoRa.setTxPower(20);
    LoRa.setPreambleLength(64);
    
    // LowDataRateOptimize для SF11+ и узкой полосы
    uint8_t modemConfig3 = readRegister(REG_MODEM_CONFIG3);
    modemConfig3 |= 0x01;  // LowDataRateOptimize = 1
    modemConfig3 |= 0x02;  // AgcAutoOn = 1
    writeRegister(REG_MODEM_CONFIG3, modemConfig3);

        uint8_t checkLDRO = readRegister(REG_MODEM_CONFIG3);
    Serial.print("LowDataRateOptimize: ");
    Serial.println((checkLDRO & 0x01) ? "ON" : "OFF");
    Serial.print("AgcAutoOn: ");
    Serial.println((checkLDRO & 0x02) ? "ON" : "OFF");
  }
  check.LoRa.busy=false;
}

void LoRaCheck(){
  uint8_t version = 0;
  
  // ВАЖНО: Отключаем SD-карту от шины перед ручной проверкой LoRa
  digitalWrite(SD_CS, HIGH); 
  
  // Start SPI manual transaction
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  // Send 0x42 (Version Register).
  // In SPI, the 8th bit is 0 for Read, 1 for Write.
  SPI.transfer(0x42 & 0x7F);
  version = SPI.transfer(0x00); // Read the result
  
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
  
  if(version != 0x12){
    check.LoRa.OK = false;
    check.LoRa.busy=false;
    additData.flags=changeBit(additData.flags,check.LoRa.flagpos,check.LoRa.OK);
  }
}
bool checkSent() {
  uint8_t irqFlags = 0;
  digitalWrite(SD_CS, HIGH); 

  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x12 & 0x7F);
  irqFlags = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  // Если шина выдает 0xFF — модуль физически отключился или завис
  if (irqFlags == 0xFF){
    check.LoRa.OK = false;
    check.LoRa.busy = false;
    return true;}

  // Если пакет ушел (бит 3 установлен)
  if (irqFlags & 0x08) {
    SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x12 | 0x80);
    SPI.transfer(0x08); // Очистка флага TxDone
    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    LoRa.idle(); 
    check.LoRa.busy = false;
    return false; // Успех
  }
  return true; // Все еще передает
}

void ahtConnect(){
  check.AHT.last=millis();
  if(i2cDevicePresent(0x38))check.AHT.OK=aht.begin();
}

void bmpConnect(){
  check.BMP.last=millis();
  if(i2cDevicePresent(0x77))check.BMP.OK=bmp280.begin()==0;
}

void accelConnect(){
  check.gyro.last=millis();
  if(i2cDevicePresent(0x68)){
    check.gyro.OK=sensor.wakeup();
    additData.flags=changeBit(additData.flags,check.gyro.flagpos,check.gyro.OK);
    if(check.gyro.OK) {
      sensor.setAccelSensitivity(0);  //  2g
      sensor.setGyroSensitivity(0);   //  250 degrees/s
      sensor.setThrottle();
    }
  }
}

void magnConnect() {
  check.mag.last = millis();
  check.mag.OK =mag.begin();
}

void startI2CDevices(){
  delay(500);
  Wire.begin();
  Wire.setTimeOut(1000);
  ahtConnect();
  bmpConnect();
  accelConnect();
  magnConnect();
}

void pmsConnect(){
  Serial.end();
  delay(400);
  Serial.begin(9600);
  Serial.setTimeout(100);
  check.pms.last=millis();
  //Serial.println("pms connect");
}

void gpsConnect(){
  Serial1.end();
  delay(100);
  Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
  Serial1.setTimeout(100);
  check.GPS.last=millis();
  delay(50);
  Serial1.write(GPSsettings,sizeof(GPSsettings));
}

bool readGPSFrame(Stream &serial){
  if(serial.available()){
    while (serial.available()) {
      gps.encode(serial.read());
      check.GPS.last=millis();
      check.GPS.OK=true;
      additData.flags=changeBit(additData.flags,check.GPS.flagpos,check.GPS.OK);}
    if (gps.location.isValid()) {
      allData.lat = gps.location.lat();
      allData.lon = gps.location.lng();}
    if (gps.altitude.isValid()){
      allData.altitude = (uint16_t)gps.altitude.meters();}
//    if (gps.time.isValid()){
//      allData.UT_seconds=gps.time.second()+gps.time.minute()*60+gps.time.hour()*3600;
//    }
  }
  if(millis()-check.GPS.last>check.GPS.timeout){
    check.GPS.last=millis();
    check.GPS.OK=false;
    additData.flags=changeBit(additData.flags,check.GPS.flagpos,check.GPS.OK);
    gpsConnect();
  }
  return check.GPS.OK;
}

void smsConnect(){
  Serial2.end();
  delay(50);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(200);
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
      additData.flags=changeBit(additData.flags,check.SMS.flagpos,check.SMS.OK);
      return true;}
  
  check.SMS.OK=false;
  additData.flags=changeBit(additData.flags,check.SMS.flagpos,check.SMS.OK);
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
      additData.q2G=get2gQuality(serial);
      serial.println("AT+CMGF=1");
      delay(50);
      serial.print("AT+CMGS=\"");
      serial.print(number);
      serial.println("\"");
      delay(50);
      dataToSMS(allData,additData,row,sizeof(row));
      serial.print(row);
      serial.write(26);
  }else smsConnect();
}

void setup() {
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = 5000,
    .idle_core_mask = (1<<0),
    .trigger_panic = true};
  esp_task_wdt_reconfigure(&twdt_config);
  esp_task_wdt_add(NULL);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  // -------------------------------------------------------------
  // ИСПРАВЛЕНО: Жёсткая инициализация пинов CS
  // -------------------------------------------------------------
  pinMode(CS_PIN, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(SD_CS, HIGH);
  // -------------------------------------------------------------
  
  startI2CDevices();
  
  pinMode(HOT_PIN, OUTPUT);
  pinMode(CAM_PIN, OUTPUT);
  digitalWrite(CAM_PIN,LOW);
  digitalWrite(HOT_PIN,LOW);

  LoRa.setPins(CS_PIN, RST, DIO0);
  check.LoRa.timeout=0;
  check.SMS.timeout=5000;
  check.GPS.timeout=5000;
  check.pms.timeout=10000;
  check.AHT.timeout=1000;
  check.BMP.timeout=1000;
  check.gyro.timeout=1000;
  check.mag.timeout=1000;

  check.termo.flagpos=0;
  check.LoRa.flagpos=1;
  check.GPS.flagpos=2;
  check.SMS.flagpos=3;
  check.SD.flagpos=4;
  check.pms.flagpos=5;
  check.gyro.flagpos=6;
  check.cam.flagpos=7;
  
  calibrator.gyro.x.offset=  2.336;
  calibrator.gyro.y.offset=  2.351;
  calibrator.gyro.z.offset= -0.221;
  calibrator.accel.x.offset= 0.035378469830884045;
  calibrator.accel.x.scale=  10.01711296392382;
  calibrator.accel.y.offset= -0.010919070720751357;
  calibrator.accel.y.scale=   9.854182783125843;
  calibrator.accel.z.offset= -0.0060115734257293755;
  calibrator.accel.z.scale=   9.569241972040562;
  calibrator.gtemp.offset=    21.892719725589476;
  calibrator.gtemp.scale=     0.8899545931264473;

  calibrator.volt.scale=  3.10/4095;
  calibrator.volt.offset=-0.97*4095/3.10;
  
  calibrator.q2G.scale= 1;
  calibrator.q2G.offset=0;

  
  check.termo.targetTemp =    1;
  check.termo.tempThreshold = 1;
  check.termo.minVoltage =    2.844;
  check.termo.voltThreshold = 0.117;
  check.termo.timeout =       100;
  
  
  check.cam.minVoltage =    2.844;
  check.cam.voltThreshold = 0.085;
  check.cam.targetTemp =    50;
  check.cam.tempThreshold = 10;
  check.cam.timeout =       1*1000*60;
  check.cam.workTime =      2*1000*60;
  check.cam.sleepTime =     8*1000*60;
  
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
      additData.accel.x = calibrate(calibrator.accel.x,sensor.getAccelX(),1000);
      additData.accel.y = calibrate(calibrator.accel.y,sensor.getAccelY(),1000);
      additData.accel.z = calibrate(calibrator.accel.z,sensor.getAccelZ(),1000);
      additData.gyro.x = calibrate(calibrator.gyro.x,sensor.getGyroX(),100);
      additData.gyro.y = calibrate(calibrator.gyro.y,sensor.getGyroY(),100);
      additData.gyro.z = calibrate(calibrator.gyro.z,sensor.getGyroZ(),100);
  
      additData.gtemp = calibrate(calibrator.gtemp,sensor.getTemperature(),100);
      
      check.gyro.last=millis();
  }
  else if (millis()-check.gyro.last>check.gyro.timeout){
    accelConnect();
  }
  
  if(check.AHT.OK){
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);
    allData.AHT_temp = temp.temperature*100;
    allData.AHT_hum = humidity.relative_humidity*100;
  }
  else if (millis()-check.AHT.last>check.AHT.timeout){
    ahtConnect();
  }
  
  if(check.BMP.OK){
    allData.BMP_temp = bmp280.getTemperature()*100;
    allData.BMP_pres = bmp280.getPressure()/10;
  }
  else if (millis()-check.BMP.last>check.BMP.timeout){
    bmpConnect();
  }
  
  if(check.mag.OK){
      sensors_event_t event; 
      if (mag.getEvent(&event)) {
        additData.mag.x=calibrate(calibrator.mag.x,event.magnetic.x,100);
        additData.mag.y=calibrate(calibrator.mag.y,event.magnetic.y,100);
        additData.mag.z=calibrate(calibrator.mag.z,event.magnetic.z,100);}
      else {
        check.mag.OK = false;
      }
      check.mag.last=millis();
  }
  else if (millis()-check.mag.last>check.mag.timeout){
    magnConnect();
  }

  additData.volt = calibrate(calibrator.volt,analogRead(TEST_PIN),1000);
  
  allData.now=millis();
  additData.now=millis();

  if(millis()-check.termo.last>check.termo.timeout){
    check.termo.last=millis();
    if (additData.gtemp < (check.termo.targetTemp-check.termo.tempThreshold)*100 && additData.volt > (check.termo.minVoltage+check.termo.voltThreshold)*1000 && check.gyro.OK) { 
      check.termo.isOn=true;
      digitalWrite(HOT_PIN,HIGH);
    } else if((additData.gtemp > check.termo.targetTemp *100 || additData.volt < check.termo.minVoltage*1000) || !check.gyro.OK){
      check.termo.isOn=false;
      digitalWrite(HOT_PIN,LOW);
    }      
    additData.flags=changeBit(additData.flags,check.termo.flagpos,check.termo.isOn);
  }
  
  if(millis()-check.cam.last>check.cam.timeout){
    check.cam.last=millis();
    if(check.cam.isOn==true && millis()-check.cam.lastSwitch>check.cam.workTime){
      check.cam.isOn=false;
      check.cam.lastSwitch=millis();
    }
    else{
      if ((additData.gtemp < (check.cam.targetTemp-check.cam.tempThreshold)*100 || !check.gyro.OK) && additData.volt > (check.cam.minVoltage+check.cam.voltThreshold)*1000 && check.cam.isOn==false && (millis()-check.cam.lastSwitch>check.cam.sleepTime)) { 
        check.cam.isOn=true;
        check.cam.lastSwitch=millis();
      } else if((additData.volt < check.cam.minVoltage*1000 || (additData.gtemp > (check.cam.targetTemp)*100 && check.gyro.OK)) && check.cam.isOn){
        check.cam.isOn=false;
        check.cam.lastSwitch=millis();
      }
    }
    digitalWrite(CAM_PIN, check.cam.isOn);
    additData.flags=changeBit(additData.flags,check.cam.flagpos,check.cam.isOn);
  }

      
  dataToCsv(allData,additData,row,sizeof(row));
  
  if(check.SD.OK){
    File file = SD.open("/allData.csv", FILE_APPEND);
    if (file) {
      check.SD.last=millis();
      file.print(row);
      file.close();
      check.SD.last=millis();
    }else{
      check.SD.OK=false;
      additData.flags=changeBit(additData.flags,check.SD.flagpos,check.SD.OK);
    }
  }else sdConnect();
  
  dataToJson(allData,additData,row,sizeof(row));
  Serial.print(row);
  
  if (millis() - check.SMS.last> check.SMS.timeout) {
    sendSMS(Serial2);
  }
  
  if(check.LoRa.OK && check.LoRa.busy){
    checkSent();}
  if(millis() - check.LoRa.last > check.LoRa.timeout){
    LoRaCheck();
    if(check.LoRa.busy){
      check.LoRa.OK=false;}
    if (check.LoRa.OK) {
      
      if(additSender.quenue>=additSender.maxQuenue)
      {
        sciSender.quenue++;
        additSender.quenue=0;
        generatePayload(additSender);
        check.LoRa.timeout=additSender.timeout;
        Serial.println();
        LoRa.beginPacket();
        LoRa.write(additSender.payload, additSender.par+additSender.dataLen+1);
        
      for(int i=0;i<additSender.dataLen + additSender.par+1;i++){
      Serial.printf("%02X", additSender.payload[i]);}
        LoRa.endPacket(true);
      }else if(sciSender.quenue>=sciSender.maxQuenue)
      {
        additSender.quenue++;
        sciSender.quenue=0;
        generatePayload(sciSender);
        check.LoRa.timeout=sciSender.timeout;
        
        Serial.println();
        
        LoRa.beginPacket();
        LoRa.write(sciSender.payload, sciSender.par+sciSender.dataLen+1);
        LoRa.endPacket(true); 
        
      for(int i=0;i<sciSender.dataLen + sciSender.par+1;i++){
      Serial.printf("%02X", sciSender.payload[i]);}
      }
      
      check.LoRa.busy=true;
      
    } else {
      LoRaConnect();
    }
    check.LoRa.last=millis();
  }
  
  
  esp_task_wdt_reset();
}
