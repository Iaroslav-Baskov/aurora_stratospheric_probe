#include <SPI.h>
#include <LoRa.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

#define CS    4
#define RST   15
#define DIO0  2

BluetoothSerial SerialBT;

struct float3d{
  float x,y,z;};
  
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
  "lat","lon","altitude",
};
int normal_size;
SensorData data;
int rssi;
float snr;
const int numVars = sizeof(labels) / sizeof(labels[0]);
char row[2048];
void setup() {
  Serial.begin(115200);
  SerialBT.begin("AuroraData", true);

  LoRa.setPins(CS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed!");
    while (true);
  }
  normal_size=sizeof(SensorData)+2;
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(62.5E3);
  LoRa.setCodingRate4(8);

  Serial.println("LoRa receiver ready (no triplication)");
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
    if (payload.size() < normal_size) {
      generateErrorRow(row,sizeof(row),payload, normal_size,rssi,snr);
      SerialBT.println(row);
      Serial.println(row); 
      return;
    }
    if (payload[0] != 0xAA || payload[normal_size-1] != 0xBB) {
      generateErrorRow(row,sizeof(row),payload, normal_size,rssi,snr);
      SerialBT.println(row);
      Serial.println(row);
    return;}

    memcpy(&data, &payload[1], sizeof(SensorData));
    
    dataToJson(data,row,sizeof(row));
    SerialBT.println(row);
    
  }
 }
  String PayloadParse(std::vector<uint8_t> payload){
    String output;
    for(int i=0;i<payload.size();i++){
      output=String(output+"x"+String(payload[i],HEX));
    }
    return output;
  }
void dataToJson(SensorData data,char buffer[],int len){
  int i=0;
  snprintf(buffer, len,
    "{%s:%d,%s:%d,%s:%.2f,%s:%.2f,%s:%.2f,%s:%.0f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.3f,%s:%.4f,%s:%d,%s:%d,%s:%d,%s:%d,%s:%d,%s:%d,%s:%.9f,%s:%.9f,%s:%.0f}\n",
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
    "%d,%d,%.2f,%.2f,%.2f,%.0f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%d,%d,%d,%d,%d,%.9f,%.9f,%.0f\n",
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
void generateErrorRow(char buffer[],int len,std::vector<unsigned char> payload, int normal_size,int rssi,float snr){
  int i=0;
  snprintf(buffer, len,
    "{\"payload_length\":%d,\"normal_size\":%d,\"rssi\":%d,\"snr\":%.3f}",
    payload.size(),
    normal_size,
    rssi,
    snr);
}
