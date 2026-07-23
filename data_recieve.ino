#include <SPI.h>
#include <LoRa.h>
#include <BluetoothSerial.h>
#include <RS-FEC.h>

// Pins for the Receiver
#define CS    4
#define CR    4
#define RST   15
#define DIO0  2

#define SF 12
#define BW 62500.0

// Регистры SX1276 для FEI
#define REG_FEI_MSB        0x28
#define REG_FEI_MID        0x29
#define REG_FEI_LSB        0x2A
#define REG_FRF_MSB        0x06
#define REG_FRF_MID        0x07
#define REG_FRF_LSB        0x08
#define REG_MODEM_CONFIG3  0x26

// Базовая частота (433 МГц)
#define TX_FREQ 432.998840

uint32_t currentFreq = 0;
BluetoothSerial SerialBT;

// Internal 3D structures for integers
struct int16_t3d {
  int16_t x, y, z;
};

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

struct __attribute__((packed)) ReceiveData {
  int rssi;
  float snr;
  int errors;
  bool malformed;
  float correction;
  float freq;
};

template<typename T, uint8_t parity>
struct receiver {
  T *sentData;
  uint8_t dataLen;
  uint8_t reg;
  RS::ReedSolomon<sizeof(T), parity> rs; 
  uint8_t par;

  receiver(T *data, uint8_t r) : 
    sentData(data),
    dataLen(sizeof(T)),
    reg(r),
    par(parity) {}
};

const char* scientific_labels[] = {
  "now[ms]", "AHT_temp[C]", "AHT_hum",
  "BMP_temp[C]", "BMP_pres",
  "pm1_0", "pm2_5", "pm10_0",
  "p03um", "p05um", "p10um",
  "lat", "lon", "altitude"
};

const char* technical_labels[] = {
  "now[ms]",
  "gx", "gy", "gz",
  "ax[m/s2]", "ay[m/s2]", "az[m/s2]",
  "gtemp[C]",
  "magx[uT]", "magy[uT]", "magz[uT]",
  "voltage", "2Grssi", "flags"
};

const char* radio_labels[] = {
  "rssi", "snr", "malformed", "errors", "freq-correction", "current_freq"
};

ReceiveData radioInfo;

ScientificData allData;
TechnicalData additData;

receiver<ScientificData,8> allReceive(&allData,0x01);
receiver<TechnicalData,8> additReceive(&additData,0x08);

char row[4096];

// ======== ПРОТОТИПЫ ========
uint8_t readRegister(uint8_t address);
void writeRegister(uint8_t address, uint8_t value);
void applyFrequencyCorrection();
void dataToJson(ScientificData d, char buffer[], ReceiveData d2);
void dataToJson(TechnicalData d, char buffer[], ReceiveData d2);
void generateErrorRow(char buffer[], const ReceiveData& rd, const uint8_t* payload, int actualSize);

template<typename T, uint8_t parity>
bool receive(uint8_t* buffer, receiver<T, parity> &Sender, ReceiveData &rec, int len);

// ======== РЕГИСТРЫ ========
uint8_t readRegister(uint8_t address) {
  digitalWrite(CS, LOW);
  SPI.transfer(address & 0x7F);
  uint8_t value = SPI.transfer(0);
  digitalWrite(CS, HIGH);
  return value;
}

void writeRegister(uint8_t address, uint8_t value) {
  digitalWrite(CS, LOW);
  SPI.transfer(address | 0x80);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}

// ======== ЧАСТОТНАЯ СИНХРОНИЗАЦИЯ ========
void applyFrequencyCorrection() {
  // Читаем FEI
  int32_t fei = readRegister(REG_FEI_MSB) << 16;
  fei |= readRegister(REG_FEI_MID) << 8;
  fei |= readRegister(REG_FEI_LSB);
  
  // Расширяем знак (20-битное число)
  if (fei & 0x80000) {
    fei |= 0xFFF00000;
  }
  
  // Переводим в Герцы
  radioInfo.correction = (fei * 32.0f * 1e6f) / (1 << 24);
  
  // Корректируем ТОЛЬКО если сигнал чистый (SNR > 0) и ошибка в разумных пределах
  float snr = LoRa.packetSnr();
  if ( abs(radioInfo.correction) < 500000.0f && abs(radioInfo.correction) > 100.0f) {
    // Новая частота = текущая - ошибка
    float newFreqMHz = radioInfo.freq- (radioInfo.correction / 1e6f)/60;
    
    // Ограничиваем диапазоном 432-434 МГц
//    if (newFreqMHz < 432.0) newFreqMHz = 432.0;
//    if (newFreqMHz > 434.0) newFreqMHz = 434.0;
    
    uint32_t newFreqReg = (uint32_t)((newFreqMHz * (1 << 19)) / 32.0f);
    
    // Применяем коррекцию
    currentFreq = newFreqReg;
    radioInfo.freq= newFreqMHz;
    writeRegister(REG_FRF_MSB, (currentFreq >> 16) & 0xFF);
    writeRegister(REG_FRF_MID, (currentFreq >> 8) & 0xFF);
    writeRegister(REG_FRF_LSB, currentFreq & 0xFF);
    
//    Serial.print("FEI: ");
//    Serial.print(radioInfo.correction, 0);
//    Serial.print(" Гц -> частота: ");
//    Serial.print(radioInfo.freq, 6);
//    Serial.println(" МГц");
  } else{
//    Serial.print("FEI слишком большой: ");
//    Serial.print(radioInfo.correction, 0);
//    Serial.print(" Гц, SNR=");
//    Serial.println(snr);
  }
  
}

// ======== ПРИЕМ ========
template<typename T, uint8_t parity>bool receive(uint8_t* buffer, receiver<T, parity> &Sender, ReceiveData &rec, int len) {
  if(buffer[0]!=Sender.reg || Sender.dataLen + Sender.par + 1 != len){
    rec.malformed=true;
    rec.errors=0;
    return false;
  }
  
  uint8_t inputWorkBuffer[sizeof(T) + parity];
  Sender.rs.Decode(&buffer[1], inputWorkBuffer);
  
  rec.errors=0;
  for (int i = 0; i < Sender.dataLen; i++) {
    if (inputWorkBuffer[i] != buffer[i+1]) {
      rec.errors++;
    }
  }
  rec.malformed = (rec.errors * 2 > Sender.par);
  memcpy(Sender.sentData, inputWorkBuffer, Sender.dataLen);
  return true;
}

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  SerialBT.begin("AuroraData", false); 

  LoRa.setPins(CS, RST, DIO0);

  if (!LoRa.begin((uint32_t)(TX_FREQ * 1e6))) {
    Serial.println("LoRa init failed!");
    while (true);
  }

  // Настройки LoRa
  LoRa.setSpreadingFactor(SF);
  LoRa.setSignalBandwidth(BW);
  LoRa.setCodingRate4(CR);
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
    
  // Сохраняем начальную частоту
  currentFreq = (uint32_t)((TX_FREQ * (1 << 19)) / 32.0f);
  radioInfo.freq = TX_FREQ;
  
  Serial.println("LoRa готов");
  Serial.print("Частота: ");
  Serial.print(TX_FREQ, 6);
  Serial.println(" МГц");
  Serial.println("Автоматическая частотная синхронизация включена");
}

// ======== LOOP ========
void loop() {
  int packetSize = LoRa.parsePacket();
  uint8_t buffer[255];
  memset(buffer, 0, 255);
  
  if (packetSize > 0) {
    int bytesRead = 0;
    while (LoRa.available() && bytesRead < 255) {
      uint8_t ch = LoRa.read();
      buffer[bytesRead++] = ch;
    }
    
    radioInfo.rssi = LoRa.packetRssi();
    radioInfo.snr = LoRa.packetSnr();
    
    
    bool flag = false;
    if(!flag){
      flag = receive(buffer, allReceive, radioInfo, bytesRead);
      if(flag) {
        dataToJson(allData, row, radioInfo);
        // Применяем коррекцию ПОСЛЕ приема
        applyFrequencyCorrection();
      }
    }
    if(!flag){
      flag = receive(buffer, additReceive, radioInfo, bytesRead);
      if(flag) {
        dataToJson(additData, row, radioInfo);
        // Применяем коррекцию ПОСЛЕ приема
        applyFrequencyCorrection();
      }
    }
    if(!flag){
      generateErrorRow(row, radioInfo, buffer, bytesRead);
    }
    
    SerialBT.println(row);
    Serial.println(row);
  }
}

// ======== JSON ФУНКЦИИ ========
void dataToJson(ScientificData d, char buffer[], ReceiveData d2) {
  snprintf(buffer, 2048,
    "{"
    "\"%s\":%lu,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%lu,"
    "\"%s\":%u,"
    "\"%s\":%u,"
    "\"%s\":%u,"
    "\"%s\":%u,"
    "\"%s\":%u,"
    "\"%s\":%u,"
    "\"%s\":%.9f,"
    "\"%s\":%.9f,"
    "\"%s\":%u,"
    "\"%s\":%d,"
    "\"%s\":%.2f,"
    "\"%s\":%d,"
    "\"%s\":%d,"
    "\"%s\":%.2f,"
    "\"%s\":%.5f"
    "}",
    scientific_labels[0],  d.now,
    scientific_labels[1],  (float)d.AHT_temp / 100.0, 
    scientific_labels[2],  (float)d.AHT_hum / 100.0,
    scientific_labels[3],  (float)d.BMP_temp / 100.0, 
    scientific_labels[4],  (unsigned long)d.BMP_pres * 10,
    scientific_labels[5],  d.pm1_0, 
    scientific_labels[6],  d.pm2_5, 
    scientific_labels[7],  d.pm10_0,
    scientific_labels[8],  d.p03um, 
    scientific_labels[9],  d.p05um, 
    scientific_labels[10], d.p10um,
    scientific_labels[11], d.lat, 
    scientific_labels[12], d.lon, 
    scientific_labels[13], d.altitude,
    radio_labels[0],       d2.rssi, 
    radio_labels[1],       d2.snr,
    radio_labels[2],       d2.malformed ? 1 : 0,
    radio_labels[3],       d2.errors,
    radio_labels[4],       d2.correction,
    radio_labels[5],       d2.freq
  );
}
void dataToJson(TechnicalData d, char buffer[], ReceiveData d2) {
  snprintf(buffer, 4096,
    "{"
    "\"%s\":%lu,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%.3f,"
    "\"%s\":%.3f,"
    "\"%s\":%.3f,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%.2f,"
    "\"%s\":%.3f,"
    "\"%s\":%u,"
    "\"%s\":%u,"
    "\"%s\":%d,"
    "\"%s\":%.2f,"
    "\"%s\":%d,"
    "\"%s\":%d,"    // <-- ЗДЕСЬ БЫЛА ОШИБКА: было %.2f, а должно быть %d
    "\"%s\":%.5f"
    "}",
    technical_labels[0],  d.now,
    technical_labels[1],  (float)d.gyro.x / 100.0, 
    technical_labels[2],  (float)d.gyro.y / 100.0, 
    technical_labels[3],  (float)d.gyro.z / 100.0,
    technical_labels[4],  (float)d.accel.x / 1000.0, 
    technical_labels[5],  (float)d.accel.y / 1000.0, 
    technical_labels[6],  (float)d.accel.z / 1000.0,
    technical_labels[7],  (float)d.gtemp / 100.0,
    technical_labels[8],  (float)d.mag.x / 100.0, 
    technical_labels[9],  (float)d.mag.y / 100.0, 
    technical_labels[10], (float)d.mag.z / 100.0,
    technical_labels[11], (float)d.volt / 1000.0,
    technical_labels[12], d.q2G, 
    technical_labels[13], d.flags,
    radio_labels[0],      d2.rssi, 
    radio_labels[1],      d2.snr,
    radio_labels[2],      d2.malformed ? 1 : 0,
    radio_labels[3],      d2.errors,
    radio_labels[4],      d2.correction,
    radio_labels[5],      d2.freq
  );
}

void generateErrorRow(char buffer[], const ReceiveData& rd, const uint8_t* payload, int actualSize) {
  char hexBuffer[256] = ""; 
  char hexByte[4];
  
  for (int i = 0; i < actualSize && i < 120; i++) {
    snprintf(hexByte, sizeof(hexByte), "%02X", payload[i]);
    strcat(hexBuffer, hexByte);
  }
  
  snprintf(buffer, 2048,
    "{\"malformed\":%s,\"len\":%d,\"rssi\":%d,\"snr\":%.2f,\"errors\":%d,\"raw\":\"%s\",\"%s\":%.2f,\"%s\":%.5f}",
    rd.malformed ? "true" : "false", actualSize, rd.rssi, rd.snr, rd.errors, hexBuffer, radio_labels[4], rd.correction, radio_labels[5], rd.freq);
}
