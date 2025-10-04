/*
  ESP32-C3 Insole Logger — 16ch MUX + microSD (SPI)
  --------------------------------------------------
  - Reads 16 channels via CD74HC4067 (single MUX)
  - Logs CSV to microSD at fixed rate (default 100 Hz)
  - CSV format: time_ms,ch0,...,ch15
  - Stable ADC read: EN toggle, settle, dummy read, small averaging
  - ADC: 12-bit, 11 dB attenuation (~0..3.6V)

  Wiring (pull-up topology, common=GND):
    ESP 3.3V ──[ Rpullup ≈ 33kΩ ]──●── MUX COM(SIG) ──(CHn)── Sensor ── GND
                                   │
                                   └── ESP32-C3 ADC pin
    * MUX VCC = 3.3 V, GND = common
    * EN active-LOW (tie LOW or drive from GPIO)
    * Add 0.1uF decoupling at MUX VCC–GND (module usually has it)
    * (Optional) ADC RC: COM—[1k~4.7k]—ADC, ADC—[10~47nF]—GND

  microSD (SPI):
    microSD CS → SD_CS (GPIO you choose)
    microSD SCK → SD_SCK
    microSD MOSI → SD_MOSI
    microSD MISO → SD_MISO
    VCC=3.3V, GND=common

  Notes:
    - Edit the GPIO numbers below to match your wiring.
    - Generates unique file name: /LOG000.CSV .. /LOG999.CSV
*/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// ===================== Direction Selection =========================== 
// ===== SELECT EXACTLY ONE =====
#define FOOT_RIGHT
// #define FOOT_LEFT

#if defined(FOOT_RIGHT) == defined(FOOT_LEFT)
#error "Select exactly one: define FOOT_RIGHT OR FOOT_LEFT (uncomment ONE line at the top)."
#endif

// ===================== User Pin Map (EDIT THESE) =====================
// CD74HC4067
#define PIN_S0      1
#define PIN_S1      2
#define PIN_S2      3
#define PIN_S3      10
#define PIN_EN      8      // /EN (active-LOW); set USE_EN=0 if tied to GND
#define USE_EN      0

// ADC (ESP32-C3 ADC1 pin; adjust per board)
#define ADC_PIN     0      // e.g., GPIO1 (A1)

// microSD SPI pins (choose free GPIOs; ESP32-C3 pin matrix allows remap)
#define SD_SCK      4     // EXAMPLE pins — change to your mapping
#define SD_MISO     5
#define SD_MOSI     6     // if this conflicts with PIN_EN, change one of them
#define SD_CS       7     // your CS (SS) pin for the SD module

// ========================= Logger settings ==========================
#define BAUD                115200
#define ADC_BITS            12
#define ADC_SETTLE_US       120     // after EN low & address set
#define ADC_DUMMY_WAIT_US   80
#define ADC_AVG_SAMPLES     4
#define PRINT_HEADER_SERIAL 1       // also print header to Serial once
#define LOG_HZ              100     // 100 Hz CSV
#define FAKE_DATA           0       // 1: random data (no MUX/ADC needed)

// ===================================================================
File logFile;
uint32_t sample_interval_us = (1000000UL / LOG_HZ);

static inline void setAddress(uint8_t ch){
  digitalWrite(PIN_S0, (ch >> 0) & 1);
  digitalWrite(PIN_S1, (ch >> 1) & 1);
  digitalWrite(PIN_S2, (ch >> 2) & 1);
  digitalWrite(PIN_S3, (ch >> 3) & 1);
}

static inline void muxEnable(bool on){
#if USE_EN
  // Active-LOW
  digitalWrite(PIN_EN, on ? LOW : HIGH);
#else
  (void)on;
#endif
}

static inline int readStableADC(){
  delayMicroseconds(ADC_DUMMY_WAIT_US);
  (void)analogRead(ADC_PIN);   // dummy read to clear previous S/H
  delayMicroseconds(ADC_DUMMY_WAIT_US);
  int acc = 0;
  for(int i=0;i<ADC_AVG_SAMPLES;i++){
    acc += analogRead(ADC_PIN);
    delayMicroseconds(40);
  }
  return acc / ADC_AVG_SAMPLES;
}

static inline int readChannel(uint8_t ch){
  muxEnable(false);
  setAddress(ch);
  delayMicroseconds(20);
  muxEnable(true);
  delayMicroseconds(ADC_SETTLE_US);
  return readStableADC();
}

#if defined(FOOT_RIGHT)
String nextLogFilename(){
  for(int i=0;i<=999;i++){
    char name[16];
    sprintf(name, "/LOG%03d_R.CSV", i);
    if(!SD.exists(name)) return String(name);
  }
  return String("/LOG999_R.CSV");
}
#elif defined(FOOT_LEFT)
String nextLogFilename(){
  for(int i=0;i<=999;i++){
    char name[16];
    sprintf(name, "/LOG%03d_L.CSV", i);
    if(!SD.exists(name)) return String(name);
  }
  return String("/LOG999_L.CSV");
}
#endif

bool initSD(){
  // Initialize SPI with custom pins
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  // Try 20 MHz; lower if unstable
  if(!SD.begin(SD_CS, SPI, 20000000)){
    Serial.println("[SD] init failed. Check wiring & CS pin.");
    return false;
  }
  Serial.println("[SD] init OK.");
  return true;
}

bool openLogFile(){
  String fn = nextLogFilename();
  logFile = SD.open(fn.c_str(), FILE_WRITE);
  if(!logFile){
    Serial.println("[SD] open log failed.");
    return false;
  }
  Serial.print("[SD] logging to "); Serial.println(fn);

  // header
  logFile.print("time_ms");
  for(int i=0;i<16;i++){
    logFile.print(",ch"); logFile.print(i);
  }
  logFile.println();
  logFile.flush();

#if PRINT_HEADER_SERIAL
  Serial.println("time_ms,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15");
#endif
  return true;
}

void setup(){
  Serial.begin(BAUD);
  delay(50);

#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(ADC_BITS);
  analogSetPinAttenuation(ADC_PIN, ADC_11db); // ~0..3.6 V
#endif

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
#if USE_EN
  pinMode(PIN_EN, OUTPUT);
  muxEnable(false);
#endif

  // --- SD ---
  if(!initSD()){
    // Keep running and print to Serial only
    Serial.println("[WARN] SD disabled. Will print CSV to Serial only.");
  } else {
    openLogFile();
  }
}

void loop(){
  static uint32_t t0 = micros();
  const uint32_t now = micros();
  if((now - t0) < sample_interval_us) return;  // simple fixed-rate loop
  t0 += sample_interval_us;

  uint32_t ms = millis();

#if FAKE_DATA
  // Generate synthetic values
  char line[256];
  int n = snprintf(line, sizeof(line), "%lu", (unsigned long)ms);
  for(int ch=0; ch<16; ch++){
    int v = (ch < 8) ? 4095 : random(300, 3500);
    n += snprintf(line+n, sizeof(line)-n, ",%d", v);
  }
  line[n++] = '\n'; line[n] = 0;
#else
  // Read 16 channels
  int vals[16];
  for(uint8_t ch=0; ch<16; ch++){
    vals[ch] = readChannel(ch);
  }
  char line[256];
  int n = snprintf(line, sizeof(line), "%lu", (unsigned long)ms);
  for(int ch=0; ch<16; ch++){
    n += snprintf(line+n, sizeof(line)-n, ",%d", vals[ch]);
  }
  line[n++] = '\n'; line[n] = 0;
#endif

  // Write to SD if available
  if(logFile){
    logFile.write((const uint8_t*)line, strlen(line));
    // Flush occasionally to reduce wear but keep data safe
    static uint32_t lastFlush = 0;
    if((ms - lastFlush) > 500){ // flush every 0.5s
      logFile.flush();
      lastFlush = ms;
    }
  }

  // Optional Serial mirror for debugging
  Serial.write((const uint8_t*)line, strlen(line));
}
