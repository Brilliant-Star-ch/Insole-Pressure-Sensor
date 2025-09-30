/*
  Insole Pressure - Dummy TX (ESP32-C3)
  - 16채널 더미(또는 실측) 값을 CSV로 100Hz 전송
  - 시리얼: 115200 bps
  - 포맷: time_ms,ch0,ch1,...,ch15

  하드웨어 연결(실측 시):
  - CD74HC4067: s0~s3 -> 아래 핀
  - SIG -> ADC_PIN (아날로그 입력)
  - VCC 3.3V, GND
*/

#include <WiFi.h>
#include <ArduinoOTA.h>
#define FAKE_DATA 1    // 1: 랜덤값 송신, 0: MUX+ADC 실측
#include <Arduino.h>


const char* ssid = "KT_GiGA_B6DA";
const char* pass = "f0gh02gj71";

// --- 핀 설정 (보드에 맞게 필요 시 수정) ---
const int PIN_S0 = 21;   // CD74HC4067 S0
const int PIN_S1 = 20;   // S1
const int PIN_S2 = 10;  // S2
const int PIN_S3 = 9;  // S3
const int ADC_PIN = 0;  // ESP32-C3 ADC 입력 핀 (GPIO0가 ADC 가능)

// --- 송신 주기(Hz) ---
const int SAMPLE_RATE_HZ = 100;
const uint32_t SAMPLE_INTERVAL_MS = 1000 / SAMPLE_RATE_HZ;

// --- 유틸 ---
void selectMuxChannel(uint8_t ch) {
  digitalWrite(PIN_S0, ch & 0x01);
  digitalWrite(PIN_S1, (ch >> 1) & 0x01);
  digitalWrite(PIN_S2, (ch >> 2) & 0x01);
  digitalWrite(PIN_S3, (ch >> 3) & 0x01);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi failed, rebooting in 5s...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname("c3-insole");
  ArduinoOTA.setPassword("123");
  ArduinoOTA.onStart([](){ Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([](){ Serial.println("[OTA] End"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t){
    Serial.printf("[OTA] %u%%\n", (p * 100) / t);
  });
  ArduinoOTA.onError([](ota_error_t e){
    Serial.printf("[OTA] Error %u\n", e);
  });
  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

#if !FAKE_DATA
  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(ADC_PIN, INPUT);
#endif

  // CSV 헤더 1줄
  Serial.println("time_ms,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15");
}

void loop() {
  ArduinoOTA.handle();
  delay(2);

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last < SAMPLE_INTERVAL_MS) return;
  last = now;

  // 한 줄 출력 시작
  Serial.print(now);
  Serial.print(',');

  for (int ch = 0; ch < 16; ch++) {
    int v;
#if FAKE_DATA
    // 더미: 0~4095 범위 랜덤 (ADC 12bit 스케일 가정)
    v = random(300, 3500);
#else
    selectMuxChannel(ch);
    delayMicroseconds(150);       // 채널 전환 안정화
    v = analogRead(ADC_PIN);      // 실측
#endif

    Serial.print(v);
    if (ch < 15) Serial.print(',');
  }
  Serial.println();


}
