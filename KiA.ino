/*
 * ChatGPT Client For ESP32 on Lilygo t-embed
 * Description: For HTTPS connection using WiFiClientSecure
 * Author: Tom Mueller based on ThatProject code
 * Date: 03-08-2023
 */

#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include "ChatGPT.hpp"
#include "pin_config.h"
#include "SD_MMC.h"
#include <OneButton.h>
#include <RotaryEncoder.h>
#include "TFT_eSPI.h"
#include "APA102.h"
#include "Audio.h" 
#include <LittleFS.h>
#include "Arduino.h"
#include "driver/i2s.h"
#include "es7210.h"


File myFile;

TFT_eSPI tft = TFT_eSPI(170, 320);

RotaryEncoder encoder(PIN_ENCODE_A, PIN_ENCODE_B, RotaryEncoder::LatchMode::TWO03);
OneButton button(PIN_ENCODE_BTN, true);
APA102<PIN_APA102_DI, PIN_APA102_CLK> ledStrip;
Audio *audio;

// static const char *ssid = "supertom";
// static const char *password = "Anamika&Peter";

const char *test_root_ca =
  "-----BEGIN CERTIFICATE-----\n"
  "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\n"
  "-----END CERTIFICATE-----\n";

bool buttonState = false;

WiFiClientSecure client;
ChatGPT<WiFiClientSecure> chat_gpt(&client, "v1", "<OpenAI_API_KEY>");

void setup() {
    pinMode(PIN_POWER_ON, OUTPUT);
  digitalWrite(PIN_POWER_ON, HIGH);
  Serial.begin(115200);
  Serial.print("Connecting to WiFi network: ");
  Serial.print(WIFI_SSID);
  Serial.println("'...");
   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting...");
    delay(500);
  }
  Serial.println("Connected!");
tft.begin();

  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  //For SSL Connection (HTTPS)
  client.setCACert(test_root_ca);

  String result;
 pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);
  //tft.println("Wifi connected");
  SD_init();
//tft.fillCircle(64, 64, 61, TFT_BLUE);

  Serial.println("[ChatGPT] Only print a content message");
  if (chat_gpt.simple_message("gpt-3.5-turbo-0301", "user", "Planning a 3-day trip to San Diego", result)) {
    Serial.println("===OK===");
    Serial.println(result);
  } else {
    Serial.println("===ERROR===");
    Serial.println(result);
  }

  Serial.println("\n\n[ChatGPT] Print full message(JSON Type)");
  if (chat_gpt.full_message("gpt-3.5-turbo", "user", "What is the OpenAI mission?", result)) {
    Serial.println("===OK===");
    Serial.println(result);
  } else {
    Serial.println("===ERROR===");
    Serial.println(result);
  }
}

void loop() {}
void SD_init(void) {
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, 1);
  SD_MMC.setPins(PIN_SD_SCK, PIN_SD_MOSI, PIN_SD_MISO);
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("Card Mount Failed");
    return;
  }

  File root = SD_MMC.open("/");
  File file = root.openNextFile();

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  delay(500);
}
