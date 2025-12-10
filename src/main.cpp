#include <Arduino.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "cloud_info.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define I2S_WS   25    // LRCLK
#define I2S_SCK  26    // BCLK
#define I2S_SD   33    // DOUT

#define SAMPLE_RATE   16000
#define FFT_SIZE      1024

arduinoFFT FFT = arduinoFFT();
LiquidCrystal_I2C lcd(0x27, 16, 2);

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

struct Song {
  const char** notes;
  int length;
  const char* name;
};

const char* song1Notes[] = { "C", "C#", "D" };
const char* song2Notes[] = { "A", "B", "C#" };

Song songs[] = {
  { song1Notes, sizeof(song1Notes) / sizeof(song1Notes[0]), "Song 1" },
  { song2Notes, sizeof(song2Notes) / sizeof(song2Notes[0]), "Song 2" }
};

const int NUM_SONGS = sizeof(songs) / sizeof(songs[0]);

int currentNoteIndex = 0;        // which note we're asking for
int correctStreakCount = 0;      // how many cycles in a row we've detected it
const int REQUIRED_STREAK = 1;   // will adjust later
bool songCompleted = false;      // flag to indicate song completion
bool songStarted = false;        // flag to indicate song selection completion

Song currentSong = songs[0];     // song that's been selected

String lastLine1 = ""; // stores previous values printed to LCD
String lastLine2 = ""; // to avoid redundant LCD updates that cause flickering

void printToLCD(const String& line) { // one-line helper
  if (lastLine1 == line && lastLine2 == "") {
    return; // No need to update
  }
  lastLine1 = line;
  lastLine2 = "";
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line);
}

void printToLCD(const String& line1, const String& line2) {
  if (lastLine1 == line1 && lastLine2 == line2) {
    return; // No need to update
  }
  lastLine1 = line1;
  lastLine2 = line2;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    if (songStarted) {
      return; // Ignore further writes once the song has started
    }
    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) {
      return;
    }
    int songIndex = atoi(value.c_str());
    if (songIndex >= 0 && songIndex < NUM_SONGS) {
      Serial.println("Song selection received: " + String(songIndex));
      currentSong = songs[songIndex];
      songStarted = true;
    } else {
      Serial.println("Invalid song index received");
      printToLCD("Invalid song index", "Try again");
    }
  }
};

void setupBLE() {
  BLEDevice::init("NoteTutor");

  BLEServer *pServer   = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *selectedSongCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // helps with iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  selectedSongCharacteristic->setCallbacks(new MyCallbacks());
  Serial.println("BLE Initialized");
}

// ---------------------------------------------------------------------------
// I2S (microphone) setup
// ---------------------------------------------------------------------------

void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// ---------------------------------------------------------------------------
// Cloud (TLS/HTTP) globals and helpers
// ---------------------------------------------------------------------------

WiFiClientSecure secureClient;
HTTPClient http;

unsigned long lastSendMs = 0;
const unsigned long SEND_INTERVAL_MS = 1000;  // send at most once per second

String lastSentNote = "";

// Configure TLS / HTTP once
void setupCloud() {
  secureClient.setCACert(root_ca);
  secureClient.setTimeout(5000);            // 5s timeout is fine for telemetry

  http.setReuse(true);                      // allow connection reuse if possible
}

// Send one telemetry message to the cloud
void sendTelemetry(const String &note) {
  StaticJsonDocument<128> doc;
  doc["note"] = note;

  char buffer[128];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));

  if (!http.begin(secureClient, url)) {   // `url` comes from cloud_info.h
    Serial.println("HTTP begin failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", SAS_TOKEN);

  int httpCode = http.POST((uint8_t*)buffer, len);

  if (httpCode == 204) { // IoT Hub returns 204 No Content for successful telemetry
    Serial.print("Information sent: ");
    Serial.println(buffer);
  } else {
    Serial.print("Failed to send information. HTTP code: ");
    Serial.println(httpCode);
  }
  http.end();
}

String freqToNote(float freq) {
  if (freq <= 0) return "None";

  int midi = round(69 + 12 * log(freq / 440.0) / log(2.0));

  const char* noteNames[] = {
    "C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"
  };

  int noteIndex = midi % 12;
  int octave = (midi / 12) - 1;

  String note = String(noteNames[noteIndex]);
  return note;
}

void setup() {
  Serial.begin(9600);
  delay(500);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  printToLCD("Booting...", "");

  // FFT object already constructed globally

  // Microphone / I2S
  setupI2S();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Cloud client (TLS/HTTP)
  setupCloud();

  // BLE
  setupBLE(); // Set up the BLE server

  Serial.println("Microphone FFT pitch detection started!");

  printToLCD("Select song:", "0: TTLS, 1: ???");

  // Wait for song selection via BLE
  while (!songStarted) {
    delay(10);
  }

  printToLCD("Great choice!", "Song starting...");
  delay(2000);
}

void loop() {

  if (songCompleted) {
    printToLCD("Song complete!", "Congrats :)");
    delay(10000);
    return;
  }

  int32_t sample32;
  size_t bytes_read;

  // --- Collect samples for FFT ---
  for (int i = 0; i < FFT_SIZE; i++) {
    i2s_read(I2S_NUM_0, &sample32, sizeof(sample32), &bytes_read, portMAX_DELAY);

    float s = sample32 / 2147483648.0f;

    vReal[i] = s;
    vImag[i] = 0.0;
  }

  // --- Run FFT ---
  FFT.Windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFT_SIZE);

  double peakFreq = FFT.MajorPeak(vReal, FFT_SIZE, SAMPLE_RATE);

  const char* targetNote = currentSong.notes[currentNoteIndex];
  String detectedNote = "--";

  if (peakFreq <= 0 || isnan(peakFreq) || isinf(peakFreq)) {
    // Silence or invalid reading: reset streak
    correctStreakCount = 0;
    Serial.println("Silence or invalid peak");
  } else { // Valid frequency detected
    // Convert detected frequency to a note name
    detectedNote = freqToNote(peakFreq);

    // --- Check if the detected note matches the target ---
    if (detectedNote == targetNote) {
      correctStreakCount++;

      if (correctStreakCount >= REQUIRED_STREAK) {
        // We've seen the correct note for REQUIRED_STREAK cycles in a row
        currentNoteIndex++;
        if (currentNoteIndex >= currentSong.length) {
          songCompleted = true;
          return;
        }

        correctStreakCount = 0;  // reset streak for the next note
      }
    } else {
      // Wrong note, reset streak
      correctStreakCount = 0;
    }
  }

  // --- Update LCD with target + detected note ---
  printToLCD(String("Play: ") + currentSong.notes[currentNoteIndex], String("You: ") + detectedNote);

  // Debug serial output
  Serial.print("Freq: ");
  Serial.print(peakFreq);
  Serial.print(" Hz  Detected: ");
  Serial.print(detectedNote);
  Serial.print("  Target: ");
  Serial.print(currentSong.notes[currentNoteIndex]);
  Serial.print("  Streak: ");
  Serial.println(correctStreakCount);

  // Cloud telemetry
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    if (detectedNote != lastSentNote) {
      sendTelemetry(detectedNote);
      lastSentNote = detectedNote;
    }
  }

  // Small delay
  delay(10);
}
