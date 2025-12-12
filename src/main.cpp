#include <Arduino.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include "cloud_info.h"
#include "esp_heap_caps.h"
#include "SPIFFS.h"

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

#define I2S_WS   25    // LRCLK
#define I2S_SCK  26    // BCLK
#define I2S_SD   33    // DOUT
#define GREEN_LED_PIN  27
#define RED_LED_PIN    15

#define SAMPLE_RATE   16000
#define FFT_SIZE      1024

int fileCounter = 0;

arduinoFFT FFT = arduinoFFT();
LiquidCrystal_I2C lcd(0x27, 16, 2);

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

struct Song {
  const char** notes;
  int length;
  const char* name;
};

const char* song1Notes[] = { "C", "C", "G", "G", "A", "A", "G", "F", "F", "E", "E", "D", "D", "C"};
const char* song2Notes[] = { "G", "C", "B", "C", "E", "D", "C", "D", "E", "D", "C", "A", "A", "G", "C"};

Song songs[] = {
  { song1Notes, sizeof(song1Notes) / sizeof(song1Notes[0]), "Twinke, Twinkle, Little Star" },
  { song2Notes, sizeof(song2Notes) / sizeof(song2Notes[0]), "Happy Birthday (Short)" }
};

const int NUM_SONGS = sizeof(songs) / sizeof(songs[0]);

int currentNoteIndex = 0;         // which note we're asking for
int correctStreakCount = 0;       // how many cycles in a row we've detected it
const int REQUIRED_STREAK = 10;   // how many loops we need to detect the note correctly before we consider it detected
bool songCompleted = false;       // flag to indicate song completion
bool songStarted = false;         // flag to indicate song selection completion

Song currentSong = songs[0];      // song that's been selected

String lastLine1 = ""; // stores previous values printed to LCD
String lastLine2 = ""; // to avoid redundant LCD updates that cause flickering

void dumpSPIFFS() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  Serial.println("\nDumping SPIFFS Files");

  while (file) {
    // Print file contents
    while (file.available()) {
      Serial.write(file.read());
    }
    Serial.println();

    file = root.openNextFile();
  }
}

void printToLCD(const String& line) {
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

int songIndex = -1; // selected song index

class MyCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
    if (songStarted) {
      return; // Ignore further writes once the song has started
    }

    std::string value = pCharacteristic->getValue();
    if (value.length() == 0) {
      return;
    }

    songIndex = atoi(value.c_str());
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
  NimBLEDevice::init("NoteTutor");

  NimBLEDevice::setPower(ESP_PWR_LVL_N0);
  NimBLEDevice::setMTU(23); 
  NimBLEDevice::setSecurityAuth(false, false, false);

  NimBLEServer *pServer   = NimBLEDevice::createServer();
  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  NimBLECharacteristic *selectedSongCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );

  selectedSongCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);

  NimBLEDevice::startAdvertising();
  Serial.println("BLE Initialized (NimBLE)");
}

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

// Cloud (TLS/HTTP) globals and helpers

WiFiClientSecure secureClient;
HTTPClient http;

unsigned long lastSendMs = 0;
const unsigned long SEND_INTERVAL_MS = 1000;  // send at most once per second

String lastSentNote = "";

// Configure TLS / HTTP once
void setupCloud() {
  secureClient.setCACert(root_ca);
  secureClient.setTimeout(5000);            // 5s timeout for telemetry

  http.setReuse(true);                      // allow connection reuse if possible
}

// Send one telemetry message to the cloud
void sendTelemetry(const String &note, const String &targetNote, const int song) {
  JsonDocument doc;
  doc["note"] = note;
  doc["targetNote"] = targetNote;
  doc["song"] = song;
  doc["noteAccurate"] = (note == targetNote);

  // Serialize to buffer
  char buffer[256];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));

  // Create unique filename
  fileCounter++;
  String filename = "/telemetry_" + String(fileCounter) + ".json";

  // Write JSON to uniquely named file
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to create file");
  } else {
    file.write((uint8_t*)buffer, len);
    file.close();
    Serial.print("Wrote telemetry to ");
    Serial.println(filename);
  }

  // HTTP logic
  if (!http.begin(secureClient, url)) {
    Serial.println("HTTP begin failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", SAS_TOKEN);

  int httpCode = http.POST((uint8_t*)buffer, len);

  if (httpCode == 204) {
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

  // LEDs
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  Serial.println("SPIFFS mounted successfully");

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

  setupCloud();

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

  static unsigned long feedbackUntil = 0;     // time until we hide "Correct!" or "Try again!"
  static bool showingFeedback = false;
  static unsigned long lastValidFreqTime = 0; // for stability checking
  static String lastStableNote = "";

  if (songCompleted) {
    printToLCD("Song complete!", "Congrats :)");
    delay(10000);
    return;
  }

  // If we are still showing “Correct!” or “Try again!” then do nothing else
  if (showingFeedback) {
    if (millis() < feedbackUntil) {
      return;
    } else {
      showingFeedback = false;
    }
  }

  int32_t sample32;
  size_t bytes_read;

  // Collect samples
  for (int i = 0; i < FFT_SIZE; i++) {
    i2s_read(I2S_NUM_0, &sample32, sizeof(sample32), &bytes_read, portMAX_DELAY);
    float s = sample32 / 2147483648.0f;
    vReal[i] = s;
    vImag[i] = 0.0;
  }

  // FFT processing
  FFT.Windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFT_SIZE);

  double peakFreq = FFT.MajorPeak(vReal, FFT_SIZE, SAMPLE_RATE);

  const char* targetNote = currentSong.notes[currentNoteIndex];
  String detectedNote = "--";

  // Handle silence (does NOT count as wrong)
  if (peakFreq <= 0 || isnan(peakFreq) || isinf(peakFreq)) {
    printToLCD(String("Play: ") + targetNote, "You: --");
    return; 
  }

  detectedNote = freqToNote(peakFreq);

  // Note stability checking
  if (detectedNote == lastStableNote) {
    // note is the same as previous frame → stable
  } else {
    // note changed → reset stability timer
    lastStableNote = detectedNote;
    lastValidFreqTime = millis();
  }

  // Require 300ms of stability before evaluating
  if (millis() - lastValidFreqTime < 300) {
    // Not stable long enough → wait
    printToLCD(String("Play: ") + targetNote, String("You: ") + detectedNote);
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);
    return;
  }

  // Evaluate note after 300ms stable

  if (detectedNote == targetNote) {

    // Note is correct
    currentNoteIndex++;

    printToLCD("Correct!", "");
    showingFeedback = true;
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    feedbackUntil = millis() + 1000;  // 1 second

    if (currentNoteIndex >= currentSong.length) {
      songCompleted = true;
      dumpSPIFFS(); // print all telemetry files to serial port
    }

    // Cloud telemetry
    sendTelemetry(detectedNote, targetNote, songIndex);

    return;
  }

  // Wrong (and stable)
  printToLCD("Try again!", "");
  showingFeedback = true;
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);
  feedbackUntil = millis() + 1000;   // 1 second
  
  // Cloud telemetry
  unsigned long now = millis();
  if (now - lastSendMs >= SEND_INTERVAL_MS) {
    lastSendMs = now;

    sendTelemetry(detectedNote, targetNote, songIndex);
  }

  return;
}
