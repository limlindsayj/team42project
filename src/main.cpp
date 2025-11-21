#include <Arduino.h>
#include "driver/i2s.h"
#include <arduinoFFT.h>

#define I2S_WS   25    // LRCLK
#define I2S_SCK  26    // BCLK
#define I2S_SD   33    // DOUT

#define SAMPLE_RATE   16000
#define FFT_SIZE      1024     // must be power of 2!

arduinoFFT FFT = arduinoFFT();

double vReal[FFT_SIZE];
double vImag[FFT_SIZE];

void setup() {
  Serial.begin(9600);
  delay(500);

  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);

  Serial.println("Microphone FFT pitch detection started!");
}
String freqToNote(float freq) {
    if (freq <= 0) return "None";

    // MIDI note number
    int midi = round(69 + 12 * log(freq / 440.0) / log(2.0));

    // Note names
    const char* noteNames[] = {
        "C", "C#", "D", "D#", "E", "F",
        "F#", "G", "G#", "A", "A#", "B"
    };

    int noteIndex = midi % 12;
    int octave = (midi / 12) - 1;

    String note = String(noteNames[noteIndex]) + String(octave);
    return note;
}
void loop() {
  int32_t sample32;
  size_t bytes_read;

  // === Read samples from microphone ===
  for (int i = 0; i < FFT_SIZE; i++) {
    i2s_read(I2S_NUM_0, &sample32, sizeof(sample32), &bytes_read, portMAX_DELAY);

    // Convert microphone 32-bit PCM → float
    float s = sample32 / 2147483648.0f;  // normalize to -1..1

    vReal[i] = s;
    vImag[i] = 0.0;
  }

  // === Apply Hann window ===
  FFT.Windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

  // === FFT ===
  FFT.Compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, FFT_SIZE);

  // === Find the highest peak ===
  double peakFreq = FFT.MajorPeak(vReal, FFT_SIZE, SAMPLE_RATE);

  Serial.print("Pitch: ");
  Serial.print(peakFreq);
  Serial.print(" Hz ");
  Serial.println(freqToNote(peakFreq));

  delay(50);
}