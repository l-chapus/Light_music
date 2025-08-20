#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothA2DPSink.h"
#include <ArduinoFFT.h>
#include <LoRa.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define FFT_SIZE 32 // Plus rapide

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothA2DPSink a2dp_sink; // Instance de BluetoothA2DPSink pour la réception audio
ArduinoFFT<float> FFT = ArduinoFFT<float>(); // Utilisation de ArduinoFFT pour les calculs FFT

float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
static uint8_t frame = 0;

void audio_data_callback(const uint8_t *data, uint32_t len) {

  if (++frame % 2 != 0) return; // Affiche 1 fois sur 2

  int16_t *samples = (int16_t *)data;
  uint32_t sample_count = len / 4; // stéréo 16 bits

  // On prend FFT_SIZE échantillons du canal gauche
  for (uint16_t i = 0; i < FFT_SIZE; i++) {
    if (i < sample_count) {
      vReal[i] = samples[i * 2]; // Canal gauche
      vImag[i] = 0;
    } else {
      vReal[i] = 0;
      vImag[i] = 0;
    }
  }
  
  Serial.println("Frame: " + String(frame));

  FFT.windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);

  // Affichage des valeurs FFT dans la console
  Serial.print("vReal: ");
  for (uint8_t i = 0; i < FFT_SIZE / 2; i++) {
    Serial.print(vReal[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Affichage du spectre sur l'OLED
  display.clearDisplay();
  for (uint8_t i = 2; i < FFT_SIZE / 2; i++) { // On ignore les basses fréquences (DC)
    int barHeight = map((int)vReal[i], 0, 2000, 0, SCREEN_HEIGHT);
    int x = map(i, 2, FFT_SIZE / 2 - 1, 0, SCREEN_WIDTH - 1);
    display.drawLine(x, SCREEN_HEIGHT, x, SCREEN_HEIGHT - barHeight, SSD1306_WHITE);
  }
  display.display();

  Serial.println("-------------------");
}

// Broches LoRa pour TTGO T-Beam v1/v2
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  23
#define LORA_IRQ  26

void setup() {
  Serial.begin(115200);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); // Important !
  if (!LoRa.begin(433E6)) {
    Serial.println("Erreur init LoRa !");
    while (1);
  }
  Serial.println("LoRa prêt !");
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec de l'initialisation de l'écran OLED"));
    while (true);
  }

  a2dp_sink.start("T-BEAM audio");
  a2dp_sink.set_stream_reader(audio_data_callback);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  // test LoRa
  LoRa.beginPacket();
  LoRa.print("Hello LoRa!");
  LoRa.endPacket();
  Serial.println("Message LoRa envoyé");
  delay(5000); // Envoie un message toutes les 5 secondes
  //delay(50);
}