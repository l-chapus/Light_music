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

#define FFT_SIZE 512 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothA2DPSink a2dp_sink; // Instance de BluetoothA2DPSink pour la réception audio
ArduinoFFT<float> FFT = ArduinoFFT<float>(); // Utilisation de ArduinoFFT pour les calculs FFT

float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
static uint8_t frame = 0;

// Broches LoRa pour TTGO T-Beam v1
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  23
#define LORA_IRQ  26

#define BUTTON_PIN 38
volatile uint8_t bouton_compteur = 0;
bool lastButtonState = HIGH;
volatile bool fft_ready = false;
#define AMP_MAX 44000 // Amplitude max attendue pour la normalisation FFT

void audio_data_callback(const uint8_t *data, uint32_t len) {
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
  
  FFT.windowing(vReal, FFT_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);

  fft_ready = true;
}

void setup() {
  Serial.begin(115200);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ); // Important !

  //LoRa.setSpreadingFactor(7);    // Valeur minimale (7 = plus rapide, 12 = plus lent)
  //LoRa.setSignalBandwidth(250E3); // 250 kHz = plus rapide, 125 kHz = défaut
  //LoRa.setCodingRate4(5);        // 4/5 = plus rapide, 4/8 = plus lent

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

  Serial.println("Bluetooth prêt !");

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  pinMode(BUTTON_PIN, INPUT_PULLUP); // GPIO 38 en entrée avec pull-up

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Compteur bouton: ");
  display.println(bouton_compteur);
  display.display();
}

uint8_t get_battery_level(uint8_t ID) {
  uint8_t mode = 153;
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&ID, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&mode, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  LoRa.endPacket();

  long int start = millis();
  uint8_t level = 0;

  while(millis() - start < 2000) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      uint8_t level = LoRa.read();
      Serial.print("Niveau de batterie reçu: ");
      Serial.print(level);
      Serial.println("%");
      break;
    }
  }

  return level;
}

void fonction_test() {
  //get_battery_level(1);
  //delay(1000);

  // Fonction vide pour test
  uint8_t ID = 1;
  uint8_t mode = 139;
  uint8_t R = 80;
  uint8_t G = 10;
  uint8_t B = 150;
  uint8_t vitesse = 20;
  uint8_t sens = 20;
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&ID, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&mode, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&R, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&G, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&B, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&vitesse, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&sens, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
 
  LoRa.endPacket();

  //delay(5000);
  //sens = 0;
  //R = 200;
  //LoRa.beginPacket();
  //LoRa.write((uint8_t*)&ID, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  //LoRa.write((uint8_t*)&mode, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  //LoRa.write((uint8_t*)&R, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  //LoRa.write((uint8_t*)&G, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  //LoRa.write((uint8_t*)&B, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  //LoRa.write((uint8_t*)&vitesse, sizeof(uint8_t)); // Envoie l'entier sur 2 octets
  //LoRa.write((uint8_t*)&sens, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
  //LoRa.endPacket();
}

void loop() {
  static unsigned long last_fft_send = 0;

  // Gestion du bouton
  bool buttonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == HIGH && buttonState == LOW) { // Front descendant
    bouton_compteur++;
    Serial.print("Bouton appuyé, compteur = ");
    Serial.println(bouton_compteur);
    // Affichage sur OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Compteur bouton: ");
    display.println(bouton_compteur);
    display.display();
    delay(200); // Anti-rebond simple

    fonction_test(); // Test d'envoi

    // Envoi via LoRa : entier au début
    //LoRa.beginPacket();
    //LoRa.write((uint8_t*)&bouton_compteur, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
    //LoRa.endPacket();
    Serial.println("Compteur envoyé via LoRa");
  }
  lastButtonState = buttonState;

  // Envoi FFT si prêt et 100 ms écoulées
  if (fft_ready && (millis() - last_fft_send >= 10)) {
    fft_ready = false;
    Serial.println("last_fft_send est de : " + String(millis() - last_fft_send));
    last_fft_send = millis();
  
    // ENVOI LoRa : compteur (mode) + vReal[]
    uint8_t val = 0;
    float val_f = 0;
    uint8_t ID = 1;
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&ID, sizeof(bouton_compteur)); // Envoie l'entier sur 2 octets
    LoRa.write((uint8_t*)&bouton_compteur, sizeof(bouton_compteur)); // 1 octet

    for (uint8_t i = 0; i < 16; i++) {
      val_f = vReal[i];
      if (val_f < 0) val_f = 0; // Juste au cas où
      if (val_f > AMP_MAX) val_f = AMP_MAX; // amplitude max attendue
      val = (uint8_t)(val_f * 255.0 / AMP_MAX); // Normalisation sur 8 bits
      LoRa.write((uint8_t*)&val, sizeof(val)); // 1 octet par valeur
    }
    Serial.println("----------------");

    LoRa.endPacket();
  }
}

