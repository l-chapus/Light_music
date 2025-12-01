#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothA2DPSink.h"
#include <ArduinoFFT.h>
#include <LoRa.h>
#include "BluetoothSerial.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define FFT_SIZE 512 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BluetoothSerial SerialBT; // Bluetooth SPP pour commandes
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

  if (!LoRa.begin(433E6)) {
    Serial.println("Erreur init LoRa !");
    while (1);
  }
  Serial.println("LoRa prêt !");
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec de l'initialisation de l'écran OLED"));
    while (true);
  }

  // Démarrage du Bluetooth SPP pour recevoir des commandes (remplace A2DP)
  if (!SerialBT.begin("T-BEAM-CMD")) {
    Serial.println("Erreur init Bluetooth SPP !");
  } else {
    Serial.println("Bluetooth SPP prêt !");
  }
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

uint8_t getBatteryLevel(uint8_t ID, bool resetMode) {
  uint8_t mode = 204;
  uint8_t nbArgument = 0;
  if (resetMode) mode = 203; // Mode reset
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&ID, sizeof(ID)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&mode, sizeof(mode)); // Envoie l'entier sur 2 octets
  LoRa.write((uint8_t*)&nbArgument, sizeof(nbArgument)); // Envoie l'entier sur 2 octets
  LoRa.endPacket();

  long int start = millis();
  uint8_t level = 0;

  while(millis() - start < 2000) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      level = LoRa.read();
      Serial.print("Niveau de batterie reçu: ");
      Serial.print(level);
      Serial.println("%");
      break;
    }
  }
  return level;
}

void sendDataLora(uint8_t ID, uint8_t mode, uint8_t* data, uint8_t length) {
  Serial.print("Mode envoyé: ");Serial.println(mode);  
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&ID, sizeof(ID)); // Envoie l'entier sur 1 octets
  LoRa.write((uint8_t*)&mode, sizeof(mode)); // Envoie l'entier sur 1 octets
  LoRa.write((uint8_t*)&length, sizeof(length)); // Envoie l'entier sur 1 octets
  LoRa.write(data, length); // écrit tout le buffer en une fois
  LoRa.endPacket();
}

void fonction_test() {
  //getBatteryLevel(1, true);
  //delay(1000);

  // Fonction vide pour test
  uint8_t ID = 1;
  uint8_t mode = 107;
  uint8_t R = 10;
  uint8_t G = 80;
  uint8_t B = 200;
  uint8_t vitesse = 20;
  uint8_t sens = 20;
  uint8_t data[5] = {R, G, B, vitesse, sens};  
  sendDataLora(ID, mode, data, 5);
  delay(500);
  sendDataLora(ID, ++mode, data, 5);
  delay(100);
}

// Fonction de traitement des commandes reçues via Bluetooth SPP
void handleCommand(const String &cmd) {
  // Convertit une commande "n1;n2;n3;..." en bytes uint8_t et envoie via LoRa.
  // Format attendu : ID;MODE;PARAM1;PARAM2;...
  Serial.print("Cmd reçu: "); Serial.println(cmd);

  // Copie dans un buffer modifiable
  char buf[128];
  cmd.toCharArray(buf, sizeof(buf));

  const uint8_t MAX_PARAMS = 10;
  uint8_t params[MAX_PARAMS];
  uint8_t count = 0;

  char *token = strtok(buf, ";");
  while (token != NULL && count < MAX_PARAMS) {
    long v = atol(token);
    if (v < 0) v = 0;
    if (v > 255) v = 255; // clamp sur 0..255
    params[count++] = (uint8_t)v;
    token = strtok(NULL, ";");
  }

  if (count == 0) {
    Serial.println("Aucun nombre trouve dans la commande.");
    SerialBT.println("ERR:EMPTY");
    return;
  }

  // ID = premier octet, MODE = deuxième octet (si présent), data = reste
  uint8_t ID = params[0];
  uint8_t mode = 0;
  uint8_t *data = NULL;
  uint8_t datalen = 0;

  if (count >= 2) {
    mode = params[1];
    if (count > 2) {
      data = &params[2];
      datalen = count - 2;
    }
  } else {
    // si seul ID fourni, on envoie sans data avec mode = 0
    mode = 0;
    datalen = 0;
  }

  // Affiche les valeurs converties pour debug
  Serial.print("Parsed bytes: ");
  for (uint8_t i = 0; i < count; i++) {
    Serial.print(params[i]);
    if (i + 1 < count) Serial.print(", ");
  }
  Serial.println("Longueur des données envoyées : ");Serial.print(datalen);
  Serial.println();

  // Envoi via LoRa
  sendDataLora(ID, mode, data, datalen);
  SerialBT.println("OK");
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
  }
  lastButtonState = buttonState;

  // Lecture des commandes Bluetooth SPP
  if (SerialBT && SerialBT.available()) {
    String cmd = SerialBT.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) handleCommand(cmd);
  }

}