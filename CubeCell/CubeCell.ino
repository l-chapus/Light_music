#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "CubeCell_NeoPixel.h"

#define PIN GPIO3
#define NUM_LEDS 120  // Nombre total de LEDs sur la bande 121 au total

#define RF_FREQUENCY        433000000 // fréquence 433 MHz
#define LORA_BANDWIDTH      0         // 0: 125kHz, 1: 250kHz, 2: 500kHz
#define LORA_SPREADING_FACTOR 7       // SF7..SF12
#define LORA_CODINGRATE     1         // 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH 8        // préambule
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

static RadioEvents_t RadioEvents;

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

CubeCell_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

uint32_t mode = 0;

void setup() {
  Serial.begin(115200);

  strip.begin();
  strip.clear(); 
  strip.show(); // Initialise toutes les LEDs éteintes

  delay(150);   // stabilisation

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);

  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("LoRa RX démarré...");
  Radio.Rx(0);  // 0 = réception continue
}

void loop() {
  Radio.IrqProcess();  // gestion des interruptions radio
  
  switch (mode) {
    case 1:
      mode_1();
      break;

    default:
      strip.clear(); 
      strip.show();
      break;
  } 

}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Serial.print("Message reçu : ");
  //for (uint16_t i = 0; i < size; i++) {
  //  Serial.print((uint8_t)payload[i]);
  //  mode = (uint8_t)payload[i];
  //}
  for (int i = 0; i < 4; i++) {
      int b = (uint8_t)payload[i];
      if (b == -1) break;
      ((uint8_t*)&mode)[i] = b;
    }
  Serial.println("Compteur reçu : ");
  Serial.print(mode);

  Serial.println();
  Serial.printf("RSSI: %d dBm, SNR: %d dB\n", rssi, snr);

  Radio.Rx(0); // relance la réception
}

void mode_1(){
  Serial.println("Mode 1 !!");
  for (int i = 1; i < NUM_LEDS; i+=8) {
    strip.setPixelColor(i + 0, strip.Color(50, 0, 0)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 1, strip.Color(40, 10, 0)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 2, strip.Color(30, 20, 0)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 3, strip.Color(20, 30, 0)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 4, strip.Color(10, 40, 0)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 5, strip.Color(0, 50, 0)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 6, strip.Color(0, 40, 10)); // rouge
    strip.show();
    delay(10);
    strip.setPixelColor(i + 7, strip.Color(0, 30, 20)); // rouge
    strip.show();
    delay(50);
  }
  delay(1000);
  strip.clear();
  strip.show();
  delay(1000);
  
}