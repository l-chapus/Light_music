#include "LoRaWan_APP.h"
#include "Arduino.h"

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

void setup() {
  Serial.begin(115200);

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
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Serial.print("Message reçu : ");
  for (uint16_t i = 0; i < size; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.printf("RSSI: %d dBm, SNR: %d dB\n", rssi, snr);

  Radio.Rx(0); // relance la réception
}