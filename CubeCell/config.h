#ifndef CONFIG_H
#define CONFIG_H

// --- DEBUG---
#define DEBUG_BOUTON false
#define DEBUG_ANIMATION true
#define DEBUG_RECEPTION false
#define DEBUG_BATTERIE false

// --- Pin pour la bande LED ---
#define PIN_LEDS GPIO2
#define NUM_LEDS 101  // Nombre total de LEDs sur la bande 101 au total
#define WIDTH 10
#define HEIGHT 10

// --- Données nécessaire pour le module LoRa ---
#define RF_FREQUENCY        433000000 // fréquence 433 MHz
#define LORA_BANDWIDTH      0         // 0: 125kHz, 1: 250kHz, 2: 500kHz
#define LORA_SPREADING_FACTOR 7       // SF7..SF12
#define LORA_CODINGRATE     1         // 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH 8        // préambule
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

// --- Pin pour le bouton et le Enable du convertisseur 5V ---
#define PIN_BOUTON GPIO4
#define PIN_EN GPIO5
#define TEMPO_SLEEP 1000 // temporisation pour le ON/OFF en millisecondes

const uint8_t FFT_SIZE = 16; // Taille de la FFT

const uint8_t correspondance_led[10][10] = {
  {100, 81, 80, 61, 60, 41, 40, 21, 20, 1},
  {99, 82, 79, 62, 59, 42, 39, 22, 19, 2},
  {98, 83, 78, 63, 58, 43, 38, 23, 18, 3},
  {97, 84, 77, 64, 57, 44, 37, 24, 17, 4},
  {96, 85, 76, 65, 56, 45, 36, 25, 16, 5},
  {95, 86, 75, 66, 55, 46, 35, 26, 15, 6},
  {94, 87, 74, 67, 54, 47, 34, 27, 14, 7},
  {93, 88, 73, 68, 53, 48, 33, 28, 13, 8},
  {92, 89, 72, 69, 52, 49, 32, 29, 12, 9},
  {91, 90, 71, 70, 51, 50, 31, 30, 11, 10},
};

// --- Sauvegarde de l'id dans l'EEPROM ---
#define EEPROM_SIZE 64       // taille réservée dans la Flash
#define EEPROM_ADDR_ID 0     // adresse de stockage de ton ID

// --- Batterie ---
const uint8_t LIRE_NIVEAU = 90; // Temps entre 2 lectures du niveau de batterie (temps en secondes)
const uint8_t TEMPS_INACTIVITEE = 1; // Temps après arrêt automatique (temps en minutes)

#endif