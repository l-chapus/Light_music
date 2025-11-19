#ifndef CONFIG_H
#define CONFIG_H

// --- DEBUG---
#define DEBUG_BOUTON false
#define DEBUG_ANIMATION true
#define DEBUG_RECEPTION false
#define DEBUG_BATTERIE true

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

const uint8_t CORRESPONDANCE_LED[10][10] = {
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
const uint8_t LIRE_NIVEAU = 40; // Temps entre 2 lectures du niveau de batterie (temps en secondes)
const uint8_t TEMPS_INACTIVITEE = 5; // Temps après arrêt automatique (temps en minutes)


// --- Constante pour l'animation de trou noir ---
const float DISTANCE[10][10] = {
  {7.07, 6.40, 5.83, 5.39, 5.10, 5.00, 5.10, 5.39, 5.83, 6.40},
  {6.40, 5.66, 5.00, 4.47, 4.12, 4.00, 4.12, 4.47, 5.00, 5.66},
  {5.83, 5.00, 4.24, 3.61, 3.16, 3.00, 3.16, 3.61, 4.24, 5.00},
  {5.39, 4.47, 3.61, 2.83, 2.24, 2.00, 2.24, 2.83, 3.61, 4.47},
  {5.10, 4.12, 3.16, 2.24, 1.41, 1.00, 1.41, 2.24, 3.16, 4.12},
  {5.00, 4.00, 3.00, 2.00, 1.00, 0.00, 1.00, 2.00, 3.00, 4.00},
  {5.10, 4.12, 3.16, 2.24, 1.41, 1.00, 1.41, 2.24, 3.16, 4.12},
  {5.39, 4.47, 3.61, 2.83, 2.24, 2.00, 2.24, 2.83, 3.61, 4.47},
  {5.83, 5.00, 4.24, 3.61, 3.16, 3.00, 3.16, 3.61, 4.24, 5.00},
  {6.40, 5.66, 5.00, 4.47, 4.12, 4.00, 4.12, 4.47, 5.00, 5.66},
};

const float INFLUENCE[10][10] = {
  {0.00, 0.09, 0.17, 0.23, 0.27, 0.29, 0.27, 0.23, 0.17, 0.06},
  {0.09, 0.19, 0.29, 0.36, 0.41, 0.43, 0.41, 0.36, 0.29, 0.19},
  {0.17, 0.29, 0.39, 0.48, 0.55, 0.57, 0.55, 0.48, 0.39, 0.29},
  {0.23, 0.36, 0.48, 0.60, 0.68, 0.71, 0.68, 0.60, 0.48, 0.36},
  {0.27, 0.41, 0.55, 0.68, 0.80, 0.86, 0.80, 0.68, 0.55, 0.41},
  {0.29, 0.43, 0.57, 0.71, 0.86, 1.00, 0.86, 0.71, 0.57, 0.43},
  {0.27, 0.41, 0.55, 0.68, 0.80, 0.86, 0.80, 0.68, 0.55, 0.41},
  {0.23, 0.36, 0.48, 0.60, 0.68, 0.71, 0.68, 0.60, 0.48, 0.36},
  {0.17, 0.29, 0.39, 0.48, 0.55, 0.57, 0.55, 0.48, 0.39, 0.29},
  {0.09, 0.19, 0.29, 0.36, 0.41, 0.43, 0.41, 0.36, 0.29, 0.19},
};

const float ANGLE[10][10] = {
  {-2.36, -2.25, -2.11, -1.95, -1.77, -1.57, -1.37, -1.19, -1.03, -0.90},
  {-2.47, -2.36, -2.21, -2.03, -1.82, -1.57, -1.33, -1.11, -0.93, -0.79},
  {-2.60, -2.50, -2.36, -2.16, -1.89, -1.57, -1.25, -0.98, -0.79, -0.64},
  {-2.76, -2.68, -2.55, -2.36, -2.03, -1.57, -1.11, -0.79, -0.59, -0.46},
  {-2.94, -2.90, -2.82, -2.68, -2.36, -1.57, -0.79, -0.46, -0.32, -0.24},
  {3.14, 3.14, 3.14, 3.14, 3.14, 0.00, 0.00, 0.00, 0.00, 0.00},
  {2.94, 2.90, 2.82, 2.68, 2.36, 1.57, 0.79, 0.46, 0.32, 0.24},
  {2.76, 2.68, 2.55, 2.36, 2.03, 1.57, 1.11, 0.79, 0.59, 0.46},
  {2.60, 2.50, 2.36, 2.16, 1.89, 1.57, 1.25, 0.98, 0.79, 0.64},
  {2.47, 2.36, 2.21, 2.03, 1.82, 1.57, 1.33, 1.11, 0.93, 0.79},
};

#endif