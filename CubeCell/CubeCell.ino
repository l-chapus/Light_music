#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "CubeCell_NeoPixel.h"

#define PIN_LEDS GPIO2
#define NUM_LEDS 120  // Nombre total de LEDs sur la bande 121 au total

#define RF_FREQUENCY        433000000 // fréquence 433 MHz
#define LORA_BANDWIDTH      0         // 0: 125kHz, 1: 250kHz, 2: 500kHz
#define LORA_SPREADING_FACTOR 7       // SF7..SF12
#define LORA_CODINGRATE     1         // 1=4/5, 2=4/6, 3=4/7, 4=4/8
#define LORA_PREAMBLE_LENGTH 8        // préambule
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define PIN_BOUTON GPIO4
#define PIN_EN GPIO5

bool etatSortie = false;       // mémorise l'état actuel
bool dernierEtatBouton = HIGH; // HIGH = relâché (pull-up activé)
unsigned long dernierDebounce = 0;
const unsigned long debounceDelay = 50; // anti-rebond en millisecondes

static RadioEvents_t RadioEvents;

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

CubeCell_NeoPixel strip(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

uint8_t mode = 0; 
int offset = 0;
int tab_spiral[120] = {0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0, 0, 0, 0, 5, 10, 25, 0, 0, 0};
int tab_spiral_temp[120];
int spiral_offset = 0;  // décalage du motif

// Définition du motif lumineux
const int motif[] = {25, 10, 5, 0, 0, 0, 0, 0, 0};  
const int motif_size = sizeof(motif) / sizeof(motif[0]);

const uint16_t FFT_SIZE = 16;

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

  pinMode(PIN_BOUTON, INPUT_PULLUP);
  pinMode(PIN_EN, OUTPUT); 
}

void loop() {
  Radio.IrqProcess();  // gestion des interruptions radio

  int etat = digitalRead(PIN_BOUTON);
  // si l'état a changé, on démarre la temporisation d'anti-rebond
  if (etat != dernierEtatBouton) {
    dernierDebounce = millis();
  }

  // si le nouvel état est stable depuis assez longtemps
  if ((millis() - dernierDebounce) > debounceDelay) {
    if (etat == LOW && dernierEtatBouton == HIGH) {
      // bouton pressé (transition HIGH → LOW)
      etatSortie = !etatSortie;  // on inverse la sortie
      digitalWrite(PIN_EN, etatSortie);
      Serial.print("Sortie GPIO5 = ");
      Serial.println(etatSortie ? "HIGH" : "LOW");
    }
  }

  // mise à jour pour la prochaine boucle
  dernierEtatBouton = etat;

  // TEMP
  //float voltage = getBatteryVoltage();
  //Serial.printf("Tension batterie : %.2f V\n", voltage);
  //Serial.print(voltage);
  
  //Serial.println("--------------");
  //delay(5000);
  // END TEMP

  switch (mode) {
    case 0:
      break;
    case 1:
      mode_1();
      break;
    case 2:
      mode_couleur_static(25,0,0);
      break;
    case 3:
      mode_couleur_static(25,25,0);
      break;
    case 4:
      mode_couleur_static(0,25,0);
      break;
    case 5:
      mode_couleur_static(0,25,25);
      break;
    case 6:
      mode_couleur_static(0,0,25);
      break;
    case 7:
      mode_couleur_static(25,0,25);
      break;
    case 8:
      animation_spiral();
      break;
    case 9:
      animation_scintillement();
      break;
    case 10:
      animation_arc_en_ciel();
      break;
    case 11:
      animation_equalizer();
      break;

    default:
      strip.clear(); 
      strip.show();
      break;
  } 

}

void affichage_musique(uint8_t amplitude[FFT_SIZE]) {
  strip.clear();

  // On découpe le bandeau en (FFT_SIZE) segments
  // Exemple : 120 LEDs / (FFT_SIZE) = nombre de LEDs par bande
  int leds_per_band = NUM_LEDS / (FFT_SIZE);

  for (int band = 0; band < FFT_SIZE; band++) {
    // Récupère l’amplitude et la mappe sur une intensité 0..255
    int level = amplitude[band];
    if (level > leds_per_band) level = leds_per_band;

    // Choix d’une couleur en fonction de la bande (dégradé spectral)
    uint32_t color = strip.Color(
      (band * 5) % 155,          // rouge varie
      (255 - (band * 8)) % 155,  // vert varie
      (band * 15) % 155          // bleu varie
    );

    // Allume les LEDs correspondant au niveau de cette bande
    int start = band * leds_per_band;
    for (int i = 0; i < level; i++) {
      int led_index = start + i;
      if (led_index < NUM_LEDS) {
        strip.setPixelColor(led_index, color);
      }
    }
  }

  strip.show();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Serial.print("Message reçu : ");
 
  memcpy(&mode, payload, sizeof(mode));

  uint8_t vReal_rx[FFT_SIZE];

  // Vérif si assez de données pour la FFT
  uint16_t expected_size = 1 + (FFT_SIZE) * sizeof(uint8_t);
  if (size < expected_size) {
    Serial.println("⚠ Paquet tronqué, données FFT incomplètes !");
  } else {
    memcpy(vReal_rx, payload + 1, (FFT_SIZE) * sizeof(uint8_t));
  }

  // --- Affichage ---
  Serial.print("Compteur/mode : ");
  Serial.println(mode);

  Serial.print("FFT (partielle) : ");
  for (uint8_t i = 0; i < FFT_SIZE; i++) {
    Serial.print(vReal_rx[i], 1); // 1 décimale
    Serial.print(" ");
  }
  Serial.println();
  //Serial.printf("RSSI: %d dBm, SNR: %d dB\n", rssi, snr);
  Serial.println("-------------------");

  affichage_musique(vReal_rx);
}

void animation_equalizer() {
  for (int i = 0; i < NUM_LEDS; i++) {
    // Trouve la position de cette LED dans le motif
    int index = (i + spiral_offset) % motif_size;
    int val = motif[index];

    strip.setPixelColor(i, val, 0, 0);
  }

  strip.show();

  // Décalage à chaque frame
  spiral_offset = (spiral_offset + 1) % motif_size;

  delay(40);
}

void animation_arc_en_ciel() {
  static int offset = 0;
  for (int i = 0; i < NUM_LEDS; i++) {
    int pixelHue = (i * 65536L / NUM_LEDS + offset) & 0xFFFF;
    strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
  }
  strip.show();
  offset += 256;  // vitesse de défilement
  delay(20);
}

void animation_scintillement() {
  for(int i=0; i<NUM_LEDS; i++){
    if(random(0,10)<2) strip.setPixelColor(i, 30, 0 ,90);
    else strip.setPixelColor(i,0,0,0);
  }
  strip.show();
  delay(100);
}

void animation_spiral(){
  int k=0;
  for (int i = 1; i < NUM_LEDS; i++) {
    k = i-1;
    strip.setPixelColor(i, strip.Color(tab_spiral[k], 0, 0)); 
    
    if (k<119){
      tab_spiral_temp[k+1] = tab_spiral[k];
    }
  }
  
  if (offset == 0){
    tab_spiral_temp[0] = 25;
  }
  else if (offset == 1){
    tab_spiral_temp[0] = 10;
  }
  else if (offset == 2){
    tab_spiral_temp[0] = 5;
  }
  else {
    tab_spiral_temp[0] = 0;
  }
  offset = (offset + 1)%9;

  for (int i = 0; i < NUM_LEDS; i++) {
    tab_spiral[i] = tab_spiral_temp[i];
  }
  strip.show();
  delay(40);
}

void mode_couleur_static(int r, int g, int b){ // Affichage d'une couleur static
  for (int i = 1; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b)); 
    strip.show();
    delay(5);
  }
}

void mode_1(){
  for (int i = 1; i < NUM_LEDS; i+=8) {
    strip.setPixelColor(i + 0, strip.Color(50, 0, 0)); 
    strip.show();
    delay(10);
    strip.setPixelColor(i + 1, strip.Color(40, 10, 0)); 
    strip.show();
    delay(10);
    strip.setPixelColor(i + 2, strip.Color(30, 20, 0)); 
    strip.show();
    delay(10);
    strip.setPixelColor(i + 3, strip.Color(20, 30, 0));
    strip.show();
    delay(10);
    strip.setPixelColor(i + 4, strip.Color(10, 40, 0)); 
    strip.show();
    delay(10);
    strip.setPixelColor(i + 5, strip.Color(0, 50, 0)); 
    strip.show();
    delay(10);
    strip.setPixelColor(i + 6, strip.Color(0, 40, 10));
    strip.show();
    delay(10);
    strip.setPixelColor(i + 7, strip.Color(0, 30, 20)); 
    strip.show();
    delay(50);
  }
  delay(1000);
  strip.clear();
  strip.show();
  delay(1000);
}