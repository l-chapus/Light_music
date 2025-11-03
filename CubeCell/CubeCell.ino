#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "CubeCell_NeoPixel.h"
#include "config.h"

bool etatSortie = false;       // mémorise l'état actuel
unsigned long int tempsAppui = 0;

static RadioEvents_t RadioEvents; // Pour la réception LoRa

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
uint8_t dataLora[FFT_SIZE];   // tableau des données reçu en LoRa
bool dataReady = false;

CubeCell_NeoPixel strip(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

uint8_t mode = 0;

// === Fonction appelée lors de l’interruption ===
void boutonInterrupt() {
  if (digitalRead(PIN_BOUTON) == LOW) {
    tempsAppui = millis();                // Sauvegarde le temps où le bouton a été appuyé
  }
  else {
    if (millis() - tempsAppui > TEMPO_SLEEP){
      etatSortie = !etatSortie;
      if (!etatSortie) animationStop();
      digitalWrite(PIN_EN, etatSortie); // allume le bandeau LED
      if (etatSortie) animationStart();
      mode = 0;

      if (DEBUG_BOUTON) Serial.println("ON/OFF");
    }
    else if (etatSortie == true){
      mode++;                           // Incrémente le mode
      if (DEBUG_BOUTON) Serial.printf("Mode : %d \n", mode);
    }
  }
}

void setup() {
  pinMode(PIN_BOUTON, INPUT_PULLUP); // Bouton
  pinMode(PIN_EN, OUTPUT);           // Enable du convertisseur 5V

  attachInterrupt(digitalPinToInterrupt(PIN_BOUTON), boutonInterrupt, CHANGE);

  if (DEBUG_ANIMATION) {
    digitalWrite(PIN_EN, true); // allume le bandeau LED
    etatSortie = true;
  }
  
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

void setPixelColor(uint8_t x, uint8_t y, uint8_t R, uint8_t G, uint8_t B) {
  if (x > 10 || x < 0 || y > 10 || y < 0) {// si on est en dehors du tableau
    Serial.println("ERREUR 2 : Coordonnées en dehors du tableau !"); 
    return;
  }
  strip.setPixelColor(correspondance_led[y][x], R, G, B);
  //if (DEBUG_ANIMATION) Serial.printf("Coordonnée demandée x : %d , y : %d et numéro de la LED : %d \n",x ,y ,correspondance_led[y][x]);
}

void loop() { 
  Radio.IrqProcess();  // gestion des interruptions radio

  // TEMP
  //float voltage = getBatteryVoltage();
  //Serial.printf("Tension batterie : %.2f V\n", voltage);
  //Serial.print(voltage);
  
  //Serial.println("--------------");
  //delay(5000);
  // END TEMP

  switch (mode) {
    case 0: // utilisation en DEBUG uniquement
      ligne_couleur_static(9, 250, 0, 0);
      break;
    case 1:   // mode pour l'affichage de la musique
      colone_couleur_static(8, 0,150, 0);
      break;
    case 2:
      animation_matrix(); 
      break;
    case 3:
      animation_wave();
      break;
    case 4:
      couleur_static(0,25,0);
      break;
    case 5:
      couleur_static(0,25,25);
      break;
    case 6:
      animation_scintillement();
      break;
    case 248:
      colone_couleur_static(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   // colone static avec une couleur
      dataReady = false;
      break;
    case 249:
      ligne_couleur_static(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   // ligne static avec une couleur
      dataReady = false;
      break;
    case 250:
      couleur_static(dataLora[0], dataLora[1], dataLora[2]);                      // uni avec une couleur static
      dataReady = false;
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
  memcpy(&mode, payload, sizeof(mode)); // Lecture du mode d'affichage

  if (mode == 1){
    // Vérif si assez de données pour la FFT
    uint16_t expected_size = 1 + (FFT_SIZE) * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 1 : Paquet tronqué, données FFT incomplètes !");
    } else {
      memcpy(dataLora, payload + 1, (FFT_SIZE) * sizeof(uint8_t));
    }
    affichage_musique(dataLora);
    dataReady = true;
  }

  if (mode == 249 || mode == 248){ // extraction de 4 variables
    uint16_t expected_size = 4 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 249 : Paquet tronqué, données incomplètes !");
    } else {
      memcpy(dataLora, payload + 1, 4 * sizeof(uint8_t));
      dataReady = true;
    }
  }
  if (mode == 250){ // extraction de 3 variables
    uint16_t expected_size = 3 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 250 : Paquet tronqué, données incomplètes ! ");
    } else {
      memcpy(dataLora, payload + 1, 3 * sizeof(uint8_t));
      dataReady = true;
    }
  }
   
  // --- Affichage DEBUG ---
  if (DEBUG_RECEPTION) {
    Serial.print("Message reçu : ");
    Serial.print("Compteur/mode : ");
    Serial.println(mode);
  
    Serial.print("FFT (partielle) : ");
    for (uint8_t i = 0; i < FFT_SIZE; i++) {
      Serial.print(dataLora[i], 1); // 1 décimale
      Serial.print(" ");
    }
    Serial.println();
    Serial.printf("RSSI: %d dBm, SNR: %d dB\n", rssi, snr);
    Serial.println("-------------------");
  }
  
}

void couleur_static(int r, int g, int b){ // Affichage d'une couleur static
  for (int x = 0; x < WIDTH; x++) {
    for (int y = 0; y < HEIGHT; y++) {
      setPixelColor(x, y, r, g, b); 
    }
  }
  strip.show();
}

void ligne_couleur_static(int numLigne, int r, int g, int b){ // Affichage d'une ligne avec une couleur static
  for (int x = 0; x < WIDTH; x++) {
    setPixelColor(x, numLigne, r, g, b); 
  }
  delay(15);
  strip.show();
}
void colone_couleur_static(int numcolone, int r, int g, int b){ // Affichage d'une colone avec une couleur static
  for (int y = 0; y < HEIGHT; y++) {
    setPixelColor(numcolone, y, r, g, b); 
  }
  delay(15);
  strip.show();
}

void animation_wave() {
  static uint8_t frame = 0;
  const int cx = WIDTH / 2;
  const int cy = HEIGHT / 2;

  strip.clear();
  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      float dx = x - cx;
      float dy = y - cy;
      float dist = sqrt(dx * dx + dy * dy);
      if (abs(dist - frame / 2.0) < 0.8) {
        uint8_t c = 255 - (dist * 30);
        setPixelColor(x, y, c, c / 2, 0);
      }
    }
  }

  strip.show();
  frame = (frame + 1) % (HEIGHT * 3);
  delay(50);
}

void animation_matrix() {
  static int drops[10] = {0};

  // Fait descendre chaque colonne
  for (int x = 0; x < 10; x++) {
    drops[x]++;
    if (drops[x] >= 10 || random(0, 100) < 5)
      drops[x] = 0;
  }

  // Efface tout
  strip.clear();

  // Affiche les gouttes vertes
  for (int x = 0; x < 10; x++) {
    int y = drops[x];
    if (y >= 0 && y < 10)
      setPixelColor(x, y, 0, 255, 0);     // tête brillante
    if (y - 1 >= 0)
      setPixelColor(x, y - 1, 0, 100, 0); // traînée moyenne
    if (y - 2 >= 0)
      setPixelColor(x, y - 2, 0, 40, 0);  // traînée faible
  }

  strip.show();
  delay(100);
}

void animation_scintillement() {
  for(int i=1; i<NUM_LEDS; i++){
    if(random(0,10)<2) strip.setPixelColor(i, 30, 0 ,90);
    else strip.setPixelColor(i,0,0,0);
  }
  strip.show();
  delay(100);
}

void animationStart() {
  mode = 2;
  delay(100);
  couleur_static(0,0,10);
  delay(300);
  couleur_static(0,0,20);
  delay(300);
  couleur_static(0,0,80);
  delay(300);
  strip.clear(); 
  strip.show();
}

void animationStop() {
  mode = 2;
  delay(100);
  couleur_static(10,0,0);
  delay(300);
  couleur_static(20,0,0);
  delay(300);
  couleur_static(50,0,0);
  delay(300);
  strip.clear(); 
  strip.show();
}