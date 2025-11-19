#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "CubeCell_NeoPixel.h"
#include "config.h"
#include "EEPROM.h"

bool etatSortie = false;       // mémorise l'état actuel
unsigned long int tempsAppui = 0;
uint8_t deviceID = 0;        // variable de ton identifiant

static RadioEvents_t RadioEvents; // Pour la réception LoRa

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
uint8_t dataLora[FFT_SIZE];   // tableau des données reçu en LoRa
bool dataReady = false;

CubeCell_NeoPixel strip(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

uint8_t mode = 0;
uint8_t modePrecedent = 0;
static int frame = 0;
bool animationContinue = false;

uint32_t tempsBatterie = 0;      // Temps pour la lecture de la batterie
uint32_t tempsInactivitee = 0;   // Temps pour l'inactivitée
uint32_t tempsCourant = 0;      // Temps pour la lecture de la batterie
uint8_t niveauBatterie = 0;

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

  EEPROM.begin(EEPROM_SIZE); // initialise la mémoire Flash
  // Lecture de l'ID en mémoire
  deviceID = EEPROM.read(EEPROM_ADDR_ID);
  Serial.print("ID actuel lu depuis la Flash : ");
  Serial.println(deviceID);

  tempsBatterie = millis();     // initialise le temps
  tempsInactivitee = millis();  // initialise le temps
  niveauBatterie = getBatteryLevel();
}

void setPixelColor(uint8_t x, uint8_t y, uint8_t R, uint8_t G, uint8_t B) {
  if (x > WIDTH  || x < 0 || y > HEIGHT || y < 0) {// si on est en dehors du tableau
    Serial.println("ERREUR 2 : Coordonnées en dehors du tableau !"); 
    return;
  }
  strip.setPixelColor(CORRESPONDANCE_LED[y][x], R, G, B);
  //if (DEBUG_ANIMATION) Serial.printf("Coordonnée demandée x : %d , y : %d et numéro de la LED : %d \n",x ,y ,CORRESPONDANCE_LED[y][x]);
}
uint32_t getPixelColor(uint8_t x, uint8_t y) {
  return strip.getPixelColor(CORRESPONDANCE_LED[y][x]);
}
uint8_t getBatteryLevel(){
  float voltage = getBatteryVoltage();
  float niveauBatterie = 100.0 * (1 - exp(-0.0055 * (voltage - 3600))) /
                     (1 - exp(-0.0055 * (4230 - 3600)));            // calcul issue du max et min de batterie, 100% => 4240 et 0% => 3820
  niveauBatterie = constrain(niveauBatterie, 0, 100);
 
  if (DEBUG_BATTERIE) {
    Serial.printf("Tension batterie : %.2f V\n", voltage);
    Serial.printf("Niveau calculé de la batterie : %.2f %\n", niveauBatterie);
    Serial.printf("Temps courant : %d \n", tempsCourant);
    Serial.println("--------------"); 
  }

  return niveauBatterie;
}

void modeChange(){
  if (modePrecedent != mode || dataReady) {          // détecte le changement de mode
    tempsInactivitee = millis();      
    modePrecedent = mode;
  }
  
  switch (mode) {
    case 0: // utilisation en DEBUG uniquement
      break;
    case 1:
      animationBlackHole(250, 0, 10, 20, 1);
      animationContinue = true;
      break;
    case 2:
      animationBlackHole(10, 0, 150, 10, 0);
      animationContinue = true;
      break;
    case 3:
      animationExplosion(150, 0, 10, 100);
      animationContinue = true;
      break;
    case 4:
      animationStaticColor(0,25,0);
      animationContinue = false;
      break;
    case 5:
      animationMatrix(100); 
      animationContinue = true;
      break;
    case 6:
      animationSparkle(0, 20, 200, 90, 30);
      animationContinue = true;
      break;
    case 7:
      animationGradientFlow(0, 20, 200, 10, 150, 0, 20); 
      animationContinue = true;
      break;
    case 8: 
      animationBreathing(0, 20, 200, 30); 
      animationContinue = true;
      break;
    case 9:
      animationScrolling(100, 0, 50, 50, 3, 2); 
      animationContinue = true;
      break;
    case 10:
      animationScrolling(10, 100, 80, 20, 0, 5); 
      animationContinue = true;
      break;
    case 11:
      animationNoise(40, 6);
      animationContinue = true;
      break;
    case 12:
      frame = 150;
      animationStroboscope(150, 20, 10, 50); 
      animationContinue = true;
      break;
    case 13:
      animationRainbow(20, 1, 20); 
      animationContinue = true;
      break;
    case 14:
      animationWaveNegativ(10, 0, 150, 50, 0); 
      animationContinue = true;
      break;
    case 15:
      animationBlackHole(10, 0, 150, 10, 1); 
      animationContinue = true;
      break;

    case 134:
      if (dataReady){
        setPixelColor(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);  
        dataReady = false;
        animationContinue = false; 
      }
      break;
    case 135:
      if (dataReady || animationContinue){
        animationBlackHole(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);  
        dataReady = false;
        animationContinue = true; 
      }
      break;
    case 136:
      if (dataReady || animationContinue){
        animationWaveNegativ(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);  
        dataReady = false;
        animationContinue = true; 
      }
      break;
    case 137:
      if (dataReady || animationContinue){
        animationRainbow(dataLora[0], dataLora[1], dataLora[2]);  
        dataReady = false;
        animationContinue = true; 
      }
      break;
    case 138:
      if (dataReady || animationContinue){
        frame = dataLora[4];
        dataReady = false;
        animationContinue = true;
        mode = 139;
        animationStroboscope(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   
      }
      break;
    case 139:
      if (animationContinue){     // appelé uniquement par le mode 138
        animationStroboscope(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 140:
      if (dataReady || animationContinue){
        animationNoise(dataLora[0], dataLora[1]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 141:
      if (dataReady || animationContinue){
        animationBreathing(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 142:
      if (dataReady || animationContinue){
        animationGradientFlow(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4], dataLora[5], dataLora[6]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 143:
      if (dataReady || animationContinue){
        animationSparkle(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 144:
      if (dataReady || animationContinue){
        animationMatrix(dataLora[0]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 145:
      if (dataReady || animationContinue){
        animationExplosion(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 146:
      if (dataReady || animationContinue){
        animationWavePositiv(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 147:
      if (dataReady || animationContinue){
        animationScrolling(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4], dataLora[5]);   // défilement avec une couleur
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 148:
      if (dataReady){
        animationColumnStatic(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   // colone static avec une couleur
        dataReady = false;
        animationContinue = false;
      }
      break;
    case 149:
      if (dataReady){
        animationRowStatic(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   // ligne static avec une couleur
        dataReady = false;
        animationContinue = false;
      }
      break;
    case 150:
      if (dataReady){
        animationStaticColor(dataLora[0], dataLora[1], dataLora[2]);                      // uni avec une couleur static
        dataReady = false;
        animationContinue = false;
      }
      break;
    case 151:
      if (dataReady){
        strip.clear(); 
        strip.show();
        dataReady = false;
        animationContinue = false;
      }
      break;  
    case 152:       // mode pour l'affichage de la batterie faible
      animationScrolling(100, 0, 0, 200, 2, 3);
      animationContinue = true;
      break;
    case 153:       // mode pour renvoyer le niveau de batterie
      static uint8_t niveauBatterieToSend = 0; 
      niveauBatterieToSend = getBatteryLevel();
      delay(100);
      Radio.Send(&niveauBatterieToSend, sizeof(niveauBatterieToSend));
      delay(20);
      mode = 0;
      animationContinue = false;
      break;
    case 154:
      mode = 0;
      animationContinue = false;
      routineStop();
      break;
    case 200:
      if (dataReady){
        if (deviceID != dataLora[0]){
          EEPROM.write(EEPROM_ADDR_ID, dataLora[0]);
          EEPROM.commit(); // obligatoire pour valider l’écriture
        }
        dataReady = false;
      }
      break;

    default:
      strip.clear(); 
      strip.show();
      animationContinue = false;
      break;
  } 
}

// === Fonction appelée lors de l’interruption ===
void boutonInterrupt() {
  if (digitalRead(PIN_BOUTON) == LOW) {
    tempsAppui = millis();                // Sauvegarde le temps où le bouton a été appuyé
  }
  else {
    if (millis() - tempsAppui > TEMPO_SLEEP){
      etatSortie = !etatSortie;
      if (!etatSortie) routineStop();
      //digitalWrite(PIN_EN, etatSortie); // allume ou éteint le bandeau LED
      if (etatSortie) routineStart();

      mode = 0;

      if (DEBUG_BOUTON) Serial.println("ON/OFF");
    }
    else if (etatSortie == true){
      if (mode > 100) mode = 0;
      mode++;                           // Incrémente le mode
      modeChange();
      if (DEBUG_BOUTON) Serial.printf("Mode : %d \n", mode);
    }
    strip.clear();
    strip.show();
  }
}

void loop() {  

  if (!etatSortie) {      // Carte en veille
    lowPowerHandler();    // met en veille profond la carte = 0.24 mA
  }
  else {                  // Carte allumée
    tempsCourant = millis();
    
    if ((tempsCourant - tempsBatterie) > LIRE_NIVEAU * 1000) {      // lecture du niveau de batterie
      tempsBatterie = tempsCourant;
      int niveauBatterie = getBatteryLevel();
      if (niveauBatterie <= 8) {   // fait clignoter une led pour signaler le faible pourcentage
        mode = 152;
        animationContinue = true;
      }
      if (niveauBatterie <= 2) {   // eteind de force la carte pour préservé la batterie restante
        etatSortie = false;
        routineStop();
      }
    }

    if ((tempsCourant - tempsInactivitee) > TEMPS_INACTIVITEE * 1000 * 60 && tempsCourant > tempsInactivitee) {      // eteind la carte au bout de TEMPS_INACTIVITEE minutes
      tempsInactivitee = tempsCourant;
      etatSortie = false;
      
      if (DEBUG_BATTERIE){
        Serial.println("--------------"); 
        Serial.print("Inactivité détécté, la carte s'éteind \n");
        Serial.print("Temps courant : "); Serial.println(tempsCourant);
        Serial.print("Temps inactivité : "); Serial.println(tempsInactivitee);
        Serial.print("Temps après calcul : "); Serial.println(tempsCourant - tempsInactivitee);
        Serial.print("Temps après calcul abs : "); Serial.println(abs(tempsCourant - tempsInactivitee));
        Serial.println("--------------"); 
      }
      
      routineStop();      
    }

    Radio.IrqProcess();  // gestion des interruptions radio
    if (animationContinue){
      modeChange();
    }
  }
}

bool extractPayload(uint8_t *payload, uint16_t sizePayload, uint8_t n) {
  if (sizePayload < n * sizeof(uint8_t)) return false;
  memcpy(dataLora, payload + 2, n * sizeof(uint8_t));
  dataReady = true;
  return true;
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint8_t IdRecu = 0;
  dataReady = false;
  
  memcpy(&IdRecu, payload, sizeof(IdRecu)); // Lecture de l'ID
  if (IdRecu != deviceID) return;           // si on ne vise pas ce composant

  memcpy(&mode, payload + 1, sizeof(mode)); // Lecture du mode d'affichage

  if (mode == 151 || mode == 153 || mode == 154){ // extraction de 0 variable
    dataReady = true;
  }
  if (mode == 200 || mode == 144){ // extraction de 1 variable
    if (!extractPayload(payload, size, 1)) {
      Serial.println("ERREUR 250 : Paquet tronqué, données incomplètes ! ");
    } 
  }
  if (mode == 140){ // extraction de 2 variable
    if (!extractPayload(payload, size, 2)) {
      Serial.println("ERREUR 255 : Paquet tronqué, données incomplètes ! ");
    } 
  }
  if (mode == 150){ // extraction de 3 variables
    if (!extractPayload(payload, size, 3)) {
      Serial.println("ERREUR 255 : Paquet tronqué, données incomplètes ! ");
    } 
  }
  if (mode == 149 || mode == 148 || mode == 145 || mode == 141){ // extraction de 4 variables
    if (!extractPayload(payload, size, 4)) {
      Serial.println("ERREUR 255 : Paquet tronqué, données incomplètes ! ");
    } 
  }
  if (mode == 146 || mode == 143 || mode == 138 || mode == 134){ // extraction de 5 variables
    if (!extractPayload(payload, size, 5)) {
      Serial.println("ERREUR 255 : Paquet tronqué, données incomplètes ! ");
    } 
  }
  if (mode == 147){ // extraction de 6 variables
    if (!extractPayload(payload, size, 6)) {
      Serial.println("ERREUR 255 : Paquet tronqué, données incomplètes ! ");
    } 
  }
  if (mode == 142){ // extraction de 7 variables
    if (!extractPayload(payload, size, 7)) {
      Serial.println("ERREUR 255 : Paquet tronqué, données incomplètes ! ");
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
  if (dataReady) modeChange();
}

void routineStop() {
  frame = 0;
  for (int k=0; k<25; ++k){
    animationWavePositiv(200, 0, 30, 20, 1);
  }
  delay(30);
  Radio.Sleep();    // Stop la réception LoRa
  delay(100);
  digitalWrite(PIN_EN, etatSortie); // éteint le bandeau LED
}
void routineStart() {
  frame = 0;
  digitalWrite(PIN_EN, etatSortie); // allume le bandeau LED
  delay(200);
  for (int k=0; k<25; ++k){
    animationWavePositiv(0, 20, 200, 20, 0);
  }
  delay(30);
  // Redémarre la réception LoRa
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  Radio.Rx(0);  // 0 = réception continue
  delay(100);
}
void animationStaticColor(uint8_t R, uint8_t G, uint8_t B){ // Affichage d'une couleur static
  for (int x = 0; x < WIDTH; x++) {
    for (int y = 0; y < HEIGHT; y++) {
      setPixelColor(x, y, R, G, B); 
    }
  }
  strip.show();
}
void animationRowStatic(uint8_t R, uint8_t G, uint8_t B, uint8_t numLigne){ // Affichage d'une ligne avec une couleur static
  if (numLigne > HEIGHT) return;
  for (int x = 0; x < WIDTH; x++) {
    setPixelColor(x, numLigne, R, G, B); 
  }
  delay(15);
  strip.show();
}
void animationColumnStatic(uint8_t R, uint8_t G, uint8_t B, uint8_t numcolone){ // Affichage d'une colone avec une couleur static
  if (numcolone > WIDTH) return;
  for (int y = 0; y < HEIGHT; y++) {
    setPixelColor(numcolone, y, R, G, B); 
  }
  delay(15);
  strip.show();
}
void animationScrolling(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse, uint8_t sens, uint8_t trainee) {
  if (trainee > WIDTH) return;
  
  // ---  Atténue la couleur existante pour créer la traînée ---
  for (uint8_t y = 0; y < 10; y++) {
    for (uint8_t x = 0; x < 10; x++) {
      // On récupère la couleur actuelle
      uint32_t color = getPixelColor(x, y);
      uint8_t r = (color >> 16) & 0xFF;
      uint8_t g = (color >> 8) & 0xFF;
      uint8_t b = color & 0xFF;

      // On réduit la luminosité selon le coefficient de traînée
      r = r * trainee/10;
      g = g * trainee/10;
      b = b * trainee/10;

      setPixelColor(x, y, r, g, b);
    }
  }

  // --- Allume la ligne/colonne active ---
  switch (sens) {
    case 0: { // Droite
      uint8_t x_actif = frame % 10;
      animationColumnStatic(R, G, B, x_actif);
      break;
    }

    case 1: { // Gauche
      uint8_t x_actif = (9 - (frame % 10));
      animationColumnStatic(R, G, B, x_actif);
      break;
    }

    case 2: { // Bas
      uint8_t y_actif = frame % 10;
      animationRowStatic(R, G, B, y_actif);
      break;
    }

    case 3: { // Haut
      uint8_t y_actif = (9 - (frame % 10));
      animationRowStatic(R, G, B, y_actif);
      break;
    }

    default:
      Serial.println("ERREUR 4 : Sens de défilement invalide !");
      return;
  }

  // --- Mise à jour de l'affichage ---
  strip.show();

  // --- Frame suivante ---
  frame = (frame + 1) % 10;
  delay(vitesse);
}
void animationWavePositiv(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse, uint8_t sens) {
  if (sens != 0 && sens != 1) return;

  const int cx = WIDTH / 2;
  const int cy = HEIGHT / 2;
  const int maxFrame = HEIGHT * 3; // amplitude max de l’onde

  strip.clear();
  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      float dx = x - cx;
      float dy = y - cy;
      float dist = sqrt(dx * dx + dy * dy);
      if (abs(dist - frame / 2.0) < 0.8) {
        uint8_t cR = max(0, R - (dist * 30));
        uint8_t cG = max(0, G - (dist * 30));
        uint8_t cB = max(0, B - (dist * 30));
        setPixelColor(x, y, cR, cG, cB);
      }
    }
  }

  strip.show();
  // Gestion du sens de propagation
  if (sens == 0) { 
    frame++;  // vers l’extérieur
    if (frame >= maxFrame) frame = 0;
  } 
  else {    
    frame--;  // vers le centre
    if (frame <= 0) frame = maxFrame;
  }
  delay(vitesse);
}
void animationExplosion(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse) {
  const int cx = WIDTH / 2;
  const int cy = HEIGHT / 2;

  for (int radius = 0; radius < max(WIDTH, HEIGHT); radius++) {
    strip.clear();
    for (int y = 0; y < HEIGHT; y++) {
      for (int x = 0; x < WIDTH; x++) {
        float dist = sqrt(pow(x - cx, 2) + pow(y - cy, 2));
        if (abs(dist - radius) < 0.8) {
          setPixelColor(x, y, R, G, B);
        }
      }
    }
    strip.show();
    delay(vitesse);
  }
}
void animationMatrix(uint8_t vitesse) {
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
  delay(vitesse);
}
void animationSparkle(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse, uint8_t remplissage) {
  if (remplissage > 100) return;
  for(int i=1; i<NUM_LEDS; i++){
    if(random(0,100)<remplissage) strip.setPixelColor(i, R, G ,B);
    else strip.setPixelColor(i,0,0,0);
  }
  strip.show();
  delay(vitesse);
}
void animationGradientFlow(uint8_t R_1, uint8_t G_1, uint8_t B_1, uint8_t R_2, uint8_t G_2, uint8_t B_2, uint8_t vitesse) {
  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      float t = (sin((x + frame) * 0.2) + 1) / 2.0;
      int r = R_1 + t * (R_2 - R_1);
      int g = G_1 + t * (G_2 - G_1);
      int b = B_1 + t * (B_2 - B_1);
      setPixelColor(x, y, r, g, b);
    }
  }
  strip.show();
  frame++;
  delay(vitesse);
}
void animationBreathing(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse) {
  float intensity = (sin(frame * 0.05) + 1.0) / 2.0;

  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      setPixelColor(x, y, R * intensity, G * intensity, B * intensity);
    }
  }

  strip.show();
  frame++;
  delay(vitesse);
}
void animationNoise(uint8_t vitesse, uint8_t intensite) {
  if (intensite > 10) return;
  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      uint8_t r = min(255, random(25)*intensite);
      uint8_t g = min(255, random(25)*intensite);
      uint8_t b = min(255, random(25)*intensite);
      setPixelColor(x, y, r, g, b); 
    }
  }
  strip.show();
  delay(vitesse);
}
void animationStroboscope(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse){
  if (frame > 0) {
    animationStaticColor(R, G, B);
    strip.show();
    delay(vitesse);
    strip.clear();
    strip.show();
    delay(vitesse);
    if (frame != 255) frame--;
  }
}
void animationRainbow(uint8_t vitesse, uint8_t sens, uint8_t intensite) {
  strip.clear();
  if (intensite > 255) intensite = 255;

  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      uint8_t hue = (x * 10 + y * 5 + frame) & 255;
      // Couleur HSV -> RGB
      uint32_t c = strip.gamma32(strip.ColorHSV(hue * 256));
      // Extraction RGB
      uint8_t r = (c >> 16) & 255;
      uint8_t g = (c >> 8) & 255;
      uint8_t b = c & 255;

      // Application de l’intensité (0–255)
      r = (r * intensite) >> 8;
      g = (g * intensite) >> 8;
      b = (b * intensite) >> 8;

      setPixelColor(x, y, r, g, b);
    }
  }
  strip.show();
  // gestion du sens
  frame += (sens == 0 ? 1 : -1);

  delay(vitesse);
}
void animationWaveNegativ(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse, uint8_t sens) {
  if (sens != 0 && sens != 1) return;
  strip.clear();

  for (uint8_t y = 0; y < HEIGHT; y++) {
    float intensity = (sin((y * 0.4) + frame * 0.12) + 1) * 0.5;
    for (uint8_t x = 0; x < WIDTH; x++)
      setPixelColor(x, y, R * intensity, G * intensity, B * intensity);
  }

  strip.show();
  frame += (sens == 0 ? 1 : -1);
  delay(vitesse);
}
void animationBlackHole(uint8_t R, uint8_t G, uint8_t B, uint8_t vitesse, uint8_t sens) {
  if (sens != 0 && sens != 1) return;
  strip.clear();

  const float cx = WIDTH / 2.0;
  const float cy = HEIGHT / 2.0;

  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      float dist = DISTANCE[y][x];        // Tableau précalculé
      float influence = INFLUENCE[y][x];  // Tableau précalculé
      float angle = ANGLE[y][x];          // Tableau précalculé

      angle += frame * 0.03 * influence;

      float rFactor = sin(frame * 0.03 + DISTANCE[y][x] * 0.5) * 0.5 + 0.5;

      uint8_t r = R * influence * rFactor;
      uint8_t g = G * influence * rFactor;
      uint8_t b = B * influence * rFactor;

      setPixelColor(x, y, r, g, b);
    }
  }

  strip.show();
  frame += (sens == 0 ? 1 : -1);
  delay(vitesse);
}