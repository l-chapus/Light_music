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
  niveauBatterie = getPourcentageBatterie();
}

void setPixelColor(uint8_t x, uint8_t y, uint8_t R, uint8_t G, uint8_t B) {
  if (x > WIDTH  || x < 0 || y > HEIGHT || y < 0) {// si on est en dehors du tableau
    Serial.println("ERREUR 2 : Coordonnées en dehors du tableau !"); 
    return;
  }
  strip.setPixelColor(correspondance_led[y][x], R, G, B);
  //if (DEBUG_ANIMATION) Serial.printf("Coordonnée demandée x : %d , y : %d et numéro de la LED : %d \n",x ,y ,correspondance_led[y][x]);
}
uint32_t getPixelColor(uint8_t x, uint8_t y) {
  return strip.getPixelColor(correspondance_led[y][x]);
}
uint8_t getPourcentageBatterie(){
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

void changementMode(){
  tempsInactivitee = millis();      // détecte le changement de mode
  switch (mode) {
    case 0: // utilisation en DEBUG uniquement
      break;
    case 1:   // mode pour l'affichage de la musique
      defilement(10, 80, 63, 20, 2, 4);
      animationContinue = true;
      break;
    case 2:
      //animation_comet(200, 50, 30, 30, 20); 
      defilement(10, 0, 150, 50, 3, 2); 
      //animation_noise(10);
      animationContinue = true;
      break;
    case 3:
      animation_explosion(150, 0, 10, 100);
      animationContinue = true;
      break;
    case 4:
      couleur_static(0,25,0);
      animationContinue = false;
      break;
    case 5:
      animation_matrix(100); 
      animationContinue = true;
      break;
    case 6:
      animation_scintillement(0, 20, 200, 90, 30);
      animationContinue = true;
      break;
    case 7:
      animation_gradient_flow(0, 20, 200, 10, 150, 0, 20); 
      animationContinue = true;
      break;
    case 8: 
      animation_breathing(0, 20, 200, 30); 
      animationContinue = true;
      break;
    case 9:
      defilement(100, 0, 50, 50, 3, 2); 
      animationContinue = true;
      break;
    case 10:
      defilement(10, 100, 80, 20, 0, 5); 
      animationContinue = true;
      break;

    case 141:
      if (dataReady || animationContinue){
        animation_breathing(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 142:
      if (dataReady || animationContinue){
        animation_gradient_flow(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4], dataLora[5], dataLora[6]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 143:
      if (dataReady || animationContinue){
        animation_scintillement(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 144:
      if (dataReady || animationContinue){
        animation_matrix(dataLora[0]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 145:
      if (dataReady || animationContinue){
        animation_explosion(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 146:
      if (dataReady || animationContinue){
        animation_vague(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4]);   
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 147:
      if (dataReady || animationContinue){
        defilement(dataLora[0], dataLora[1], dataLora[2], dataLora[3], dataLora[4], dataLora[5]);   // défilement avec une couleur
        dataReady = false;
        animationContinue = true;
      }
      break;
    case 148:
      if (dataReady){
        colone_couleur_static(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   // colone static avec une couleur
        dataReady = false;
        animationContinue = false;
      }
      break;
    case 149:
      if (dataReady){
        ligne_couleur_static(dataLora[0], dataLora[1], dataLora[2], dataLora[3]);   // ligne static avec une couleur
        dataReady = false;
        animationContinue = false;
      }
      break;
    case 150:
      if (dataReady){
        couleur_static(dataLora[0], dataLora[1], dataLora[2]);                      // uni avec une couleur static
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
      defilement(100, 0, 0, 200, 2, 3);
      animationContinue = true;
      break;
    case 153:       // mode pour renvoyer le niveau de batterie
      static uint8_t niveauBatterieToSend = 0; 
      niveauBatterieToSend = getPourcentageBatterie();
      delay(100);
      Radio.Send(&niveauBatterieToSend, sizeof(niveauBatterieToSend));
      delay(20);
      mode = 0;
      animationContinue = false;
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
      changementMode();
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
      int niveauBatterie = getPourcentageBatterie();
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
      changementMode();
    }
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
  uint8_t IdRecu = 0;
  dataReady = false;
  
  memcpy(&IdRecu, payload, sizeof(IdRecu)); // Lecture de l'ID
  if (IdRecu != deviceID) return;           // si on ne vise pas ce composant

  memcpy(&mode, payload + 1, sizeof(mode)); // Lecture du mode d'affichage

  if (mode == 1){
    // Vérif si assez de données pour la FFT
    uint16_t expected_size = 2 + (FFT_SIZE) * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 1 : Paquet tronqué, données FFT incomplètes !");
    } else {
      memcpy(dataLora, payload + 2, (FFT_SIZE) * sizeof(uint8_t));
    }
    affichage_musique(dataLora);
    dataReady = true;
  }

  if (mode == 151 || mode == 153){ // extraction de 0 variable
    dataReady = true;
  }
  if (mode == 200 || mode == 144){ // extraction de 1 variable
    uint16_t expected_size = 1 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 250 : Paquet tronqué, données incomplètes ! ");
    } else {
      memcpy(dataLora, payload + 2, 1 * sizeof(uint8_t));
      dataReady = true;
    }
  }
  if (mode == 150){ // extraction de 3 variables
    uint16_t expected_size = 3 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 249 : Paquet tronqué, données incomplètes ! ");
    } else {
      memcpy(dataLora, payload + 2, 3 * sizeof(uint8_t));
      dataReady = true;
    }
  }
  if (mode == 149 || mode == 148 || mode == 145 || mode == 141){ // extraction de 4 variables
    uint16_t expected_size = 4 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 248 : Paquet tronqué, données incomplètes !");
    } else {
      memcpy(dataLora, payload + 2, 4 * sizeof(uint8_t));
      dataReady = true;
    }
  }
  if (mode == 146 || mode == 143){ // extraction de 5 variables
    uint16_t expected_size = 5 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 247 : Paquet tronqué, données incomplètes !");
    } else {
      memcpy(dataLora, payload + 2, 5 * sizeof(uint8_t));
      dataReady = true;
    }
  }
  if (mode == 147){ // extraction de 6 variables
    uint16_t expected_size = 6 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 246 : Paquet tronqué, données incomplètes !");
    } else {
      memcpy(dataLora, payload + 2, 6 * sizeof(uint8_t));
      dataReady = true;
    }
  }
  if (mode == 142){ // extraction de 7 variables
    uint16_t expected_size = 7 * sizeof(uint8_t);
    if (size < expected_size) {
      Serial.println("ERREUR 247 : Paquet tronqué, données incomplètes !");
    } else {
      memcpy(dataLora, payload + 2, 7 * sizeof(uint8_t));
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
  if (dataReady) changementMode();
}

void routineStop() {
  frame = 0;
  for (int k=0; k<25; ++k){
    animation_vague(200, 0, 30, 20, 1);
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
    animation_vague(0, 20, 200, 20, 0);
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
void couleur_static(int R, int G, int B){ // Affichage d'une couleur static
  for (int x = 0; x < WIDTH; x++) {
    for (int y = 0; y < HEIGHT; y++) {
      setPixelColor(x, y, R, G, B); 
    }
  }
  strip.show();
}
void ligne_couleur_static(int R, int G, int B, int numLigne){ // Affichage d'une ligne avec une couleur static
  for (int x = 0; x < WIDTH; x++) {
    setPixelColor(x, numLigne, R, G, B); 
  }
  delay(15);
  strip.show();
}
void colone_couleur_static(int R, int G, int B, int numcolone){ // Affichage d'une colone avec une couleur static
  for (int y = 0; y < HEIGHT; y++) {
    setPixelColor(numcolone, y, R, G, B); 
  }
  delay(15);
  strip.show();
}
void defilement(uint8_t R, uint8_t G, uint8_t B, int vitesse, uint8_t sens, uint8_t trainee) {
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
      colone_couleur_static(R, G, B, x_actif);
      break;
    }

    case 1: { // Gauche
      uint8_t x_actif = (9 - (frame % 10));
      colone_couleur_static(R, G, B, x_actif);
      break;
    }

    case 2: { // Bas
      uint8_t y_actif = frame % 10;
      ligne_couleur_static(R, G, B, y_actif);
      break;
    }

    case 3: { // Haut
      uint8_t y_actif = (9 - (frame % 10));
      ligne_couleur_static(R, G, B, y_actif);
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
void animation_vague(uint8_t R, uint8_t G, uint8_t B, int vitesse, uint8_t sens) {
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
void animation_explosion(uint8_t R, uint8_t G, uint8_t B, int vitesse) {
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
void animation_matrix(int vitesse) {
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
void animation_scintillement(uint8_t R, uint8_t G, uint8_t B, int vitesse, uint8_t remplissage) {
  for(int i=1; i<NUM_LEDS; i++){
    if(random(0,100)<remplissage) strip.setPixelColor(i, R, G ,B);
    else strip.setPixelColor(i,0,0,0);
  }
  strip.show();
  delay(vitesse);
}
void animation_gradient_flow(uint8_t R_1, uint8_t G_1, uint8_t B_1, uint8_t R_2, uint8_t G_2, uint8_t B_2, int vitesse) {
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
void animation_breathing(uint8_t R, uint8_t G, uint8_t B, int vitesse) {
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

void animation_noise(int vitesse) {
  for (uint8_t y = 0; y < HEIGHT; y++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      uint8_t r = random(255);
      uint8_t g = random(255);
      uint8_t b = random(255);
      setPixelColor(x, y, r, r, r); 
    }
  }
  strip.show();
  delay(vitesse);
}