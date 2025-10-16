#include "Arduino.h"
#include "LoRaWan_APP.h"

// Durées en millisecondes
#define timetillsleep   5000       // reste éveillée 5 secondes
#define timetillwakeup  300000     // sommeil 300 000 ms = 5 minutes

static TimerEvent_t sleepTimer;
static TimerEvent_t wakeUpTimer;
uint8_t lowpower = 1;

void measureBattery() {
    float voltage = getBatteryVoltage() / 1000.0; // mV → V
    int percent = map(voltage * 1000, 3300, 4200, 0, 100);
    percent = constrain(percent, 0, 100);

    Serial.printf("\n--- Mesure batterie ---\n");
    Serial.printf("Tension : %.3f V\n", voltage);
    Serial.printf("Charge estimée : %d %%\n", percent);
    Serial.println("-----------------------\n");
}

// ---- Fonction appelée quand on entre en veille ----
void onSleep() {
    Serial.printf("→ Passage en veille pendant %d ms (%.1f min)...\r\n", timetillwakeup, timetillwakeup / 60000.0);
    lowpower = 1;

    // Démarre le timer de réveil
    TimerSetValue(&wakeUpTimer, timetillwakeup);
    TimerStart(&wakeUpTimer);

    // Stoppe le timer de sommeil
    TimerStop(&sleepTimer);
}

// ---- Fonction appelée quand on se réveille ----
void onWakeUp() {
    Serial.println("↗ Réveil du mode veille !");
    lowpower = 0;

    // Mesure la batterie au réveil
    measureBattery();

    // Après quelques secondes éveillé, retour en veille
    TimerSetValue(&sleepTimer, timetillsleep);
    TimerStart(&sleepTimer);
}

// ---- Initialisation ----
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Démarrage du programme de mesure batterie (CubeCell)");
    Radio.Sleep(); // met le modem LoRa en veille

    // Initialise les timers de gestion du mode veille
    TimerInit(&sleepTimer, onSleep);
    TimerInit(&wakeUpTimer, onWakeUp);

    // Démarre la première séquence (on s’endort immédiatement après setup)
    onSleep();
}

// ---- Boucle principale ----
void loop() {
    if (lowpower)
        lowPowerHandler(); // gestion du sommeil profond

    delay(400);
}
