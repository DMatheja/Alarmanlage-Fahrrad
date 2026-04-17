#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <WiFi.h>

// --- Konfiguration ---
#define DEBUG_MODE 0 // Auf 1 setzen für Serial Monitor (kostet Strom)

#if DEBUG_MODE
  #define DEBUG_BEGIN(x)   Serial.begin(x); delay(500)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_BEGIN(x)
  #define DEBUG_PRINTLN(x)
#endif

// Pins
const int batPin = 8, radarPin = 4, int1Pin = 2, buzzPin = 3;
const int sdaPin = 0, sclPin = 1; // ACHTUNG: Falls Serial nicht geht, liegen hier TX/RX
const int btnPins[3] = {5, 6, 7}, ledPins[3] = {10, 20, 21}; 

// Zeiten
const unsigned long RADAR_COOLDOWN     = 5000; // Alarm alle 5 Sekunden
const unsigned long IMU_COOLDOWN       = 500;
const unsigned long BAT_DISPLAY_TIME   = 5000;
const unsigned long HEARTBEAT_INTERVAL = 3000;

// Status
bool isArmed = true, showBat = false, wasBtn1 = false, sleepCmd = false, sleepHld = false;
int unlockCnt = 0, ledMode = 1, ledStep = 0;
unsigned long batStart = 0, lastAnim = 0, l1End = 0, l3End = 0, lastHb = 0, lastRad = 0, lastImu = 0, slpStart = 0;
float batLevel = 0.0;
uint8_t snsrAddr = 0x6A;
Adafruit_LSM6DS3 lsm;

// --- Strukturen ---
struct Debouncer {
    int pin; bool st = 1, last = 1; unsigned long t = 0;
    void init(int p) { pinMode(pin = p, INPUT_PULLUP); }
    bool upd(bool &pr, bool &rel) {
        bool r = digitalRead(pin); pr = rel = 0;
        if (r != last) t = millis();
        if (millis() - t > 30 && r != st) { st = r; st == LOW ? pr = 1 : rel = 1; }
        return last = r, st;
    }
} btns[3];

struct Buzzer {
    unsigned long nxt = 0, end = 0; int bps = 0, d = 0, p = 0; bool on = 0;
    void play(int c, int _d, int _p = 0) { bps = c; d = _d; p = _p; nxt = millis(); }
    void upd(unsigned long m) {
        if (bps > 0 && m >= nxt && !on) { digitalWrite(buzzPin, 1); on = 1; end = m + d; bps--; }
        if (on && m >= end) { digitalWrite(buzzPin, 0); on = 0; nxt = m + p; }
    }
} buzz;

// --- Funktionen ---
void setLEDs(bool l1, bool l2, bool l3) { 
    digitalWrite(ledPins[0], l1); digitalWrite(ledPins[1], l2); digitalWrite(ledPins[2], l3); 
}

void beep(int d) { digitalWrite(buzzPin, 1); delay(d); digitalWrite(buzzPin, 0); }

void beeps(int n, int d, int p, int endD = 0) { 
    for(int i=0; i<n; i++) { beep(d); delay(p); } 
    if(endD) beep(endD); 
}

void reg(uint8_t r, uint8_t v) { 
    Wire.beginTransmission(snsrAddr); Wire.write(r); Wire.write(v); Wire.endTransmission(); 
}

void rstSnsr() { 
    reg(0x1B, 0); Wire.requestFrom(snsrAddr, (uint8_t)1); Wire.read(); 
    sensors_event_t a, g, t; lsm.getEvent(&a, &g, &t); 
}

void trigBat() { 
    batLevel = analogReadMilliVolts(batPin) / 500.0; // Annahme: Teiler 1:2
    showBat = true; batStart = millis(); 
}

void render(unsigned long m);

void enterSleep() {
    trigBat(); unsigned long s = millis();
    while(millis() - s < 5000) { render(millis()); delay(10); }
    setLEDs(0, 0, 0); beep(800);
    while(!digitalRead(btnPins[0]) || !digitalRead(btnPins[1]) || !digitalRead(btnPins[2])) delay(10);
    gpio_pullup_en((gpio_num_t)btnPins[0]); 
    esp_deep_sleep_enable_gpio_wakeup(1ULL << btnPins[0], ESP_GPIO_WAKEUP_GPIO_LOW);
    esp_deep_sleep_start();
}

void setup() {
    DEBUG_BEGIN(115200);
    WiFi.mode(WIFI_OFF); btStop();
    pinMode(buzzPin, OUTPUT); pinMode(radarPin, INPUT_PULLDOWN); pinMode(int1Pin, INPUT_PULLUP);
    for(int i=0; i<3; i++) { pinMode(ledPins[i], OUTPUT); btns[i].init(btnPins[i]); }
    Wire.begin(sdaPin, sclPin, 100000); 
    snsrAddr = lsm.begin_I2C(0x6A) ? 0x6A : 0x6B;
    reg(0x10, 0x40); reg(0x5B, 0x01); reg(0x58, 0x80); reg(0x5E, 0x20);
    analogReadResolution(12); analogSetAttenuation(ADC_11db);
    esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO ? beeps(3, 100, 100) : beeps(2, 100, 100);
}

void process(unsigned long m) {
    bool pr[3], rel[3], st[3];
    for(int i=0; i<3; i++) st[i] = !btns[i].upd(pr[i], rel[i]);

    if (!isArmed && st[0] && st[1] && st[2]) {
        if (!sleepHld) { sleepHld = true; slpStart = m; } 
        else if (m - slpStart >= 1000) enterSleep();
        sleepCmd = true; return;
    }
    sleepHld = false; if (!st[0] && !st[1] && !st[2]) sleepCmd = false;
    if (sleepCmd) return;

    if (isArmed) {
        if (pr[2]) trigBat();
        if (st[0] && pr[1]) unlockCnt++;
        if (!st[0] && wasBtn1) {
            if (unlockCnt >= 2) { isArmed = false; buzz.play(3, 40, 60); trigBat(); }
            unlockCnt = 0;
        }
        wasBtn1 = st[0];

        // Radar: Sofortige Auslösung (Invertierte Logik: LOW = Aktiv)
        if (digitalRead(radarPin) && (m - lastRad >= RADAR_COOLDOWN)) { 
            buzz.play(1, 50); lastRad = m; l3End = m + 150; 
        }
        
        // IMU: Mit Cooldown gegen Buzzer-Vibration
        if (digitalRead(int1Pin) && (m - lastImu >= IMU_COOLDOWN)) { 
            lastImu = m;
            if (m - lastRad < 300) { rstSnsr(); } // Vibration ignorieren
            else { buzz.play(1, 300); rstSnsr(); l1End = m + 300; }
        }
    } else {
        if (digitalRead(int1Pin) && (m - lastImu >= IMU_COOLDOWN)) { rstSnsr(); lastImu = m; }
        if (rel[0]) { isArmed = true; beeps(2, 100, 100, 600); rstSnsr(); }
        if (rel[1]) { ledMode = (ledMode + 1) % 3; ledStep = 0; }
        if (rel[2]) trigBat();
    }
    if (showBat && m - batStart >= BAT_DISPLAY_TIME) showBat = false;
}

void render(unsigned long m) {
    if (isArmed && (m < l1End || m < l3End)) { setLEDs(m < l1End, 0, m < l3End); return; }
    
    if (showBat) {
        bool blink = (m % 500 < 250);
        if      (batLevel >= 4.05) setLEDs(1, 1, 1);
        else if (batLevel >= 3.95) setLEDs(1, 1, blink);
        else if (batLevel >= 3.85) setLEDs(1, 1, 0);
        else if (batLevel >= 3.75) setLEDs(1, blink, 0);
        else if (batLevel >= 3.65) setLEDs(1, 0, 0);
        else if (batLevel >= 3.55) setLEDs(blink, 0, 0);
        else                       setLEDs(m%200<100, m%200<100, m%200<100); // Kritisch
        return;
    }
    
    if (isArmed) {
        if (m - lastHb >= HEARTBEAT_INTERVAL) { 
            setLEDs(0, 1, 0); 
            if (m - lastHb >= HEARTBEAT_INTERVAL + 50) lastHb = m; 
        } else setLEDs(0, 0, 0);
    } else {
        int dly = (ledMode == 0 ? 150 : (ledMode == 1 ? 250 : 80));
        if (m - lastAnim >= dly) { 
            lastAnim = m; 
            ledStep = (ledStep + 1) % (ledMode == 0 ? 6 : (ledMode == 1 ? 2 : 4)); 
        }
        if (ledMode == 0)      setLEDs(ledStep < 3, ledStep > 0 && ledStep < 4, ledStep > 1 && ledStep < 5);
        else if (ledMode == 1) setLEDs(!ledStep, !ledStep, !ledStep);
        else                   setLEDs(ledStep == 0, ledStep == 1, ledStep == 2);
    }
}

void loop() {
    unsigned long m = millis();
    buzz.upd(m); process(m); render(m);
}