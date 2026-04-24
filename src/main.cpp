#include <Arduino.h>
#include "esp_adc_cal.h"
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>
#include <WiFi.h>

// --- PIN MAPPING ---
const int sdaPin = 1;
const int sclPin = 0;
const int int1Pin = 21; // Jetzt digitaler Input für IMU Interrupt
const int buzzerPin = 3;
const int radarPin = 4;

// NEUE PINS (Buttons)
const int btn1Pin = 8; // Links (Modus wechseln / Entsperren halten) - Verschoben von Pin 5
const int btn2Pin = 6; // Mitte (Modus umschalten)
const int btn3Pin = 7; // Rechts (Batterie anzeigen / Entsperren drücken) 

// NEUE PINS (Hardware)
const int batPin = 2;  // ADC Pin (ADC1_CH2 - stabil!)
const int ledLeftPin = 10;
const int ledCenterPin = 20;
const int ledRightPin = 5;

// --- SENSOR OBJEKTE ---
Adafruit_LSM6DS3 lsm;
uint8_t sensorAddr = 0x6A;

// --- SYSTEM VARIABLEN ---
enum SystemMode { ALARM_MODE, BACKLIGHT_MODE };
SystemMode currentMode = ALARM_MODE;

// Alarm Status Variablen
bool radarWasActive = false;
unsigned long lastVibrationTime = 0;
unsigned long buzzerOffTime = 0;
unsigned long modeStartTime = 0;
const long radarWarmupTime = 3000; // 3 Sek. warten nach Schärfen
const long vibrationCooldown = 500;

// Button State Variablen
bool btn1Last = false;
bool btn2Last = false;
bool btn3Last = false;
int unlockComboCount = 0;

// Beleuchtungs/Batterie Variablen
int lightingMode = 0; 
bool lightsEnabled = true;
bool showBat = false;
float batLevel = 0.0;
unsigned long batStart = 0;
const unsigned long BAT_DISPLAY_TIME = 3000;

// --- HILFSFUNKTIONEN ---

void setLEDs(bool l, bool c, bool r) {
  digitalWrite(ledLeftPin, l ? HIGH : LOW);
  digitalWrite(ledCenterPin, c ? HIGH : LOW);
  digitalWrite(ledRightPin, r ? HIGH : LOW);
}

void beep(int duration) {
  digitalWrite(buzzerPin, HIGH);
  buzzerOffTime = millis() + duration;
}

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(sensorAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void resetSensorFlags() {
  Wire.beginTransmission(sensorAddr); 
  Wire.write(0x1B); 
  Wire.endTransmission(false);
  Wire.requestFrom(sensorAddr, (uint8_t)1); 
  Wire.read();
  
  sensors_event_t a, g, t; 
  lsm.getEvent(&a, &g, &t);
}

void handleBuzzer(unsigned long currentMillis) {
  if (buzzerOffTime > 0 && currentMillis >= buzzerOffTime) {
    digitalWrite(buzzerPin, LOW);
    buzzerOffTime = 0;
  }
}

void processArmedState(unsigned long currentMillis, bool rawRadar, bool isVibrationActive) {
  bool isRadarActive = rawRadar && (currentMillis - modeStartTime > radarWarmupTime);

  // 1. Radar prüfen (mit kleiner Hysterese/Logik)
  if (isRadarActive && !radarWasActive) {
    Serial.println("Radar: Bewegung!");
    beep(20);
    radarWasActive = true;
  } else if (!isRadarActive) {
    radarWasActive = false;
  }

  // 2. Erschütterung prüfen
  if (isVibrationActive && (currentMillis - lastVibrationTime >= vibrationCooldown)) {
    Serial.println("IMU: Erschütterung!");
    beep(300);
    lastVibrationTime = currentMillis;
    resetSensorFlags();
  }
}

void processDisarmedState(bool isVibrationActive) {
  if (isVibrationActive) {
    resetSensorFlags();
  }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- ALARM + BACKLIGHT SYSTEM ---");

  // WiFi deaktivieren, um ADC2 (Pin 5) nutzen zu können
  WiFi.mode(WIFI_OFF);
  btStop(); // Optional: Bluetooth ebenfalls aus für maximale Stabilität

  // Pins initialisieren
  pinMode(buzzerPin, OUTPUT);
  pinMode(radarPin, INPUT);
  
  pinMode(btn1Pin, INPUT_PULLUP);
  pinMode(btn2Pin, INPUT_PULLUP);
  pinMode(btn3Pin, INPUT_PULLUP);
  pinMode(int1Pin, INPUT_PULLUP);

  pinMode(ledLeftPin, OUTPUT);
  pinMode(ledCenterPin, OUTPUT);
  pinMode(ledRightPin, OUTPUT);
  pinMode(batPin, INPUT); // Sicherstellen, dass Pin 5 als Input definiert ist

  // Bootup Signal
  setLEDs(true, true, true);
  digitalWrite(buzzerPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, LOW);
  
  // Analog Read Auflösung auf 12-Bit (ESP32 Standard: 0-4095)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db); // Ermöglicht Messungen bis ca. 3.1V am Pin

  // I2C für den LSM6DS3 starten
  Wire.begin(sdaPin, sclPin, 100000);
  
  if (lsm.begin_I2C(0x6A)) {
    sensorAddr = 0x6A;
  } else if (lsm.begin_I2C(0x6B)) {
    sensorAddr = 0x6B;
  } else {
    Serial.println("FEHLER: LSM6DS3 nicht gefunden!");
  }

  // Sensor konfigurieren (Wakeup auf INT1)
  writeReg(0x10, 0x40); // 104Hz
  writeReg(0x5B, 0x01); // MAX EMPFINDLICHKEIT
  writeReg(0x58, 0x80); // Int Enable
  writeReg(0x5E, 0x20); // Wakeup -> INT1

  beep(100); delay(100); beep(100);
  Serial.println("System bereit. (Alarm Modus aktiv)");
}

// --- HAUPTSCHLEIFE ---
void loop() {
  unsigned long m = millis();

  handleBuzzer(m);

  // Sensor- & Buttonstatus (LOW = Button gedrückt wegen INPUT_PULLUP)
  bool isRadarActive = (digitalRead(radarPin) == HIGH);
  bool isVibrationActive = (digitalRead(int1Pin) == HIGH);
  
  bool btn1 = (digitalRead(btn1Pin) == LOW);
  bool btn2 = (digitalRead(btn2Pin) == LOW);
  bool btn3 = (digitalRead(btn3Pin) == LOW);

  // Flankenerkennung Buttons
  bool btn3Pressed = (btn3 && !btn3Last);
  btn3Last = btn3;

  bool btn2Pressed = (btn2 && !btn2Last);
  btn2Last = btn2;

  bool btn1Pressed = (btn1 && !btn1Last);
  bool btn1Released = (!btn1 && btn1Last);
  btn1Last = btn1;

  // --- BUTTON LOGIK ---

  // Button 2 (Mitte) nur zum Schärfen (ALARM_MODE) erlauben, nicht zum Entschärfen
  if (btn2Pressed && currentMode == BACKLIGHT_MODE) {
    currentMode = ALARM_MODE;
    Serial.println("Modus: ALARM");
    modeStartTime = m;
    beep(100); delay(100); beep(100);
    setLEDs(0, 0, 0);
    showBat = false;
  }

  // Kombination: Wenn Btn1 gehalten wird und Btn3 gedrückt wird
  if (btn1) {
    if (btn3Pressed) {
      unlockComboCount++;
    }
  }

  // Wenn Btn1 losgelassen wird auswerten
  if (btn1Released) {
    if (unlockComboCount == 2) {
      // Modus umschalten
      if (currentMode == ALARM_MODE) {
        currentMode = BACKLIGHT_MODE;
        Serial.println("Modus: BACKLIGHT");
        beep(200); 
      } else {
        currentMode = ALARM_MODE;
        Serial.println("Modus: ALARM");
        modeStartTime = m;
        beep(100); delay(100); beep(100);
      }
      setLEDs(0, 0, 0); // LEDs zurücksetzen
      lightsEnabled = true;
      showBat = false;
    } 
    // Wenn Btn1 ganz normal (ohne Kombination) im Backlight Modus gedrückt & losgelassen wurde
    else if (unlockComboCount == 0 && currentMode == BACKLIGHT_MODE) {
      lightingMode = (lightingMode + 1) % 4;
    }
    unlockComboCount = 0; // Reset
  }

  // Normaler Btn3 Klick im Backlight Modus (Licht an/aus + Batterie anzeigen)
  if (btn3Pressed && !btn1 && currentMode == BACKLIGHT_MODE) {
    showBat = true;
    lightsEnabled = !lightsEnabled; // Licht umschalten
    beep(50);
    batStart = m;
    
    // Mehrfache Messung für Stabilität
    int rawVal = 0;
    for(int i=0; i<10; i++) {
      rawVal += analogRead(batPin);
      delay(2);
    }
    rawVal /= 10;

    float adcVoltage = (rawVal / 4095.0) * 3.3;
    
    Serial.print("Raw ADC: ");
    Serial.print(rawVal);
    Serial.print(" | ADC Voltage: ");
    Serial.print(adcVoltage);

    batLevel = adcVoltage * 1.385; // Calibrated: 3.6V Battery = ~2.6V ADC
    
    Serial.print("Batterie Level: ");
    Serial.print(batLevel);
    Serial.println("V");
  }


  // --- MODUS AUSFÜHRUNG ---
  
  if (currentMode == ALARM_MODE) {
    processArmedState(m, isRadarActive, isVibrationActive);
    
    // Alarm LEDs: 
    // Links = Radar aktiv (Blinkt alle 150ms: 100ms aus, 50ms an). 
    // Rechts = IMU Erschütterung (500ms langanhaltend)
    // Mitte = Status-Blitz (alle 5 Sek für 50ms)
    bool radarBlink = isRadarActive && (m % 400 >= 370);
    bool rightBlink = ((m - lastVibrationTime) < 500) && (m % 100 < 50);
    bool centerStatus = (m % 5000 < 50);
    
    setLEDs(radarBlink, centerStatus, rightBlink);
  } 
  else { // BACKLIGHT_MODE
    processDisarmedState(isVibrationActive);
    
    if (showBat) {
      bool blink = (m % 400 < 200);
      bool fBl = (m % 200 < 100);
      
      if (batLevel >= 4.1)      setLEDs(1, 1, 1);
      else if (batLevel >= 4.0) setLEDs(1, 1, blink);
      else if (batLevel >= 3.9) setLEDs(1, 1, 0);
      else if (batLevel >= 3.8) setLEDs(1, blink, 0);
      else if (batLevel >= 3.7) setLEDs(1, 0, 0);
      else if (batLevel >= 3.6) setLEDs(blink, 0, 0);
      else                      setLEDs(fBl, 0, 0);

      // Timeout für Batterie-Anzeige
      if (m - batStart >= BAT_DISPLAY_TIME) {
        showBat = false;
        setLEDs(0, 0, 0);
      }
    } 
    else if (lightsEnabled) {
      // Normale Backlight Lichtmodi ausführen
      if (lightingMode == 0) {
        // Alle 3 blinken alle 250ms (250 an, 250 aus = 500ms periode)
        bool state = (m % 500 < 250);
        setLEDs(state, state, state);
      } 
      else if (lightingMode == 1) {
        // 1 LED rotiert mit 3-LED Pause
        int step = (m / 300) % 4; // L, C, R, Off
        setLEDs(step == 0, step == 1, step == 2);
      } 
      else if (lightingMode == 2) {
        // 2 LEDs rotieren mit kurzer Pause
        int step = (m / 200) % 5;
        if (step == 0)      setLEDs(1, 0, 0); // Nur Links
        else if (step == 1) setLEDs(1, 1, 0); // Links + Mitte
        else if (step == 2) setLEDs(0, 1, 1); // Mitte + Rechts
        else if (step == 3) setLEDs(0, 0, 1); // Nur Rechts
        else                setLEDs(0, 0, 0); // Kurze Pause (Gap)
      }
      else if (lightingMode == 3) {
        // LED läuft schnell hin und her (KITT-Style)
        int step = (m / 50) % 4;
        if (step == 0)      setLEDs(1, 0, 0);
        else if (step == 1) setLEDs(0, 1, 0);
        else if (step == 2) setLEDs(0, 0, 1);
        else                setLEDs(0, 1, 0);
      }
    }
    else {
      setLEDs(0, 0, 0);
    }
  }

  delay(10); // Leichtes Delay für System-Stabilität und Button Debouncing
}