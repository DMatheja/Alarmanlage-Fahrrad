#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS3.h>

// --- PIN MAPPING ---
const int sdaPin = 1;
const int sclPin = 0;
const int int1Pin = 2;
const int buzzerPin = 3;
const int radarPin = 4;

// NEUE PINS
const int taillightPin = 5; // Pin für das Rücklicht (LED oder Relais/Mosfet)
const int switchPin = 10;   // Pin für den Deaktivierungs-Schalter (Schalter nach GND)

// --- SENSOR OBJEKTE ---
Adafruit_LSM6DS3 lsm;
uint8_t sensorAddr = 0x6A;

// --- HILFSFUNKTIONEN ---

// Status-Variable für Radar-Verriegelung
bool radarWasActive = false;

void beep(int duration) {
  digitalWrite(buzzerPin, HIGH);
  delay(duration);
  digitalWrite(buzzerPin, LOW);
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

// --- SETUP ---
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- BASIC ALARM + LICHT START ---");

  // Pins initialisieren
  pinMode(buzzerPin, OUTPUT);
  pinMode(taillightPin, OUTPUT);
  pinMode(radarPin, INPUT);
  
  // Der Schalter-Pin nutzt den internen Pullup-Widerstand.
  // Das bedeutet: Ist der Schalter OFFEN, liest der Pin HIGH.
  // Ist der Schalter GESCHLOSSEN (mit GND verbunden), liest der Pin LOW.
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(int1Pin, INPUT_PULLUP);

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
  Serial.println("System bereit.");
}

// --- HAUPTSCHLEIFE ---
void loop() {
  // Schalterstatus auslesen (LOW = Schalter ist zu / mit GND verbunden = SCHARF)
  bool isArmed = (digitalRead(switchPin) == LOW);
  
  // Sensoren auslesen
  bool isRadarActive = (digitalRead(radarPin) == HIGH);
  bool isVibrationActive = (digitalRead(int1Pin) == HIGH);

  if (isArmed) {
    // ALARM IST SCHARF
    digitalWrite(taillightPin, LOW); // Rücklicht aus beim Parken
    
    // 1. Radar prüfen
    if (isRadarActive && !radarWasActive) {
      Serial.println("Radar: Bewegung!");
      beep(50);
      radarWasActive = true;
    } else if (!isRadarActive) {
      radarWasActive = false;
    }

    // 2. Erschütterung prüfen
    if (isVibrationActive) {
      Serial.println("IMU: Erschütterung!");
      // Wenn das Rücklicht beim Alarm blinken soll, könnte man das hier tun:
      digitalWrite(taillightPin, HIGH);
      beep(300); 
      digitalWrite(taillightPin, LOW);
      
      resetSensorFlags();
      delay(500);
    }
    
  } else {
    // ALARM IST DEAKTIVIERT (Du fährst Fahrrad)
    digitalWrite(taillightPin, HIGH); // Rücklicht dauerhaft an
    
    // Wir müssen den Beschleunigungssensor trotzdem zurücksetzen,
    // falls er während der Fahrt auslöst, damit er später wieder funktioniert.
    if (isVibrationActive) {
      resetSensorFlags();
    }
  }

  delay(20);
}