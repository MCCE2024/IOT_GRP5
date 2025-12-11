/*
README Haus A Energie Monitor und Power Alert

Beschreibung
Dieses Programm bildet ein energieautarkes Haus mit dem Keyestudio Smart Home Kit und ESP32 nach.
Der Energiezustand wird als Batteriestand in Prozent modelliert. Nur aktive Verbraucher entladen
die Batterie. Fällt die Batterie unter vierzig Prozent, wechselt das Haus in einen Energiesparmodus
und alle Verbraucher werden abgeschaltet. LCD Anzeige und SK6812 Statusleiste bleiben immer aktiv
und zählen nicht als Verbraucher.

Steuerung über Taster und RFID
Button eins kurzer Druck wählt das nächste Gerät im Menü.
Button eins langer Druck wählt das vorherige Gerät im Menü.
Button zwei kurzer Druck schaltet das aktuell gewählte Gerät ein oder aus.
RFID Sensor öffnet die Tür, wenn die hinterlegte Karte erkannt wird und kein Energiesparmodus aktiv ist.
Der Menüpunkt Reset setzt die Batterie auf hundert Prozent und schaltet alle Verbraucher aus.

Verbraucher
Lüfter als elektrische Last mit kontinuierlichem Verbrauch bei Aktivität.
Tür über Servo, jede Bewegung verbraucht einmalig fünf Prozent Batterie.
Fenster über Servo, jede Bewegung verbraucht einmalig fünf Prozent Batterie.
Buzzer spielt eine einfache Super Mario Melodie, solange er im Menü aktiviert ist.

Energiesparlogik
Nur aktive Verbraucher reduzieren den Batteriestand, es gibt keinen Grundverbrauch.
Tür und Fenster reduzieren den Batteriestand nur einmal pro Bewegung.
Fällt die Batterie unter vierzig Prozent, wird der Energiesparmodus aktiv.
Im Energiesparmodus werden Lüfter, Tür, Fenster und Buzzer abgeschaltet.
Neue Verbraucher können im Energiesparmodus nicht eingeschaltet werden.
Der Reset Menüpunkt bleibt immer nutzbar und setzt den Zustand zurück.

Energieerzeugung
Sobald der Energiespeicher unter 40% fällt, wird der Energiesparmodus aktiviert und das Haus schaltet in den Energieerzeugungs-Modus.
Dies wird am LCD Display durch "charging mo" angezeigt. Der Lüfter wird aktiviert um per Windenergie Strom zu erzeugen.
Der Ladevorgang wird durch den SK6812 LED Streifen angezeigt.

Visualisierung
LCD Zeile eins zeigt den Batteriestand als ganze Zahl in Prozent.
LCD Zeile zwei zeigt das aktuell ausgewählte Gerät und dessen Status.
Der SK6812 LED Streifen visualisiert den Batteriestatus über Farben.
Grün bei mindestens sechzig Prozent Batterie.
Gelb zwischen vierzig und neunundfünfzig Prozent Batterie.
Rot unter vierzig Prozent Batterie bei aktivem Energiesparmodus.
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include "OneButton.h"
#include "MFRC522_I2C.h"
#include <BuzzerESP32.h>

// Hardwarebelegung
const int BTN1_PIN = 16;
const int BTN2_PIN = 27;

const int FAN_PIN1 = 19;
const int FAN_PIN2 = 18;

const int SK_PIN = 26;
const int SK_LED_COUNT = 4;

const int SERVO_DOOR_PIN = 4;
const int SERVO_WINDOW_PIN = 2;

const int BUZ_PIN = 25;

// LCD Modul
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID Modul
MFRC522_I2C mfrc522(0x28,5);

// SK6812
Adafruit_NeoPixel strip(SK_LED_COUNT, SK_PIN, NEO_GRB + NEO_KHZ800);

// Animation vars
int ledAnimIndex = 0;
unsigned long lastLedAnimTime = 0;
const unsigned long LED_ANIM_INTERVAL = 200;

// Servos
Servo servoDoor;
Servo servoWindow;
bool doorServoAttached = false;
bool windowServoAttached = false;

// Buzzer
BuzzerESP32 buzzer(BUZ_PIN);

// Tasterlogik
OneButton button1(BTN1_PIN, true);
OneButton button2(BTN2_PIN, true);

// Batteriemodell
float batteryPercent = 100.0;
unsigned long lastBatteryUpdate = 0;
const unsigned long BATTERY_INTERVAL_MS = 1000;

// Laufende Lasten
const float FAN_LOAD    = 3.0;
const float BUZZER_LOAD = 1.0;

// Einmalige Bewegungskosten
const float DOOR_MOVE_COST   = 5.0;
const float WINDOW_MOVE_COST = 5.0;

// Energiemodus
bool energySaveMode = false;
bool isCharging = false;

// Zustände der Verbraucher
bool fanOn      = false;
bool doorOpen   = false;
bool windowOpen = false;
bool buzzerOn   = false;

// Menüeinträge
enum Device {
  DEV_FAN = 0,
  DEV_DOOR = 1,
  DEV_WINDOW = 2,
  DEV_BUZZER = 3,
  DEV_RESET = 4,
  DEV_COUNT = 5
};

int selectedDevice = DEV_FAN;

// RFID Passwort aus UID 84 51 38 219
String rfidPassword = "";
String correctPassword = "845138219";

// Super Mario Motiv
const int marioNotes[] = {
  659, 659, 659, 523, 659, 784,
  0,
  523, 0, 392, 0, 330
};

const int marioDurations[] = {
  150, 150, 150, 150, 150, 300,
  300,
  300, 150, 300, 150, 400
};

const int MARIO_LENGTH = sizeof(marioNotes) / sizeof(marioNotes[0]);
int marioIndex = 0;
unsigned long marioNextChange = 0;
bool marioPlaying = false;

// Flag für Bewegungen ohne Batteriekosten, etwa bei Reset
bool suppressMoveCost = false;

// SK6812

void setAllPixels(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void handleChargingLed() {
  if (millis() - lastLedAnimTime < LED_ANIM_INTERVAL) {
    return;
  }
  lastLedAnimTime = millis();

  uint32_t color;
  if (batteryPercent < 45.0) {
    color = strip.Color(255, 0, 0); // Red
  } else if (batteryPercent < 80.0) {
    color = strip.Color(255, 165, 0); // Orange/Yellow
  } else {
    color = strip.Color(0, 255, 0); // Green
  }

  strip.clear();
  strip.setPixelColor(ledAnimIndex, color);
  strip.show();

  ledAnimIndex++;
  if (ledAnimIndex >= strip.numPixels()) {
    ledAnimIndex = 0;
  }
}

void updateStatusLeds() {
  if (isCharging) return;

  if (energySaveMode) {
    setAllPixels(strip.Color(255, 0, 0));
  } else if (batteryPercent < 60.0) {
    setAllPixels(strip.Color(255, 255, 0));
  } else {
    setAllPixels(strip.Color(0, 255, 0));
  }
}

// Lüfter nur digital, keine PWM, um Konflikte mit Servos zu vermeiden

void hardFanOff() {
  fanOn = false;
  digitalWrite(FAN_PIN1, LOW);
  digitalWrite(FAN_PIN2, LOW);
}

void applyFanState() {
  if (isCharging) {
    digitalWrite(FAN_PIN1, HIGH);
    digitalWrite(FAN_PIN2, LOW);
  } else if (fanOn && !energySaveMode) {
    digitalWrite(FAN_PIN1, HIGH);
    digitalWrite(FAN_PIN2, LOW);
  } else {
    hardFanOff();
  }
}

// Servos

void attachDoorServo() {
  if (!doorServoAttached) {
    servoDoor.setPeriodHertz(50);
    servoDoor.attach(SERVO_DOOR_PIN, 1000, 2000);
    doorServoAttached = true;
  }
}

void detachDoorServo() {
  if (doorServoAttached) {
    servoDoor.detach();
    doorServoAttached = false;
  }
}

void attachWindowServo() {
  if (!windowServoAttached) {
    servoWindow.setPeriodHertz(50);
    servoWindow.attach(SERVO_WINDOW_PIN, 1000, 2000);
    windowServoAttached = true;
  }
}

void detachWindowServo() {
  if (windowServoAttached) {
    servoWindow.detach();
    windowServoAttached = false;
  }
}

// Türbewegung

void moveDoorOnce(bool withCost) {
  attachDoorServo();
  if (!energySaveMode && doorOpen) {
    servoDoor.write(180);
  } else {
    servoDoor.write(0);
  }
  if (!energySaveMode && withCost && !suppressMoveCost) {
    batteryPercent -= DOOR_MOVE_COST;
    if (batteryPercent < 0.0) batteryPercent = 0.0;
  }
  delay(300);
  detachDoorServo();
}

// Fensterbewegung

void moveWindowOnce(bool withCost) {
  attachWindowServo();
  if (!energySaveMode && windowOpen) {
    servoWindow.write(180);
  } else {
    servoWindow.write(0);
  }
  if (!energySaveMode && withCost && !suppressMoveCost) {
    batteryPercent -= WINDOW_MOVE_COST;
    if (batteryPercent < 0.0) batteryPercent = 0.0;
  }
  delay(300);
  detachWindowServo();
}

// Buzzer

void stopBuzzerTone() {
  marioPlaying = false;
  buzzerOn = false;
  buzzer.playTone(0, 0);
}

void startMario() {
  marioIndex = 0;
  marioNextChange = 0;
  marioPlaying = true;
}

void updateMarioMelody() {
  if (!buzzerOn || energySaveMode) {
    if (marioPlaying) {
      buzzer.playTone(0, 0);
      marioPlaying = false;
    }
    return;
  }

  unsigned long now = millis();

  if (!marioPlaying) {
    startMario();
  }

  if (now < marioNextChange) {
    return;
  }

  int freq = marioNotes[marioIndex];
  int dur  = marioDurations[marioIndex];

  buzzer.playTone(freq, dur);

  marioNextChange = now + dur;
  marioIndex++;
  if (marioIndex >= MARIO_LENGTH) {
    marioIndex = 0;
  }
}

// LCD

void updateLcd() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print((int)batteryPercent);
  lcd.print(" %");
  if (isCharging) {
    lcd.print(" charging mode active");
  }

  lcd.setCursor(0, 1);
  switch (selectedDevice) {
    case DEV_FAN:
      lcd.print("Fan ");
      lcd.print(fanOn ? "ON " : "OFF");
      break;
    case DEV_DOOR:
      lcd.print("Door ");
      lcd.print(doorOpen ? "OPEN" : "CLOSE");
      break;
    case DEV_WINDOW:
      lcd.print("Win ");
      lcd.print(windowOpen ? "OPEN" : "CLOSE");
      break;
    case DEV_BUZZER:
      lcd.print("Buzz ");
      lcd.print(buzzerOn ? "ON " : "OFF");
      break;
    case DEV_RESET:
      lcd.print("Reset Ready");
      break;
  }
}

// Reset

void resetSystem() {
  energySaveMode = false;
  isCharging = false;

  fanOn      = false;
  doorOpen   = false;
  windowOpen = false;
  buzzerOn   = false;
  marioPlaying = false;

  hardFanOff();
  buzzer.playTone(0, 0);

  suppressMoveCost = true;
  doorOpen = false;
  windowOpen = false;
  moveDoorOnce(false);
  moveWindowOnce(false);
  suppressMoveCost = false;

  batteryPercent = 100.0;

  updateStatusLeds();
  updateLcd();
}

// Batterie

void updateBattery() {
  unsigned long now = millis();
  if (now - lastBatteryUpdate < BATTERY_INTERVAL_MS) {
    return;
  }
  lastBatteryUpdate = now;

  if (batteryPercent < 40.0) {
    isCharging = true;
  }

  if (isCharging) {
    batteryPercent += 5.0;
    if (batteryPercent >= 100.0) {
      batteryPercent = 100.0;
      isCharging = false;
    }
  } else {
    float consumption = 0.0;

    if (fanOn && !energySaveMode) {
      consumption += FAN_LOAD;
    }
    if (buzzerOn && !energySaveMode) {
      consumption += BUZZER_LOAD;
    }

    batteryPercent -= consumption;
  }

  if (batteryPercent > 100.0) batteryPercent = 100.0;
  if (batteryPercent < 0.0)   batteryPercent = 0.0;

  bool prevEnergySave = energySaveMode;
  if (batteryPercent < 40.0) {
    energySaveMode = true;
  } else {
    energySaveMode = false;
  }

  if (!prevEnergySave && energySaveMode) {
    bool doorWasOpen   = doorOpen;
    bool windowWasOpen = windowOpen;
    bool buzzerWasOn   = buzzerOn;

    fanOn      = false;
    doorOpen   = false;
    windowOpen = false;
    buzzerOn   = false;
    marioPlaying = false;

    hardFanOff();
    buzzer.playTone(0, 0);

    suppressMoveCost = true;
    if (doorWasOpen) {
      moveDoorOnce(false);
    }
    if (windowWasOpen) {
      moveWindowOnce(false);
    }
    suppressMoveCost = false;
  }

  updateStatusLeds();
  updateLcd();
  applyFanState();
}

// Menü

void selectNextDevice() {
  selectedDevice++;
  if (selectedDevice >= DEV_COUNT) {
    selectedDevice = 0;
  }
  updateLcd();
}

void selectPrevDevice() {
  if (selectedDevice == 0) {
    selectedDevice = DEV_COUNT - 1;
  } else {
    selectedDevice--;
  }
  updateLcd();
}

void toggleSelectedDevice() {
  if (selectedDevice != DEV_RESET && energySaveMode) {
    return;
  }

  switch (selectedDevice) {
    case DEV_FAN:
      fanOn = !fanOn;
      applyFanState();
      break;

    case DEV_DOOR:
      hardFanOff();
      doorOpen = !doorOpen;
      moveDoorOnce(true);
      break;

    case DEV_WINDOW:
      hardFanOff();
      windowOpen = !windowOpen;
      moveWindowOnce(true);
      break;

    case DEV_BUZZER:
      hardFanOff();
      if (buzzerOn) {
        stopBuzzerTone();
      } else {
        buzzerOn = true;
        startMario();
      }
      break;

    case DEV_RESET:
      resetSystem();
      break;
  }

  updateLcd();
}

// Buttons

void onBtn1Click() {
  selectNextDevice();
}

void onBtn1LongPress() {
  selectPrevDevice();
}

void onBtn2Click() {
  toggleSelectedDevice();
}

// RFID

void handleRfid() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    rfidPassword = "";
    return;
  }

  rfidPassword = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    rfidPassword += String(mfrc522.uid.uidByte[i]);
  }

  if (!energySaveMode && rfidPassword == correctPassword) {
    doorOpen = true;
    moveDoorOnce(true);
    updateLcd();
  }

  rfidPassword = "";
}

// Setup

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  strip.begin();
  strip.show();
  strip.setBrightness(50);

  pinMode(FAN_PIN1, OUTPUT);
  pinMode(FAN_PIN2, OUTPUT);
  hardFanOff();

  buzzer.setTimbre(30);
  buzzer.playTone(0, 0);
  marioPlaying = false;

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  button1.attachClick(onBtn1Click);
  button1.attachLongPressStop(onBtn1LongPress);
  button2.attachClick(onBtn2Click);

  Wire.begin();
  mfrc522.PCD_Init();

  updateStatusLeds();
  updateLcd();
  lastBatteryUpdate = millis();
}

// Loop

void loop() {
  button1.tick();
  button2.tick();

  handleRfid();
  updateBattery();
  updateMarioMelody();

  if (isCharging) {
    handleChargingLed();
  }

  delay(10);
}