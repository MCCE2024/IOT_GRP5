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

Visualisierung
LCD Zeile eins zeigt den Batteriestand als ganze Zahl in Prozent.
LCD Zeile zwei zeigt das aktuell ausgewählte Gerät und dessen Status.
Der SK6812 LED Streifen visualisiert den Batteriestatus über Farben.
Grün bei mindestens sechzig Prozent Batterie.
Gelb zwischen vierzig und neunundfünfzig Prozent Batterie.
Rot unter vierzig Prozent Batterie bei aktivem Energiesparmodus.

MQTT Integration
Der Batteriestand wird als ganze Zahl in Prozent an ein MQTT Topic gesendet.
Es wird nur gesendet, wenn sich der sichtbare Prozentwert ändert.
Nach Tür, Fenster und Reset wird sofort ein Update publiziert.
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include "OneButton.h"
#include "MFRC522_I2C.h"
#include <BuzzerESP32.h>
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

char msg[50];

// WLAN Zugangsdaten
const char* ssid = "";
const char* password = "";

// MQTT Broker Details
const char* mqtt_server = "194.182.170.71";
const int mqtt_port = 1883;
const char* mqtt_client_id = "Keyestudio-Haus-A";
const char* mqtt_topic_battery = "keyestudio/hausA/battery";

// Batteriestand als Integer letzter gesendeter Wert
int batteryLastSent = -1;

// MQTT reconnect Steuerung
unsigned long lastMqttReconnectAttempt = 0;

// Hardwarebelegung
const int BTN1_PIN = 16;
const int BTN2_PIN = 27;

const int FAN_PIN1 = 19;
const int FAN_PIN2 = 18;

const int SK_PIN = 26;
const int SK_LED_COUNT = 4;

const int SERVO_DOOR_PIN = 13;
const int SERVO_WINDOW_PIN = 5;

const int BUZ_PIN = 25;

// LCD Modul
LiquidCrystal_I2C lcd(0x27, 16, 2);

// RFID Modul
MFRC522 mfrc522(0x28);

// SK6812
Adafruit_NeoPixel strip(SK_LED_COUNT, SK_PIN, NEO_GRB + NEO_KHZ800);

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

void updateStatusLeds() {
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
  if (fanOn && !energySaveMode) {
    digitalWrite(FAN_PIN1, HIGH);
    digitalWrite(FAN_PIN2, LOW);
  } else {
    hardFanOff();
  }
}

// MQTT Publish Logik

void publishBatteryIfNeeded() {
  if (!client.connected()) return;

  int batteryRounded = (int)batteryPercent;
  if (batteryRounded == batteryLastSent) return;

  batteryLastSent = batteryRounded;
  snprintf(msg, sizeof(msg), "%d", batteryRounded);
  client.publish(mqtt_topic_battery, msg);

  Serial.print("MQTT sent battery ");
  Serial.println(batteryRounded);
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
    publishBatteryIfNeeded();
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
    publishBatteryIfNeeded();
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

  fanOn = false;
  doorOpen = false;
  windowOpen = false;
  buzzerOn = false;
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
  publishBatteryIfNeeded();
}

// Batterie

void updateBattery() {
  unsigned long now = millis();
  if (now - lastBatteryUpdate < BATTERY_INTERVAL_MS) {
    return;
  }
  lastBatteryUpdate = now;

  float consumption = 0.0;

  if (fanOn && !energySaveMode) {
    consumption += FAN_LOAD;
  }
  if (buzzerOn && !energySaveMode) {
    consumption += BUZZER_LOAD;
  }

  if (consumption > 0.0) {
    batteryPercent -= consumption;

    if (batteryPercent > 100.0) batteryPercent = 100.0;
    if (batteryPercent < 0.0)   batteryPercent = 0.0;
  }

  bool prevEnergySave = energySaveMode;
  if (batteryPercent < 40.0) {
    energySaveMode = true;
  } else {
    energySaveMode = false;
  }

  if (!prevEnergySave && energySaveMode) {
    bool doorWasOpen   = doorOpen;
    bool windowWasOpen = windowOpen;

    fanOn = false;
    doorOpen = false;
    windowOpen = false;
    buzzerOn = false;
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
  publishBatteryIfNeeded();
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

// WLAN Setup mit Timeout

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi connected");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    delay(800);
  } else {
    Serial.println("WiFi failed");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi failed");
    lcd.setCursor(0, 1);
    lcd.print("offline mode");
    delay(800);
  }
}

// MQTT reconnect non blocking

void reconnectNonBlocking() {
  if (client.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < 5000) return;
  lastMqttReconnectAttempt = now;

  Serial.print("Attempting MQTT connection...");
  bool ok = client.connect(mqtt_client_id);
  if (ok) {
    Serial.println("connected");
    publishBatteryIfNeeded();
  } else {
    Serial.print("failed rc=");
    Serial.println(client.state());
  }
}

// Setup

void setup() {
  Serial.begin(115200);

  Wire.begin();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Booting...");
  lcd.setCursor(0, 1);
  lcd.print("Haus A");

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

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);

  button1.attachClick(onBtn1Click);
  button1.attachLongPressStop(onBtn1LongPress);
  button2.attachClick(onBtn2Click);

  mfrc522.PCD_Init();

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);

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

  reconnectNonBlocking();

  if (client.connected()) {
    client.loop();
    publishBatteryIfNeeded();
  }

  delay(5);
}