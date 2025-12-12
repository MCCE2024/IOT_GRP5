/*
README Haus A Energie Monitor und Power Alert

Dieses Programm bildet ein energieautarkes Haus mit dem Keyestudio Smart Home Kit und ESP32 nach.
Der Energiezustand wird als Batteriestand in Prozent modelliert. Nur aktive Verbraucher entladen die Batterie.
LCD Anzeige und SK6812 Statusleiste bleiben immer aktiv und zählen nicht als Verbraucher.

Steuerung
Button eins kurzer Druck wählt das nächste Gerät im Menü
Button eins langer Druck wählt das vorherige Gerät im Menü
Button zwei kurzer Druck schaltet das aktuell gewählte Gerät ein oder aus
RFID öffnet die Tür bei gültiger UID und wenn kein Energiesparmodus aktiv ist
Reset setzt Batterie auf 100 und schaltet alle Verbraucher ab

Verbraucher
Fan kontinuierlich
Buzzer kontinuierlich
Yellow LED kontinuierlich
Door einmalig pro Aktion
Window einmalig pro Aktion

Energiesparmodus
Unter 40 Prozent werden alle Verbraucher abgeschaltet und Party Mode beendet
Unter 60 Prozent SK6812 gelb
Ab 60 Prozent SK6812 grün
Energiesparmodus SK6812 rot

MQTT
Batteriestand wird als ganze Zahl in Prozent an keyestudio/hausA/battery gesendet
Senden erfolgt nur bei sichtbarer Prozentänderung
WLAN und MQTT sind non blocking, Menü bleibt bedienbar

Party Mode
Party Mode ist ein eigener Menüpunkt
Im Party Mode laufen Fan, Yellow LED, Buzzer dauerhaft
Door und Window fahren zyklisch auf und zu
Batterieabzug ist deutlich schneller und kontinuierlich, damit der Effekt sichtbar ist
Während Party Mode aktiv ist, sind andere Menüaktionen gesperrt, nur Party und Reset sind schaltbar
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

const char* ssid = "";
const char* password = "";

const char* mqtt_server = "194.182.170.71";
const int mqtt_port = 1883;
const char* mqtt_client_id = "Keyestudio-Haus-A";
const char* mqtt_topic_battery = "keyestudio/hausA/battery";

int batteryLastSent = -1;

const int BTN1_PIN = 16;
const int BTN2_PIN = 27;

const int FAN_PIN1 = 19;
const int FAN_PIN2 = 18;

const int SK_PIN = 26;
const int SK_LED_COUNT = 4;

const int SERVO_DOOR_PIN = 13;
const int SERVO_WINDOW_PIN = 5;

const int BUZ_PIN = 25;

const int YELLOW_LED_PIN = 12;

LiquidCrystal_I2C lcd(0x27, 16, 2);
MFRC522 mfrc522(0x28);
Adafruit_NeoPixel strip(SK_LED_COUNT, SK_PIN, NEO_GRB + NEO_KHZ800);

Servo servoDoor;
Servo servoWindow;
bool doorServoAttached = false;
bool windowServoAttached = false;

BuzzerESP32 buzzer(BUZ_PIN);

OneButton button1(BTN1_PIN, true);
OneButton button2(BTN2_PIN, true);

float batteryPercent = 100.0f;

const float FAN_LOAD        = 3.0f;
const float BUZZER_LOAD     = 1.0f;
const float YELLOW_LED_LOAD = 1.0f;

const float DOOR_MOVE_COST   = 5.0f;
const float WINDOW_MOVE_COST = 5.0f;

bool energySaveMode = false;

bool fanOn = false;
bool doorOpen = false;
bool windowOpen = false;
bool buzzerOn = false;
bool yellowLedOn = false;

bool suppressMoveCost = false;

bool partyModeOn = false;
unsigned long lastPartyMove = 0;
const unsigned long PARTY_MOVE_INTERVAL_MS = 1200;
bool partyDoorState = false;
bool partyWindowState = false;

const float PARTY_EXTRA_LOAD = 8.0f;

enum Device {
  DEV_FAN = 0,
  DEV_DOOR = 1,
  DEV_WINDOW = 2,
  DEV_BUZZER = 3,
  DEV_YELLOW_LED = 4,
  DEV_PARTY = 5,
  DEV_RESET = 6,
  DEV_COUNT = 7
};

int selectedDevice = DEV_FAN;

String rfidPassword = "";
String correctPassword = "845138219";

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

bool wifiWasConnected = false;
unsigned long lastWifiCheck = 0;
const unsigned long WIFI_CHECK_MS = 2000;

unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_MS = 5000;

unsigned long lastBatteryMillis = 0;

void setAllPixels(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void updateStatusLeds() {
  if (energySaveMode) {
    setAllPixels(strip.Color(255, 0, 0));
  } else if (batteryPercent < 60.0f) {
    setAllPixels(strip.Color(255, 255, 0));
  } else {
    setAllPixels(strip.Color(0, 255, 0));
  }
}

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

void applyYellowLedState() {
  if (yellowLedOn && !energySaveMode) {
    digitalWrite(YELLOW_LED_PIN, HIGH);
  } else {
    digitalWrite(YELLOW_LED_PIN, LOW);
    yellowLedOn = false;
  }
}

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

void moveDoorOnce(bool withCost) {
  attachDoorServo();

  if (!energySaveMode && doorOpen) {
    servoDoor.write(180);
  } else {
    servoDoor.write(0);
  }

  if (!energySaveMode && withCost && !suppressMoveCost) {
    batteryPercent -= DOOR_MOVE_COST;
    if (batteryPercent < 0.0f) batteryPercent = 0.0f;
  }

  delay(150);
  detachDoorServo();

  publishBatteryIfNeeded();
}

void moveWindowOnce(bool withCost) {
  attachWindowServo();

  if (!energySaveMode && windowOpen) {
    servoWindow.write(180);
  } else {
    servoWindow.write(0);
  }

  if (!energySaveMode && withCost && !suppressMoveCost) {
    batteryPercent -= WINDOW_MOVE_COST;
    if (batteryPercent < 0.0f) batteryPercent = 0.0f;
  }

  delay(150);
  detachWindowServo();

  publishBatteryIfNeeded();
}

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
  int dur = marioDurations[marioIndex];

  buzzer.playTone(freq, dur);

  marioNextChange = now + (unsigned long)dur;
  marioIndex++;
  if (marioIndex >= MARIO_LENGTH) {
    marioIndex = 0;
  }
}

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

    case DEV_YELLOW_LED:
      lcd.print("Yellow ");
      lcd.print(yellowLedOn ? "ON " : "OFF");
      break;

    case DEV_PARTY:
      lcd.print("Party ");
      lcd.print(partyModeOn ? "ON " : "OFF");
      break;

    case DEV_RESET:
      lcd.print("Reset Ready");
      break;
  }
}

void stopPartyMode() {
  partyModeOn = false;

  fanOn = false;
  buzzerOn = false;
  yellowLedOn = false;

  hardFanOff();
  stopBuzzerTone();
  applyYellowLedState();

  partyDoorState = false;
  partyWindowState = false;
}

void startPartyMode() {
  partyModeOn = true;

  fanOn = true;
  buzzerOn = true;
  yellowLedOn = true;

  applyFanState();
  applyYellowLedState();
  startMario();

  partyDoorState = doorOpen;
  partyWindowState = windowOpen;

  lastPartyMove = millis();
}

void updatePartyMode() {
  if (!partyModeOn) return;
  if (energySaveMode) return;

  unsigned long now = millis();
  if (now - lastPartyMove < PARTY_MOVE_INTERVAL_MS) return;
  lastPartyMove = now;

  partyDoorState = !partyDoorState;
  partyWindowState = !partyWindowState;

  doorOpen = partyDoorState;
  windowOpen = partyWindowState;

  suppressMoveCost = true;
  moveDoorOnce(false);
  moveWindowOnce(false);
  suppressMoveCost = false;
}

void resetSystem() {
  energySaveMode = false;

  stopPartyMode();

  fanOn = false;
  doorOpen = false;
  windowOpen = false;
  buzzerOn = false;
  marioPlaying = false;
  yellowLedOn = false;

  hardFanOff();
  buzzer.playTone(0, 0);
  applyYellowLedState();

  suppressMoveCost = true;
  moveDoorOnce(false);
  moveWindowOnce(false);
  suppressMoveCost = false;

  batteryPercent = 100.0f;

  applyFanState();
  applyYellowLedState();

  updateStatusLeds();
  updateLcd();
  publishBatteryIfNeeded();
}

void updateBattery() {
  unsigned long now = millis();
  if (lastBatteryMillis == 0) lastBatteryMillis = now;

  float dtSeconds = (now - lastBatteryMillis) / 1000.0f;
  if (dtSeconds < 0.05f) return;
  lastBatteryMillis = now;

  float loadPerSecond = 0.0f;

  if (fanOn && !energySaveMode) loadPerSecond += FAN_LOAD;
  if (buzzerOn && !energySaveMode) loadPerSecond += BUZZER_LOAD;
  if (yellowLedOn && !energySaveMode) loadPerSecond += YELLOW_LED_LOAD;

  if (partyModeOn && !energySaveMode) loadPerSecond += PARTY_EXTRA_LOAD;

  if (loadPerSecond > 0.0f) {
    batteryPercent -= loadPerSecond * dtSeconds;
    if (batteryPercent < 0.0f) batteryPercent = 0.0f;
    if (batteryPercent > 100.0f) batteryPercent = 100.0f;
  }

  bool prevEnergySave = energySaveMode;
  energySaveMode = (batteryPercent < 40.0f);

  if (!prevEnergySave && energySaveMode) {
    bool doorWasOpen = doorOpen;
    bool windowWasOpen = windowOpen;

    stopPartyMode();

    fanOn = false;
    doorOpen = false;
    windowOpen = false;
    buzzerOn = false;
    marioPlaying = false;
    yellowLedOn = false;

    hardFanOff();
    buzzer.playTone(0, 0);
    applyYellowLedState();

    suppressMoveCost = true;
    if (doorWasOpen) moveDoorOnce(false);
    if (windowWasOpen) moveWindowOnce(false);
    suppressMoveCost = false;
  }

  applyFanState();
  applyYellowLedState();

  updateStatusLeds();
  updateLcd();
  publishBatteryIfNeeded();
}

void selectNextDevice() {
  selectedDevice++;
  if (selectedDevice >= DEV_COUNT) selectedDevice = 0;
  updateLcd();
}

void selectPrevDevice() {
  if (selectedDevice == 0) selectedDevice = DEV_COUNT - 1;
  else selectedDevice--;
  updateLcd();
}

void toggleSelectedDevice() {
  if (selectedDevice != DEV_RESET && energySaveMode) return;

  if (partyModeOn && selectedDevice != DEV_PARTY && selectedDevice != DEV_RESET) {
    updateLcd();
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
      if (buzzerOn) stopBuzzerTone();
      else {
        buzzerOn = true;
        startMario();
      }
      break;

    case DEV_YELLOW_LED:
      hardFanOff();
      yellowLedOn = !yellowLedOn;
      applyYellowLedState();
      break;

    case DEV_PARTY:
      if (partyModeOn) stopPartyMode();
      else startPartyMode();
      break;

    case DEV_RESET:
      resetSystem();
      break;
  }

  updateLcd();
  publishBatteryIfNeeded();
}

void onBtn1Click() {
  selectNextDevice();
}

void onBtn1LongPress() {
  selectPrevDevice();
}

void onBtn2Click() {
  toggleSelectedDevice();
}

void handleRfid() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    rfidPassword = "";
    return;
  }

  rfidPassword = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    rfidPassword += String(mfrc522.uid.uidByte[i]);
  }

  if (!energySaveMode && !partyModeOn && rfidPassword == correctPassword) {
    doorOpen = true;
    moveDoorOnce(true);
    updateLcd();
  }

  rfidPassword = "";
}

void wifiManagerTick() {
  unsigned long now = millis();
  if (now - lastWifiCheck < WIFI_CHECK_MS) return;
  lastWifiCheck = now;

  bool isConnected = (WiFi.status() == WL_CONNECTED);

  if (!isConnected) {
    if (wifiWasConnected) {
      wifiWasConnected = false;
      Serial.println("WiFi disconnected");
    }
    WiFi.disconnect(false);
    WiFi.begin(ssid, password);
    return;
  }

  if (!wifiWasConnected) {
    wifiWasConnected = true;
    Serial.println("WiFi connected");
    Serial.println(WiFi.localIP());
  }
}

void mqttManagerTick() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < MQTT_RECONNECT_MS) return;
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

  pinMode(YELLOW_LED_PIN, OUTPUT);
  digitalWrite(YELLOW_LED_PIN, LOW);

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

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  client.setServer(mqtt_server, mqtt_port);

  updateStatusLeds();
  updateLcd();

  lastBatteryMillis = millis();
}

void loop() {
  button1.tick();
  button2.tick();

  wifiManagerTick();
  mqttManagerTick();

  if (client.connected()) {
    client.loop();
  }

  handleRfid();

  updatePartyMode();
  updateBattery();
  updateMarioMelody();

  publishBatteryIfNeeded();

  delay(2);
}