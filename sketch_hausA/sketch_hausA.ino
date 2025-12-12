/*
README Haus A Energie Monitor und Power Alert

Dieses Programm modelliert ein energieautarkes Haus mit ESP32 und Keyestudio Smart Home Kit.
Der Energiezustand wird als Batteriestand in Prozent geführt. Es existiert kein Grundverbrauch.
Nur aktiv geschaltete Verbraucher reduzieren den Batteriestand.

Bedienung
Linker Button kurzer Druck wählt das nächste Gerät
Linker Button langer Druck wählt das vorherige Gerät
Rechter Button kurzer Druck schaltet das ausgewählte Gerät
RFID öffnet die Tür bei gültiger UID wenn kein Energiesparmodus aktiv ist
Reset setzt Batterie auf 100 Prozent und schaltet alle Verbraucher aus

Verbraucher
Fan kontinuierlich
Buzzer kontinuierlich mit Super Mario Loop
Gelbe LED kontinuierlich
Tür einmalig 5 Prozent pro Bewegung
Fenster einmalig 5 Prozent pro Bewegung
Party Mode eigener Modus mit konstant 1 Prozent pro Sekunde

Energiesparmodus
Unter 40 Prozent werden alle Verbraucher abgeschaltet und Party Mode beendet
Unter 60 Prozent SK6812 gelb
Ab 60 Prozent SK6812 grün
Energiesparmodus SK6812 rot

Party Mode
Tür und Fenster bewegen zyklisch auf und zu
Gelbe LED an
Buzzer an
Fan bleibt im Party Mode immer aus, unabhängig vom Menüstatus
*/

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <ESP32Servo.h>
#include <OneButton.h>
#include <MFRC522_I2C.h>
#include <BuzzerESP32.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "";
const char* password = "";

const char* mqtt_server = "194.182.170.71";
const int mqtt_port = 1883;
const char* mqtt_client_id = "Keyestudio-Haus-A";
const char* mqtt_topic_battery = "keyestudio/hausA/battery";

WiFiClient espClient;
PubSubClient client(espClient);
char msg[50];
int batteryLastSent = -1;

const int BTN_LEFT_PIN  = 16;
const int BTN_RIGHT_PIN = 27;

const int FAN_PIN1 = 19;
const int FAN_PIN2 = 18;

const int SERVO_DOOR_PIN   = 13;
const int SERVO_WINDOW_PIN = 5;

const int BUZ_PIN = 25;
const int YELLOW_LED_PIN = 12;

const int SK_PIN = 26;
const int SK_LED_COUNT = 4;

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_NeoPixel strip(SK_LED_COUNT, SK_PIN, NEO_GRB + NEO_KHZ800);
MFRC522 mfrc522(0x28);

Servo servoDoor;
Servo servoWindow;
bool doorServoAttached = false;
bool windowServoAttached = false;

BuzzerESP32 buzzer(BUZ_PIN);

OneButton btnLeft(BTN_LEFT_PIN, true);
OneButton btnRight(BTN_RIGHT_PIN, true);

float batteryPercent = 100.0f;

const float FAN_LOAD        = 3.0f;
const float BUZZER_LOAD     = 1.0f;
const float YELLOW_LED_LOAD = 1.0f;

const float DOOR_MOVE_COST   = 5.0f;
const float WINDOW_MOVE_COST = 5.0f;

const float PARTY_LOAD_PER_SECOND = 1.0f;

bool energySaveMode = false;

bool fanOn = false;
bool doorOpen = false;
bool windowOpen = false;
bool buzzerOn = false;
bool yellowLedOn = false;

bool partyModeOn = false;
unsigned long lastPartyMove = 0;
const unsigned long PARTY_MOVE_INTERVAL_MS = 2200;

bool suppressMoveCost = false;

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

const int marioNotes[] = { 659,659,659,523,659,784,0,523,0,392,0,330 };
const int marioDurations[] = { 150,150,150,150,150,300,300,300,150,300,150,400 };
const int MARIO_LENGTH = sizeof(marioNotes) / sizeof(marioNotes[0]);

int marioIndex = 0;
unsigned long marioNextChange = 0;
bool marioPlaying = false;

unsigned long lastBatteryMillis = 0;

unsigned long lastWifiReconnectAttempt = 0;
const unsigned long WIFI_RECONNECT_MS = 8000;

unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_MS = 5000;

enum MoveState { MOVE_IDLE = 0, MOVE_DOOR_ACTIVE = 1, MOVE_WINDOW_ACTIVE = 2 };
MoveState moveState = MOVE_IDLE;
unsigned long moveUntilMs = 0;
const unsigned long SERVO_ACTIVE_MS = 180;

bool queuedDoorMove = false;
bool queuedWindowMove = false;
bool queuedDoorCost = false;
bool queuedWindowCost = false;

void lcdPrint16(uint8_t row, const String& text) {
  String t = text;
  if (t.length() > 16) t = t.substring(0, 16);
  while (t.length() < 16) t += " ";
  lcd.setCursor(0, row);
  lcd.print(t);
}

void setAllPixels(uint32_t color) {
  for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, color);
  strip.show();
}

void updateStatusLeds() {
  if (energySaveMode) setAllPixels(strip.Color(255, 0, 0));
  else if (batteryPercent < 60.0f) setAllPixels(strip.Color(255, 255, 0));
  else setAllPixels(strip.Color(0, 255, 0));
}

void hardFanOff() {
  fanOn = false;
  digitalWrite(FAN_PIN1, LOW);
  digitalWrite(FAN_PIN2, LOW);
}

void applyFanState() {
  if (partyModeOn) {
    digitalWrite(FAN_PIN1, LOW);
    digitalWrite(FAN_PIN2, LOW);
    return;
  }
  if (energySaveMode || !fanOn) {
    digitalWrite(FAN_PIN1, LOW);
    digitalWrite(FAN_PIN2, LOW);
    return;
  }
  digitalWrite(FAN_PIN1, HIGH);
  digitalWrite(FAN_PIN2, LOW);
}

void applyYellowLedState() {
  if (yellowLedOn && !energySaveMode) digitalWrite(YELLOW_LED_PIN, HIGH);
  else digitalWrite(YELLOW_LED_PIN, LOW);
}

void publishBatteryIfNeeded() {
  if (!client.connected()) return;
  int batteryRounded = (int)batteryPercent;
  if (batteryRounded == batteryLastSent) return;
  batteryLastSent = batteryRounded;
  snprintf(msg, sizeof(msg), "%d", batteryRounded);
  client.publish(mqtt_topic_battery, msg);
}

void attachDoorServo() {
  if (doorServoAttached) return;
  servoDoor.setPeriodHertz(50);
  servoDoor.attach(SERVO_DOOR_PIN, 1000, 2000);
  doorServoAttached = true;
}

void detachDoorServo() {
  if (!doorServoAttached) return;
  servoDoor.detach();
  doorServoAttached = false;
}

void attachWindowServo() {
  if (windowServoAttached) return;
  servoWindow.setPeriodHertz(50);
  servoWindow.attach(SERVO_WINDOW_PIN, 1000, 2000);
  windowServoAttached = true;
}

void detachWindowServo() {
  if (!windowServoAttached) return;
  servoWindow.detach();
  windowServoAttached = false;
}

void requestDoorMove(bool withCost) {
  if (energySaveMode) return;

  if (moveState != MOVE_IDLE) {
    queuedDoorMove = true;
    queuedDoorCost = withCost && !suppressMoveCost;
    return;
  }

  moveState = MOVE_DOOR_ACTIVE;
  moveUntilMs = millis() + SERVO_ACTIVE_MS;

  attachDoorServo();
  servoDoor.write(doorOpen ? 180 : 0);

  if (withCost && !suppressMoveCost) {
    batteryPercent -= DOOR_MOVE_COST;
    if (batteryPercent < 0.0f) batteryPercent = 0.0f;
  }

  publishBatteryIfNeeded();
}

void requestWindowMove(bool withCost) {
  if (energySaveMode) return;

  if (moveState != MOVE_IDLE) {
    queuedWindowMove = true;
    queuedWindowCost = withCost && !suppressMoveCost;
    return;
  }

  moveState = MOVE_WINDOW_ACTIVE;
  moveUntilMs = millis() + SERVO_ACTIVE_MS;

  attachWindowServo();
  servoWindow.write(windowOpen ? 180 : 0);

  if (withCost && !suppressMoveCost) {
    batteryPercent -= WINDOW_MOVE_COST;
    if (batteryPercent < 0.0f) batteryPercent = 0.0f;
  }

  publishBatteryIfNeeded();
}

void moveTick() {
  if (moveState != MOVE_IDLE && millis() >= moveUntilMs) {
    if (moveState == MOVE_DOOR_ACTIVE) detachDoorServo();
    if (moveState == MOVE_WINDOW_ACTIVE) detachWindowServo();
    moveState = MOVE_IDLE;
  }

  if (moveState == MOVE_IDLE) {
    if (queuedDoorMove) {
      bool cost = queuedDoorCost;
      queuedDoorMove = false;
      queuedDoorCost = false;
      requestDoorMove(cost);
      return;
    }
    if (queuedWindowMove) {
      bool cost = queuedWindowCost;
      queuedWindowMove = false;
      queuedWindowCost = false;
      requestWindowMove(cost);
      return;
    }
  }
}

void startMario() {
  marioIndex = 0;
  marioPlaying = true;
  marioNextChange = millis();
}

void stopMario() {
  marioPlaying = false;
  buzzer.playTone(0, 0);
}

void updateMarioMelody() {
  if (!buzzerOn || energySaveMode) {
    if (marioPlaying) stopMario();
    return;
  }

  unsigned long now = millis();
  if (!marioPlaying) startMario();

  while ((long)(now - marioNextChange) >= 0) {
    int freq = marioNotes[marioIndex];
    int dur  = marioDurations[marioIndex];

    if (freq > 0) buzzer.playTone(freq, 0);
    else buzzer.playTone(0, 0);

    marioNextChange += (unsigned long)dur;

    marioIndex++;
    if (marioIndex >= MARIO_LENGTH) marioIndex = 0;
  }
}

void updateLcd() {
  int b = (int)batteryPercent;
  lcdPrint16(0, String(b) + " %");

  String line2;

  if (energySaveMode) {
    line2 = "Energy Save";
    lcdPrint16(1, line2);
    return;
  }

  if (selectedDevice == DEV_FAN) line2 = String("Fan ") + (fanOn ? "ON" : "OFF");
  if (selectedDevice == DEV_DOOR) line2 = String("Door ") + (doorOpen ? "OPEN" : "CLOSE");
  if (selectedDevice == DEV_WINDOW) line2 = String("Win ") + (windowOpen ? "OPEN" : "CLOSE");
  if (selectedDevice == DEV_BUZZER) line2 = String("Buzz ") + (buzzerOn ? "ON" : "OFF");
  if (selectedDevice == DEV_YELLOW_LED) line2 = String("Yellow ") + (yellowLedOn ? "ON" : "OFF");
  if (selectedDevice == DEV_PARTY) line2 = String("Party ") + (partyModeOn ? "ON" : "OFF");
  if (selectedDevice == DEV_RESET) line2 = "Reset Ready";

  lcdPrint16(1, line2);
}

void stopPartyMode() {
  partyModeOn = false;

  yellowLedOn = false;
  buzzerOn = false;

  applyYellowLedState();
  stopMario();

  hardFanOff();
  applyFanState();
}

void startPartyMode() {
  partyModeOn = true;

  hardFanOff();
  applyFanState();

  yellowLedOn = true;
  applyYellowLedState();

  buzzerOn = true;
  startMario();

  lastPartyMove = millis();
}

void updatePartyMode() {
  if (!partyModeOn) return;
  if (energySaveMode) return;

  hardFanOff();
  applyFanState();

  unsigned long now = millis();
  if (now - lastPartyMove < PARTY_MOVE_INTERVAL_MS) return;
  lastPartyMove = now;

  doorOpen = !doorOpen;
  windowOpen = !windowOpen;

  suppressMoveCost = true;
  requestDoorMove(false);
  requestWindowMove(false);
  suppressMoveCost = false;
}

void resetSystem() {
  stopPartyMode();

  energySaveMode = false;

  fanOn = false;
  yellowLedOn = false;
  buzzerOn = false;

  doorOpen = false;
  windowOpen = false;

  hardFanOff();
  applyYellowLedState();
  stopMario();

  suppressMoveCost = true;
  requestDoorMove(false);
  requestWindowMove(false);
  suppressMoveCost = false;

  batteryPercent = 100.0f;
  batteryLastSent = -1;

  updateStatusLeds();
  publishBatteryIfNeeded();
}

void updateBattery() {
  unsigned long now = millis();
  if (lastBatteryMillis == 0) lastBatteryMillis = now;

  float dtSeconds = (now - lastBatteryMillis) / 1000.0f;
  if (dtSeconds < 0.05f) return;
  lastBatteryMillis = now;

  if (partyModeOn && !energySaveMode) {
    batteryPercent -= PARTY_LOAD_PER_SECOND * dtSeconds;
  } else {
    float loadPerSecond = 0.0f;
    if (fanOn && !energySaveMode) loadPerSecond += FAN_LOAD;
    if (buzzerOn && !energySaveMode) loadPerSecond += BUZZER_LOAD;
    if (yellowLedOn && !energySaveMode) loadPerSecond += YELLOW_LED_LOAD;
    if (loadPerSecond > 0.0f) batteryPercent -= loadPerSecond * dtSeconds;
  }

  if (batteryPercent < 0.0f) batteryPercent = 0.0f;
  if (batteryPercent > 100.0f) batteryPercent = 100.0f;

  bool prev = energySaveMode;
  energySaveMode = batteryPercent < 40.0f;

  if (!prev && energySaveMode) {
    stopPartyMode();
    hardFanOff();
    yellowLedOn = false;
    buzzerOn = false;
    applyYellowLedState();
    stopMario();
  }

  applyFanState();
  applyYellowLedState();

  updateStatusLeds();
  publishBatteryIfNeeded();
}

void selectNextDevice() {
  selectedDevice++;
  if (selectedDevice >= DEV_COUNT) selectedDevice = 0;
}

void selectPrevDevice() {
  if (selectedDevice == 0) selectedDevice = DEV_COUNT - 1;
  else selectedDevice--;
}

void toggleSelectedDevice() {
  if (selectedDevice == DEV_RESET) {
    resetSystem();
    return;
  }

  if (energySaveMode) return;

  if (partyModeOn && selectedDevice != DEV_PARTY) {
    stopPartyMode();
  }

  if (selectedDevice == DEV_FAN) {
    fanOn = !fanOn;
    applyFanState();
    return;
  }

  if (selectedDevice == DEV_DOOR) {
    doorOpen = !doorOpen;
    requestDoorMove(true);
    return;
  }

  if (selectedDevice == DEV_WINDOW) {
    windowOpen = !windowOpen;
    requestWindowMove(true);
    return;
  }

  if (selectedDevice == DEV_BUZZER) {
    buzzerOn = !buzzerOn;
    if (!buzzerOn) stopMario();
    return;
  }

  if (selectedDevice == DEV_YELLOW_LED) {
    yellowLedOn = !yellowLedOn;
    applyYellowLedState();
    return;
  }

  if (selectedDevice == DEV_PARTY) {
    if (partyModeOn) stopPartyMode();
    else startPartyMode();
    return;
  }
}

void onLeftClick() { selectNextDevice(); }
void onLeftLongPress() { selectPrevDevice(); }
void onRightClick() { toggleSelectedDevice(); }

void handleRfid() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    rfidPassword = "";
    return;
  }

  rfidPassword = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) rfidPassword += String(mfrc522.uid.uidByte[i]);

  if (!energySaveMode && !partyModeOn && rfidPassword == correctPassword) {
    doorOpen = true;
    requestDoorMove(true);
  }

  rfidPassword = "";
}

void wifiManagerTick() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastWifiReconnectAttempt < WIFI_RECONNECT_MS) return;
  lastWifiReconnectAttempt = now;

  WiFi.reconnect();
}

void mqttManagerTick() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (client.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnectAttempt < MQTT_RECONNECT_MS) return;
  lastMqttReconnectAttempt = now;

  client.connect(mqtt_client_id);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  lcd.init();
  lcd.backlight();

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

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  btnLeft.attachClick(onLeftClick);
  btnLeft.attachLongPressStop(onLeftLongPress);
  btnRight.attachClick(onRightClick);

  btnLeft.setDebounceMs(35);
  btnRight.setDebounceMs(35);
  btnLeft.setClickMs(450);
  btnRight.setClickMs(450);
  btnLeft.setPressMs(700);

  mfrc522.PCD_Init();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);

  client.setServer(mqtt_server, mqtt_port);

  lastBatteryMillis = millis();

  updateStatusLeds();
  updateLcd();
}

void loop() {
  btnLeft.tick();
  btnRight.tick();

  wifiManagerTick();
  mqttManagerTick();
  if (client.connected()) client.loop();

  handleRfid();

  updatePartyMode();
  updateBattery();
  updateMarioMelody();
  moveTick();

  updateLcd();
}