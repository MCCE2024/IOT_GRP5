/*
README Haus B Energie Monitor und Power Alert + E-Auto Ladestation

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
RFID Sensor startet die Ladung an der E-Auto Ladestation - je nach verwendetem RFID-Tag wird der Stromverbrauch dem jeweiligen Haus verrechnet
Der Menüpunkt Reset setzt die Batterie auf hundert Prozent und schaltet alle Verbraucher aus.

Verbraucher
Lüfter als elektrische Last mit kontinuierlichem Verbrauch bei Aktivität.
Tür über Servo, jede Bewegung verbraucht einmalig fünf Prozent Batterie.
Fenster über Servo, jede Bewegung verbraucht einmalig fünf Prozent Batterie.
Buzzer spielt eine einfache Super Mario Melodie, solange er im Menü aktiviert ist.
E-Auto verbraucht 

Energiesparlogik
Nur aktive Verbraucher reduzieren den Batteriestand, es gibt keinen Grundverbrauch.
Tür und Fenster reduzieren den Batteriestand nur einmal pro Bewegung.
Fällt die Batterie unter vierzig Prozent, wird der Energiesparmodus aktiv.
Im Energiesparmodus werden Lüfter, Tür, Fenster, Buzzer und E-Ladestation abgeschaltet.
Neue Verbraucher können im Energiesparmodus nicht eingeschaltet werden.
Der Reset Menüpunkt bleibt immer nutzbar und setzt den Zustand zurück.

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
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];

// WLAN-Zugangsdaten
const char* ssid = "";
const char* password = "";

// MQTT-Broker-Details
const char* mqtt_server = "194.182.170.71";
const int mqtt_port = 1883;
const char* mqtt_client_id = "Keyestudio-Haus-B"; // MUSS EINZIGARTIG SEIN!
const char* mqtt_topic_battery = "keyestudio/hausB/battery"; // Topic zum Senden
float batteryLastSent = -1.0;

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
MFRC522_I2C mfrc522(0x28,5);

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
const float BEV_LOAD = 1.0;

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
bool bevOn      = false;

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

// RFID Passwort aus UID 184 33 186 159
String rfidPassword = "";
String correctPassword = "18433186159";

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

  fanOn      = false;
  doorOpen   = false;
  windowOpen = false;
  buzzerOn   = false;
  marioPlaying = false;
  bevOn      = false;

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

  float consumption = 0.0;

  if (fanOn && !energySaveMode) {
    consumption += FAN_LOAD;
  }
  if (buzzerOn && !energySaveMode) {
    consumption += BUZZER_LOAD;
  }
  if (bevOn && !energySaveMode) {
    consumption += BEV_LOAD;
  }
  batteryPercent -= consumption;

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
    bevOn      = false;

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
    bevOn = !bevOn;
    updateLcd();
  }

  rfidPassword = "";
}

// Setup
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

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

void reconnect() {
  // Loop, bis wir wieder verbunden sind
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Versuche, eine Verbindung herzustellen
    if (client.connect(mqtt_client_id)) {
      Serial.println("connected");
      // Optional: Ein Subskription-Topic für Befehle von Node-RED abonnieren
      // client.subscribe("keyestudio/hausA/lampe/set");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Warte 5 Sekunden, bevor der nächste Versuch gestartet wird
      delay(5000);
    }
  }
}

// Loop

void loop() {
  button1.tick();
  button2.tick();

  handleRfid();
  updateBattery();
  updateMarioMelody();

  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // Muss regelmäßig aufgerufen werden, um Daten zu verarbeiten

  // Topic schreiben wenn sich Batterielöadezustand ändert
  if (batteryLastSent != batteryPercent) { 
    batteryLastSent = batteryPercent;
    snprintf (msg, 50, "%.0f", batteryPercent); // Wert in String umwandeln
    Serial.print("Publishing battery: ");
    Serial.println(msg);
    client.publish(mqtt_topic_battery, msg); // Daten an den Broker senden
  }

  delay(10);
}