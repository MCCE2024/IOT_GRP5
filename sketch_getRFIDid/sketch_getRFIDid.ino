/*
Liest auf die RFID Karten-UID aus und gibt sie über die serielle Schnittstelle aus.
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include "MFRC522_I2C.h"

MFRC522 mfrc522(0x28);   // I2C Adresse wie in Keyestudio PDF

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mfrc522.PCD_Init();

  Serial.println("Please scan card");
}

void loop() {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  Serial.print("UID: ");

  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i]);
    Serial.print(" ");
  }

  Serial.println();
  delay(1500);
}