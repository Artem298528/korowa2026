#include <Arduino.h>

// Пины кнопок
const int STARTER_PIN = 23;
const int SIDE_PIN    = 13;

void setup() {
  Serial.begin(115200);

  // Включаем внутреннюю подтяжку
  pinMode(STARTER_PIN, INPUT_PULLUP);
  pinMode(SIDE_PIN, INPUT_PULLUP);

  Serial.println("Button test started");
}

void loop() {
  int starterState = digitalRead(STARTER_PIN);
  int sideState    = digitalRead(SIDE_PIN);

  Serial.print("STARTER: ");
  Serial.print(starterState == LOW ? "PRESSED" : "RELEASED");

  Serial.print(" | SIDE: ");
  Serial.println(sideState == LOW ? "PRESSED" : "RELEASED");

  delay(200); // чтобы не спамить слишком быстро
}