#include <Wire.h>
#include <Adafruit_AHTX0.h>

Adafruit_AHTX0 aht;

const int relePin = 15;
const int trigPin = 4;
const int echoPin = 5;
const int ledPin = 2; // LED comum

void setup() {
  Serial.begin(115200);
  Serial.println("Iniciando sensores...");

  pinMode(relePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(relePin, HIGH);  // Desliga relé
  digitalWrite(ledPin, LOW);    // LED apagado

  if (!aht.begin()) {
    Serial.println("Sensor AHT20 não encontrado.");
    while (1) delay(10);
  }
  Serial.println("AHT20 encontrado!");
}

void loop() {
  // Sensor de temperatura
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  float temperatura = temp.temperature;

  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");

  if (temperatura > 26.0) {
    digitalWrite(relePin, LOW);  // Liga relé
  } else {
    digitalWrite(relePin, HIGH); // Desliga relé
  }

  // Sensor de distância
  long duration;
  float distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distância: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < 10.0) {
    digitalWrite(ledPin, HIGH); // Acende o LED
  } else {
    digitalWrite(ledPin, LOW);  // Apaga o LED
  }

  delay(1000);
}
