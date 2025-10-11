#include <Adafruit_MCP4725.h>
#include <Arduino.h>

#define SOUND_SPEED 0.034

const int trigPin = 18;
const int echoPin = 5;
const int pwmPin = 19;
const int ledPin = 2;
const int angleSensorPin = 4;
const int pin1motor = 22;
const int pin2motor = 23;
const int Pinoseg = 34;

int pwmValue;
long duration;
float distanceCm;
float angle;

int ajuste = 2047;
float erro0 = 0;
float erro1 = 0;
float erro2 = 0;
float valor1 = 0;

int contadorLeituras = 0;
bool pidAtivo = false;

unsigned long tempoAnterior = 0;
const unsigned long intervalo = 10000; // 10 ms em microssegundos
unsigned long tempoExecAnterior = 0;

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(pin1motor, OUTPUT);
  pinMode(pin2motor, OUTPUT);
  pinMode(Pinoseg, INPUT);
}

void loop() {
  unsigned long tempoAtual = micros();

  if (tempoAtual - tempoAnterior >= intervalo) {
    tempoAnterior = tempoAtual;

    // ===== Medição do tempo real entre execuções =====
    unsigned long delta = tempoAtual - tempoExecAnterior;
    tempoExecAnterior = tempoAtual;

    Serial.print("Intervalo real: ");
    Serial.print(delta);
    Serial.println(" us");
    // =================================================

    // Alterna o LED
    digitalWrite(ledPin, !digitalRead(ledPin));

    // Medir distância
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * SOUND_SPEED / 2;

    // Ler o sensor de ângulo
    int sensorValue = analogRead(angleSensorPin);
    int angle_1 = sensorValue * 360;
    float angle = angle_1 / 4095.0;

    // ==== Armazenar leituras antes de ativar o PID ====
    if (!pidAtivo) {
      contadorLeituras++;
      if (contadorLeituras == 1) {
        Serial.println("1ª leitura armazenada.");
        erro1 = 0 - (2047 - sensorValue);
      } else if (contadorLeituras == 2) {
        Serial.println("2ª leitura armazenada.");
        erro2 = erro1;
        pidAtivo = true;
      }
    }
    // ==== Se já ativou, roda o PID normalmente ====
    else {
      erro0 = 0 - (2048 - sensorValue);
      float valor0 = valor1 + 150 * 0.001534 * erro0 - 500 * 0.001534 * erro1 + 320 * 0.001534 * erro2;

      pwmValue = round(valor0);
      pwmValue = constrain(pwmValue, -255, 255);
      int Pinseg = analogRead(Pinoseg);

      if (pwmValue > 0) {
        digitalWrite(pin1motor, 1);
        digitalWrite(pin2motor, 0);
        analogWrite(pwmPin, abs(pwmValue));
      } else {
        digitalWrite(pin1motor, 0);
        digitalWrite(pin2motor, 1);
        analogWrite(pwmPin, abs(pwmValue));
      }
      if (Pinseg > 4000) {
        analogWrite(pwmPin, abs(pwmValue));
      } else {
        analogWrite(pwmPin, 0);
      }

      // Atualização dos Errors
      erro2 = erro1;
      erro1 = erro0;
      valor1 = valor0;

      Serial.print("Distância (cm): ");
      Serial.println(distanceCm);
      Serial.print("Angle: ");
      Serial.println(angle);
      Serial.print("sensor: ");
      Serial.println(sensorValue);
      Serial.print(" | PWM: ");
      Serial.println(pwmValue);

    }
  }
}
