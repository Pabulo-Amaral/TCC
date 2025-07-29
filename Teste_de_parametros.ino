#include <Adafruit_MCP4725.h>
#include <Arduino.h>
#define SOUND_SPEED 0.034 //define sound speed in cm/uS

const int trigPin = 18; //pino do Trig do sensor de distância 
const int echoPin = 5; //pino do Echo do sensor de distância
const int pwmPin = 19; //pino de saída do PWM
const int ledPin = 2; //pino do LED do esp32
const int angleSensorPin =4; // Pino analógico para o sensor de ângulo
const int pin1motor = 22;
const int pin2motor = 23;

//int pwmValue = 178; // PWM ajustado para 70%
long duration;
float distanceCm;
float angle;

// Sua sequência PRBS de 64 bits de 10ms, equivalentes a uma sequandia de PRBS de 16 bits de 40ms
const uint64_t prbsSequence = 0b1111000011110000000011111111000011110000000000001111111111110000;
const int sequenceLength = 64;
int currentIndex = 0;


hw_timer_t  * timer = NULL;
void IRAM_ATTR timer_ISR()
{
  digitalWrite(ledPin, !digitalRead(ledPin)); // Acionar/desatiar o Led de controle de tempo
  
  // Leitura do sensor de distãncia
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distanceCm = duration * SOUND_SPEED / 2;
  
 
  // Leitura do sensor angular
  int sensorValue = analogRead(angleSensorPin);
  int angle_1 = sensorValue * 360;
  float angle = angle_1 / 4095;

  
  int bit = (prbsSequence >> (sequenceLength - 1 - currentIndex)) & 0x1; // Obter o bit atual da sequência

  int pwmValue = bit * 178;// Converter o bit para valor PWM (0 ou 255)

  //Comando do sentido de ativação do motor.
  if (bit > 0){
  digitalWrite(pin1motor,1);
  digitalWrite(pin2motor,0);
  } else {
  digitalWrite(pin1motor,0);
  digitalWrite(pin2motor,1);
  }

  analogWrite(pwmPin, pwmValue);  // Enviar o sinal PWM
 
  // Mostrar no Serial para depuração
  Serial.println(distanceCm);
  Serial.println(sensorValue);
  Serial.println(bit);

  // Avançar para o próximo bit da sequancia PRBS
  currentIndex++;
  if (currentIndex >= sequenceLength) {
    currentIndex = 0; // Reiniciar a sequência
  }

  digitalWrite(ledPin, !digitalRead(ledPin)); // Acionar/desatiar o Led de controle de tempo

}
void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(pin1motor, OUTPUT);
  pinMode(pin2motor, OUTPUT);

  //confiigurar timer
  timer = timerBegin(1000000);  //ajuste de frequencia para 1MHz
  timerAttachInterrupt(timer, &timer_ISR);
  timerAlarm(timer, 10000, true,0); // configurado para acionar em 10000 ciclos, equivalente a 10 milisegundos

}

void loop() {
  
}