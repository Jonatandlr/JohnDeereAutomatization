
#include <Arduino.h>

const int encoderPin = 3;
volatile long pulseCount = 0;
volatile long frecuenciaEncoder = 0;



long interval = 100;
unsigned long previuosMillis = 0;

float sp;
float pv;

int pwm_salida = 11;
int in1 = 8;
int in2 = 9;


float cv;
float cv1;
float error;
float error1;
float error2;

float Kp = 0.036864919;
float Ki = 0.077055771;
float Kd = 0.015464848;
float Tm = 0.1;
// float Kp = 0.3;
// float Ki = 1.5;
// float Kd = 0.01;
// float Tm = 0.3;
void countPulse() {
  pulseCount++;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(pwm_salida, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currentMillis = millis();
  if ((currentMillis - previuosMillis) >= interval) {
    previuosMillis = currentMillis;
    frecuenciaEncoder = pulseCount;
    float rpm = 10 * ((frecuenciaEncoder * 60.0) / 42);  //42ticks/1rev, 60seg/1min
    // Serial.println(rpm);
    pv = rpm;
    pulseCount = 0;
  }

  sp = 300;  //set point desired 
  error = sp - pv;

  //ecuacion de diferencias
  cv = cv1 + (Kp + Kd / Tm) * error + (-Kp + Ki * Tm - 2 * Kd / Tm) * error1 + (Kd / Tm) + error2;
  cv1 = cv;
  error2 = error1;
  error1 = error;

  //saturamos la salida del pid
  if (cv > 500) { cv = 500; }
  if (cv < 0) { cv = 0; }

  int vel = cv * (255.0 / 500.0);
  // Serial.println(vel);
  analogWrite(pwm_salida, vel);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Serial.print("VEL:");
  // Serial.print(vel);
  Serial.print(">sp: ");
  Serial.println(sp);
  
  Serial.print(">pv: ");
  Serial.println(pv);
  delay(1);
}

