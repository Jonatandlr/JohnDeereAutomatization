
#include <Arduino.h>

const int encoderPin = 3;
volatile long pulseCount = 0;
volatile long frecuenciaEncoder = 0;

unsigned long interval = 100;
unsigned long previuosMillis = 0;

float sp;
float pv;
float p;
float i;

int pwm_salida = 11;
int in1 = 8;
int in2 = 9;

float cv;
float cv1;
float error;
float error1;
float error2;

float Kp = 6.2283;
float Ki = 82.5020;
// float Kd = 0.015464848;
float Tm = 0.1;

// Parámetros del filtro de Kalman
float estimate = 0;
float errorEstimate = 1;
float processNoise = 0.01;
float measurementNoise = 1;

int holi = 0;
int pwm=0;

void countPulse()
{
  pulseCount++;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(pwm_salida, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
}

void loop()
{

  unsigned long currentMillis = millis();
  if (currentMillis - previuosMillis >= interval)
  {
    previuosMillis = currentMillis;
    frecuenciaEncoder = pulseCount;
    // Predicción del estado
    float predictedEstimate = estimate;
    float predictedErrorEstimate = errorEstimate + processNoise;

    // Actualización del estado basado en la medición
    float kalmanGain = predictedErrorEstimate / (predictedErrorEstimate + measurementNoise);
    estimate = predictedEstimate + kalmanGain * (frecuenciaEncoder - predictedEstimate);
    errorEstimate = (1 - kalmanGain) * predictedErrorEstimate;

    // Output del filtro de Kalman
    float filteredRPM = 10 * ((estimate * 60) / 20);

    Serial.print(">RPM: ");
    Serial.println(filteredRPM);
    pv = filteredRPM;
    pulseCount = 0;
  }

  sp = 600; // set point desired
  error = sp - pv;

  // // ecuacion de diferencias
  // cv = cv1 + (Kp + Kd / Tm) * error + (-Kp + Ki * Tm - 2 * Kd / Tm) * error1 + (Kd / Tm) + error2;
  // cv1 = cv;
  // error2 = error1;

  cv = cv1 + (Kp + Ki * Tm) * error + Ki * error1;
  cv1 = cv;
  error1 = error;


  // p = Kp * error;
  // i = i + Ki * error * Tm;
  // cv = p + i;
  // error1 = error;

  Serial.print(">cv: ");
  Serial.println(cv);

  // saturamos la salida del pid
  if (cv > 1000)
  {
    cv = 1000;
  }
  if (cv < 470)
  {
    cv = 470;
  }
  
  Serial.print(">cv saturado: ");
  Serial.println(cv);

  int vel = cv * (255.0 / 1000.0);

  analogWrite(pwm_salida, vel);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  // Serial.print(">vel: ");
  // Serial.println(vel);
  // // Serial.print(">sp: ");
  // // Serial.println(sp);
  // Serial.print(" >pv: ");
  // Serial.println(pv);
  // delay(1);
}
