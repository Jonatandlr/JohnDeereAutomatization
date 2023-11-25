#include <Arduino.h>
// Definir pines y variables
const int encoderPin = 3;
volatile long pulseCount = 0;
unsigned long lastTime = 0;



// Definir pines y variables para el segundo encoder
const int encoderPin2 = 2;
volatile long pulseCount2 = 0;
unsigned long lastTime2 = 0;

float kp = 0.1;         // Ganancia proporcional
float ki = 0.01;        // Ganancia integral
float setpoint = 15000; // RPM de referencia
float error, integral = 0;

int ena = 11;
int in1 = 8;
int in2 = 9;
// int enb=5;
// int in3=9;
// int in4=8;
void countPulse()
{
    pulseCount++;
}
void setup()
{
    Serial.begin(115200);
    pinMode(encoderPin, INPUT_PULLUP);
    pinMode(ena, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
    // attachInterrupt(digitalPinToInterrupt(encoderPin2), countPulse2, RISING);
}


void loop()
{
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastTime;

    if (elapsedTime >= 1000)
    { // Calcular RPM cada segundo
        float rpm = (pulseCount * 60.0) / (elapsedTime / 1000.0);
        Serial.print(">RPM: ");
        Serial.println(rpm);
        pulseCount = 0;
        lastTime = currentTime;
        error = setpoint - rpm;
        // Calcular la acción integral
        integral += error;
    }
    float control = kp * error + ki * integral;
    if (control < 0)
        control = 0;
    if (control > 255)
        control = 255;
    Serial.print(">control: ");
    Serial.println(control);

    analogWrite(ena, control);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}
