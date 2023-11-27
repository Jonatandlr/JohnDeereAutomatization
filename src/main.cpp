#include <Arduino.h>
#include <motores.h>
#include <PID_v1.h>

int encoderL = 3;
int countLeft = 0;
volatile long frecuenciaEncoderL = 0;
int encoderR = 2;
int countRight = 0;
volatile long frecuenciaEncoderR = 0;
unsigned long interval = 100;
unsigned long previuosMillis = 0;

motores motor(11, 8, 9, 10, 7, 6);
const float pulsosPorRevolucion = 1000; // Ajusta esto según la resolución de tu encoder
const float radioRueda = 0.05;          // Ajusta esto según el radio de tus ruedas en metros
unsigned long tiempoPrevio = 0;

// Parámetros del filtro de Kalman left
float estimateL = 0;
float errorEstimateL = 1;
float processNoiseL = 0.01;
float measurementNoiseL = 1;

// Parámetros del filtro de Kalman right
float estimateR = 0;
float errorEstimateR = 1;
float processNoiseR = 0.01;
float measurementNoiseR = 1;

double Kp = 6.2283;
double Ki = 82.5020;
double Kd = 0.000;
float Tm = 0.1;

double InputL;         // input is rpm
double InputR;         // input is rpm
double Setpoint = 600; // setpoint is 60 rpm
double OutputL;        // output is pwm == cv
double OutputR;

PID pidLeft(&InputL, &OutputL, &Setpoint, Kp, Ki, Kd, DIRECT);
PID pidRight(&InputR, &OutputR, &Setpoint, Kp, Ki, Kd, DIRECT);

void countL()
{
    countLeft++;
    // Serial.print(">izquierda: ");
    // Serial.println(countLeft);
}
void countR()
{
    countRight++;
    // Serial.print(">derecha: ");
    // Serial.println(countLeft);
}

void setup()
{
    Serial.begin(115200);
    motor.setup();
    attachInterrupt(digitalPinToInterrupt(encoderL), countL, RISING);
    attachInterrupt(digitalPinToInterrupt(encoderR), countR, RISING);
    motor.setSpeedRight(255);
    motor.setSpeedLeft(255);
    pidLeft.SetMode(AUTOMATIC);  // Modo de control Automático
    pidRight.SetMode(AUTOMATIC); // Modo de control Automático
    pidLeft.SetOutputLimits(120, 255);
    pidRight.SetOutputLimits(120, 255);
    delay(1000);
    tiempoPrevio = millis(); // Iniciar el temporizador
}

void loop()
{

    unsigned long currentMillis = millis();
    if (currentMillis - previuosMillis >= interval)
    {
        previuosMillis = currentMillis;
        frecuenciaEncoderL = countLeft;
        frecuenciaEncoderR = countRight;

        // Predicción del estado
        float predictedEstimateL = estimateL;
        float predictedErrorEstimateL = errorEstimateL + processNoiseL;

        // Actualización del estado basado en la medición
        float kalmanGainL = predictedErrorEstimateL / (predictedErrorEstimateL + measurementNoiseL);
        estimateL = predictedEstimateL + kalmanGainL * (frecuenciaEncoderL - predictedEstimateL);
        errorEstimateL = (1 - kalmanGainL) * predictedErrorEstimateL;

        // Predicción del estado
        float predictedEstimateR = estimateR;
        float predictedErrorEstimateR = errorEstimateR + processNoiseR;

        // Actualización del estado basado en la medición
        float kalmanGainR = predictedErrorEstimateR / (predictedErrorEstimateR + measurementNoiseR);
        estimateR = predictedEstimateR + kalmanGainR * (frecuenciaEncoderR - predictedEstimateR);
        errorEstimateR = (1 - kalmanGainR) * predictedErrorEstimateR;

        // Output del filtro de Kalman
        float filteredRPML = 10 * ((estimateL * 60) / 20);
        float filteredRPMR = 10 * ((estimateR * 60) / 20);

        InputL = filteredRPML;
        InputR = filteredRPMR;

        Serial.print(">RPM left: ");
        Serial.println(filteredRPML);
        Serial.print(">RPM right: ");
        Serial.println(filteredRPMR);

        countRight = 0;
        countLeft = 0;
        pidLeft.Compute();
        pidRight.Compute();
        delay(1);
    }
    motor.setSpeedLeft(OutputL);
    motor.setSpeedRight(OutputR);
    motor.avanzar();
}