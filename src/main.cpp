#include <Arduino.h>
#include <Giroscopio.h>
#include <motores.h>
#include <PID_v1.h>

float angle;
float angleSet;
int pwm = 127;
int count = 0;

const int trayecto = 3; // distancia en metros a recorrer

const int encoderPin = 3;
volatile long pulseCount = 0;
volatile long frecuenciaEncoder = 0;
const int pulsePerRevolution = 56;
float distancia = 0;

unsigned long interval = 100;
unsigned long previuosMillis = 0;

motores motor(10, 8, 9, 5, 7, 6);
Giroscopio giros(1);

double Setpoint = 0;
double Input, Output;

PID myPID(&Input, &Output, &Setpoint, 10.4812, 5.3326, 0.00, DIRECT);

void countPulse()
{
    pulseCount++;
}

void setup()
{
    Serial.begin(115200);
    motor.setup();
    motor.setSpeedLeft(120);
    motor.setSpeedRight(120);
    giros.setup();
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(100, 200);

    attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);
    delay(50);
}

void loop()
{
    //   giros.getAngle("Yaw");

    if (pulseCount >= pulsePerRevolution)
    {
        distancia += 20.42;
        pulseCount = 0;
    }

    if (distancia <= trayecto * 100)
    // if (true)
    {
        motor.avanzar();
        angle = giros.getAngle("Yaw");
        Input = angle;
        myPID.Compute();
        motor.setSpeedRight(Output);
        Serial.print(">angle: ");
        Serial.println(angle);
        Serial.print(">output: ");
        Serial.println(Output);
    }
    else
    {
        pulseCount = 0;
        motor.stop();
        motor.setSpeedRight(200);
        motor.setSpeedLeft(200);
        delay(1000);
        angle = giros.getAngle("Yaw");
        angleSet = angle + 43;
        while (angle <= angleSet)
        {
            motor.girarIzq();
            angle = giros.getAngle("Yaw");
        }
        motor.stop();
        delay(1000);
        distancia = 0;
        motor.setSpeedLeft(120);
        motor.setSpeedRight(120);
        angle = giros.getAngle("Yaw");
        Setpoint = angle;
        Input = angle;
    }

    // // for (int i = 0; i < 20; i++)
    // // {
    // //     motor.girarIzq();
    // // }
    // angle = giros.getAngle("Yaw");
    // motor.setSpeedRight(200);
    // motor.setSpeedLeft(200);

    // while (angle <= 50)
    // {
    //     motor.girarIzq();
    //     angle = giros.getAngle("Yaw");
    //     Serial.println(angle);
    // }
    // while (true)
    // {
    //     motor.stop();
    // }
    // // // // Serial.println(giros.getAngle("Yaw"));
}
