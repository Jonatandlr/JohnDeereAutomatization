// #include <Arduino.h>

// #include <PID_v1.h>

// const int encoderPin = 3;
// volatile long pulseCount = 0;
// volatile long frecuenciaEncoder = 0;

// unsigned long interval = 100;
// unsigned long previuosMillis = 0;

// int pwm_salida = 11;
// int in1 = 8;
// int in2 = 9;

// double Kp = 6.2283;
// double Ki = 82.5020;
// double Kd = 0.000;
// float Tm = 0.1;

// double Input;          // input is rpm
// double Setpoint = 600; // setpoint is 60 rpm
// double Output;         // output is pwm == cv

// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// // Parámetros del filtro de Kalman
// float estimate = 0;
// float errorEstimate = 1;
// float processNoise = 0.01;
// float measurementNoise = 1;

// void countPulse()
// {
// pulseCount++;
// }

// void setup()
// {
// // put your setup code here, to run once:
// Serial.begin(115200);
// pinMode(encoderPin, INPUT_PULLUP);
// pinMode(pwm_salida, OUTPUT);
// pinMode(in1, OUTPUT);
// pinMode(in2, OUTPUT);
// attachInterrupt(digitalPinToInterrupt(encoderPin), countPulse, RISING);

// myPID.SetMode(AUTOMATIC); // Modo de control Automático
// myPID.SetOutputLimits(120, 255);
// delay(1000);
// }

// void loop()
// {

// unsigned long currentMillis = millis();
// if (currentMillis - previuosMillis >= interval)
// {
//     previuosMillis = currentMillis;
//     frecuenciaEncoder = pulseCount;
//     // Predicción del estado
//     float predictedEstimate = estimate;
//     float predictedErrorEstimate = errorEstimate + processNoise;

//     // Actualización del estado basado en la medición
//     float kalmanGain = predictedErrorEstimate / (predictedErrorEstimate + measurementNoise);
//     estimate = predictedEstimate + kalmanGain * (frecuenciaEncoder - predictedEstimate);
//     errorEstimate = (1 - kalmanGain) * predictedErrorEstimate;

//     // Output del filtro de Kalman
//     float filteredRPM = 10 * ((estimate * 60) / 20);
//     Serial.print(">RPM: ");
//     Serial.println(filteredRPM);
//     Input = filteredRPM;
//     pulseCount = 0;
//     myPID.Compute(); // Calcula el valor de salida
//     delay(1);
// }
// analogWrite(pwm_salida, Output);
// digitalWrite(in1, HIGH);
// digitalWrite(in2, LOW);
// Serial.print(">Output: ");
// Serial.println(Output);
// }
