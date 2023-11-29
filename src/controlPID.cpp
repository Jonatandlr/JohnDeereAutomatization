// #include <Arduino.h>
// #include <motores.h>
// #include <PID_v1.h>

// int encoderL = 3;
// int countLeft = 0;
// volatile long frecuenciaEncoderL = 0;
// int encoderR = 2;
// int countRight = 0;
// volatile long frecuenciaEncoderR = 0;
// unsigned long interval = 100;
// unsigned long previuosMillis = 0;

// motores motor(11, 8, 9, 10, 7, 6);

// unsigned long tiempoPrevio = 0;

// // Parámetros del filtro de Kalman left
// float estimateL = 0;
// float errorEstimateL = 1;
// float processNoiseL = 0.01;
// float measurementNoiseL = 1;

// // Parámetros del filtro de Kalman right
// float estimateR = 0;
// float errorEstimateR = 1;
// float processNoiseR = 0.01;
// float measurementNoiseR = 1;

// double KpL = 2.3637;
// double KiL = 84.0163;
// double KdL = 0.00;


// double KpR = 2.4162;

// double KiR = 92.6655;
// double KdR = 0.000;


// double InputL;         // input is rpm
// double InputR;         // input is rpm
// double SetpointR= 268; // setpoint of RPM
// double setpointL = 245;
// double OutputL;        // output is pwm == cv
// double OutputR;

// PID pidLeft(&InputL, &OutputL, &setpointL, KpL, KdL, KdL, DIRECT);

// PID pidRight(&InputR, &OutputR, &SetpointR, KpR, KdR, KdR, DIRECT);

// void countL()
// {
//     countLeft++;
//     // Serial.print(">izquierda: ");
//     // Serial.println(countLeft);
// }
// void countR()
// {
//     countRight++;
//     // Serial.print(">derecha: ");
//     // Serial.println(countLeft);
// }

// void setup()
// {
//     Serial.begin(115200);
//     motor.setup();
//     attachInterrupt(digitalPinToInterrupt(encoderL), countL, RISING);
//     attachInterrupt(digitalPinToInterrupt(encoderR), countR, RISING);
//     motor.setSpeedRight(0);
//     motor.setSpeedLeft(0);

//     pidLeft.SetMode(AUTOMATIC);  // Modo de control Automático
//     pidRight.SetMode(AUTOMATIC); // Modo de control Automático
//     pidLeft.SetOutputLimits(70, 255);
//     pidRight.SetOutputLimits(70, 255);
//     delay(1000);
//     tiempoPrevio = millis(); // Iniciar el temporizador
// }

// void loop()
// {

//     unsigned long currentMillis = millis();
//     if (currentMillis - previuosMillis >= interval)
//     {
//         previuosMillis = currentMillis;
//         frecuenciaEncoderL = countLeft;
//         frecuenciaEncoderR = countRight;

//         // Predicción del estado
//         float predictedEstimateL = estimateL;
//         float predictedErrorEstimateL = errorEstimateL + processNoiseL;

//         // Actualización del estado basado en la medición
//         float kalmanGainL = predictedErrorEstimateL / (predictedErrorEstimateL + measurementNoiseL);
//         estimateL = predictedEstimateL + kalmanGainL * (frecuenciaEncoderL - predictedEstimateL);
//         errorEstimateL = (1 - kalmanGainL) * predictedErrorEstimateL;

//         // Predicción del estado
//         float predictedEstimateR = estimateR;
//         float predictedErrorEstimateR = errorEstimateR + processNoiseR;

//         // Actualización del estado basado en la medición
//         float kalmanGainR = predictedErrorEstimateR / (predictedErrorEstimateR + measurementNoiseR);
//         estimateR = predictedEstimateR + kalmanGainR * (frecuenciaEncoderR - predictedEstimateR);
//         errorEstimateR = (1 - kalmanGainR) * predictedErrorEstimateR;

//         // Output del filtro de Kalman
//         float filteredRPML = 10 * ((estimateL * 60) / 56);

//         float filteredRPMR = 10 * ((estimateR * 60) / 58);

//         InputL = filteredRPML;

//         InputR = filteredRPMR;

//         Serial.print(">RPM left: ");
//         Serial.println(filteredRPML);
//         Serial.print(">PWM left: ");
//         Serial.println(OutputL);
//         Serial.print(">RPM right: ");
//         Serial.println(filteredRPMR);
//         Serial.print(">PWM right: ");
//         Serial.println(OutputR);

//         countRight = 0;
//         countLeft = 0;
//         pidLeft.Compute();
//         delay(1);
//         pidRight.Compute();
//         delay(1);
//     }
//     motor.setSpeedLeft(OutputL);
//     motor.setSpeedRight(OutputR);
//     motor.avanzar();
// }