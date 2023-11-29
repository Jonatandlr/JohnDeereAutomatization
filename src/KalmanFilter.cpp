// // Combine a gyroscope and accelerometer to get a better angle estimate
// #include <Arduino.h>

// #include <Wire.h>
// float RateRoll, RatePitch, RateYaw;
// float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
// int RateCalibrationNumber;
// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch, AngleYaw;
// uint32_t LoopTimer;
// unsigned long tiempo_prev = 0;

// float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
// float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
// float KalmanAngleYaw = 0, KalmanUncertaintyAngleYaw = 2 * 2;
// float Kalman1DOutput[] = {0, 0};

// void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
// {
//     KalmanState = KalmanState + 0.004 * KalmanInput;
//     KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
//     float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);
//     KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
//     KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
//     Kalman1DOutput[0] = KalmanState;
//     Kalman1DOutput[1] = KalmanUncertainty;
// }

// void gyro_signals(void)
// {
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1A);
//     Wire.write(0x05);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1C);
//     Wire.write(0x10);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x3B);
//     Wire.endTransmission();
//     Wire.requestFrom(0x68, 6);
//     int16_t AccXLSB = Wire.read() << 8 | Wire.read();
//     int16_t AccYLSB = Wire.read() << 8 | Wire.read();
//     int16_t AccZLSB = Wire.read() << 8 | Wire.read();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x1B);
//     Wire.write(0x8);
//     Wire.endTransmission();
//     Wire.beginTransmission(0x68);
//     Wire.write(0x43);
//     Wire.endTransmission();
//     Wire.requestFrom(0x68, 6);
//     int16_t GyroX = Wire.read() << 8 | Wire.read();
//     int16_t GyroY = Wire.read() << 8 | Wire.read();
//     int16_t GyroZ = Wire.read() << 8 | Wire.read();
//     RatePitch = (float)GyroX / 65.5;
//     RateRoll = (float)GyroY / 65.5;
//     RateYaw = (float)GyroZ / 65.5;

//     AccX = (float)AccXLSB / 4096 + 0.04;
//     AccY = (float)AccYLSB / 4096 - 0.29;
//     AccZ = (float)AccZLSB / 4096 + 0.30;

//     AnglePitch = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
//     AngleRoll = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);

//     // dt para angulo yaw
//     float dt = (millis() - tiempo_prev) / 1000.0;
//     tiempo_prev = millis();

//     // Calcular el ángulo de Yaw
//     AngleYaw = AngleYaw + RateYaw * dt;
// }
// void setup()
// {
//     Serial.begin(115200);
//     pinMode(13, OUTPUT);
//     digitalWrite(13, HIGH);
//     Wire.setClock(400000);
//     Wire.begin();
//     delay(250);
//     Wire.beginTransmission(0x68);
//     Wire.write(0x6B);
//     Wire.write(0x00);
//     Wire.endTransmission();
//     for (RateCalibrationNumber = 0; RateCalibrationNumber < 1000; RateCalibrationNumber++)
//     {
//         gyro_signals();
//         RateCalibrationRoll += RateRoll;
//         RateCalibrationPitch += RatePitch;
//         RateCalibrationYaw += RateYaw;
//         delay(1);
//     }
//     RateCalibrationRoll /= 1000;
//     RateCalibrationPitch /= 1000;
//     RateCalibrationYaw /= 1000;
//     LoopTimer = micros();
//     Serial.println("Calibracion finalizada");
// }

// void loop()
// {
//     gyro_signals();
//     RateRoll -= RateCalibrationRoll;
//     RatePitch -= RateCalibrationPitch;
//     RateYaw -= RateCalibrationYaw;
//     kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
//     KalmanAngleRoll = Kalman1DOutput[0];
//     KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

//     kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
//     KalmanAnglePitch = Kalman1DOutput[0];
//     KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

//     Serial.print("Roll Angle [°]: ");
//     Serial.print(KalmanAngleRoll);
//     Serial.print(" Pitch Angle [°]: ");
//     Serial.println(KalmanAnglePitch);
//     // Aplicar el filtro de Kalman al ángulo de Yaw
//     // kalman_1d(KalmanAngleYaw, KalmanUncertaintyAngleYaw, RateYaw, AngleYaw);
//     // KalmanAngleYaw = Kalman1DOutput[0];
//     // KalmanUncertaintyAngleYaw = Kalman1DOutput[1];

//     // Serial.print("Yaw Angle [°]: ");
//     // Serial.println(KalmanAngleYaw);
//     while (micros() - LoopTimer < 4000)
//         ;
//     LoopTimer = micros();
// }