
// #include <Arduino.h>

// #include <Wire.h>
// float RateRoll, RatePitch, RateYaw;
// float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
// int RateCalibrationNumber;
// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch, AngleYaw;
// float RateBiasYaw;

// unsigned long tiempo_prev = 0;


// float KalmanAngleYaw = 0;
// // float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
// float Q_angle = 0.1, Q_bias = 0.1, R_measure = 1;

// float P[2][2] = {{0, 0}, {0, 0}};
// float K[2];

// void kalmanUpdate(float newAngle, float newRate, float dt)
// {
//     // Prediction
//     RateYaw = newRate - RateBiasYaw;
//     KalmanAngleYaw += dt * RateYaw;


//     P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
//     P[0][1] -= dt * P[1][1];
//     P[1][0] -= dt * P[1][1];
//     P[1][1] += Q_bias * dt;

//     // Update
//     K[0] = P[0][0] / (P[0][0] + R_measure);
//     K[1] = P[1][0] / (P[0][0] + R_measure);

//     float y = newAngle - KalmanAngleYaw;
//     KalmanAngleYaw += K[0] * y;
//     RateBiasYaw += K[1] * y;

//     P[0][0] -= K[0] * P[0][0];
//     P[0][1] -= K[0] * P[0][1];
//     P[1][0] -= K[1] * P[0][0];
//     P[1][1] -= K[1] * P[0][1];
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
//     RateRoll = (float)GyroX / 65.5;
//     RatePitch = (float)GyroY / 65.5;
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
//     kalmanUpdate(AngleYaw, RateYaw, dt);
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
//         RateCalibrationYaw += RateYaw;
//         delay(1);
//     }
//     RateCalibrationYaw /= 1000;
//     Serial.println("Calibracion finalizada");
// }

// void loop()
// {
//     gyro_signals();
//     RateYaw -= RateCalibrationYaw;
//     Serial.print("angle: ");
//     Serial.println(KalmanAngleYaw);
//     delay(50);
// }