// //calibration to accelerometer

// #include <Arduino.h>

// #include <Wire.h>
// float RateRoll, RatePitch, RateYaw;
// float AccX, AccY, AccZ;
// float AngleRoll, AnglePitch, AngleYaw;
// float LoopTimer;
// unsigned long tiempo_prev = 0;
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
//     RateRoll = (float)GyroY / 65.5;
//     RatePitch = (float)GyroX / 65.5;
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
//     AngleYaw += RatePitch * dt;
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
//     tiempo_prev = millis();
// }
// void loop()
// {
//     gyro_signals();

//     // // CALIBRACION
//     // Serial.print("Acceleration X [g]= ");
//     // Serial.print(AccX);
//     // Serial.print(" Acceleration Y [g]= ");
//     // Serial.print(AccY);
//     // Serial.print(" Acceleration Z [g]= ");
//     // Serial.println(AccZ);
//     // delay(50);

//     //View the angle with accelerometer
//     Serial.print(" Roll Angel: ");
//     Serial.print(AngleRoll);

//     Serial.print(" Pitch Angle: ");
//     Serial.print(AnglePitch);
//     Serial.print(" Yaw:  ");
//     Serial.println(AngleYaw);
//     delay(50);
// }