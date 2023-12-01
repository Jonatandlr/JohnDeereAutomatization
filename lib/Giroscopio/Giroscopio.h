#ifndef Giroscopio_h
#define Giroscopio_h

#include <Arduino.h>

class Giroscopio
{
    private:
        // Variuable for Gyroscopio
        float RateRoll, RatePitch, RateYaw;
        float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
        int RateCalibrationNumber;
        float AccX, AccY, AccZ;
        float AngleRoll, AnglePitch, AngleYaw;
        float RateBiasYaw;
        unsigned long tiempo_prev = 0;

        float KalmanAngleYaw = 0;
        // float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
        float Q_angle = 0.1, Q_bias = 0.1, R_measure = 1;
        float P[2][2] = {{0, 0}, {0, 0}};
        float K[2];

        int _predeterminado;

        void kalmanUpdate(float newAngle, float newRate, float dt);

    public:
        Giroscopio(int predeterminado) { _predeterminado = predeterminado; };
        void setup();
        void setupWithoutCalibration();
        float gyro_signals();
        float getAngle(String eje);
        float getRateYaw();
};

#endif