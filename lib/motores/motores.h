#ifndef motores_h
#define motores_h

#include <Arduino.h>

class motores
{
public:
    motores(int INA, int delanteraIzq, int traseraIzq, int INB, int delanteraDer, int traseraDer);
    void setup();
    void setSpeedRight(int vel)
    {
        _velR = vel;
    }
    void setSpeedLeft(int vel)
    {
        _velL = vel;
    }
    void setSpeedVuelta(int vel) { _velvuelta = vel; };
    void setSpeed180(int vel) { _vuelta180 = vel; };
    void setVelocidad(int velL, int velR){
        _velR = velR;
        _velL = velL;
    };
    void avanzar();
    void atras();
    void stop();
    void girarDer();
    void girarIzq();

private:
    // pines
    int _INA;
    int _INB;
    int _delanteraDer;
    int _delanteraIzq;
    int _traseraDer;
    int _traseraIzq;

    // high o low
    int _velR;
    int _velL;
    int _velvuelta;
    int _vuelta180;
};

#endif