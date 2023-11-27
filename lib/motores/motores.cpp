#include <Arduino.h>
#include <motores.h>

motores::motores(int INA,int delanteraIzq,int traseraIzq,int INB,int delanteraDer,int traseraDer){
    _INB=INB;
    _delanteraDer=delanteraDer;
    _traseraDer=traseraDer;
    _INA=INA;
    _delanteraIzq=delanteraIzq;
    _traseraIzq=traseraIzq;
};

void motores::setup(){
    pinMode(_INA,OUTPUT);
    pinMode(_delanteraDer,OUTPUT);
    pinMode(_traseraDer,OUTPUT);
    pinMode(_INB,OUTPUT);
    pinMode(_delanteraIzq,OUTPUT);
    pinMode(_traseraIzq,OUTPUT);

    analogWrite(_INA,LOW);
    analogWrite(_INB,LOW);
};




void motores::avanzar(){
    analogWrite(_INA,_velL);
    digitalWrite(_delanteraDer,HIGH);
    digitalWrite(_traseraDer,LOW);
    analogWrite(_INB,_velR);
    digitalWrite(_delanteraIzq,HIGH);
    digitalWrite(_traseraIzq,LOW);
};

void motores::atras(){
    analogWrite(_INA,_velL);
    digitalWrite(_delanteraDer,LOW);
    digitalWrite(_traseraDer,HIGH);
    
    analogWrite(_INB,_velR);
    digitalWrite(_delanteraIzq,LOW);
    digitalWrite(_traseraIzq,HIGH);

};

void motores::stop(){
    analogWrite(_INA,LOW);
    digitalWrite(_delanteraDer,LOW);
    digitalWrite(_traseraDer,LOW);
    analogWrite(_INB,LOW);
    digitalWrite(_delanteraIzq,LOW);
    digitalWrite(_traseraIzq,LOW);;
};


void motores::girarDer(){
    analogWrite(_INA,_velL);
    digitalWrite(_delanteraDer,LOW);
    digitalWrite(_traseraDer,HIGH);
    analogWrite(_INB,_velR);
    digitalWrite(_delanteraIzq,HIGH);
    digitalWrite(_traseraIzq,LOW);
};


void motores::girarIzq(){
    analogWrite(_INA,_velL);
    digitalWrite(_delanteraDer,HIGH);
    digitalWrite(_traseraDer,LOW);
    analogWrite(_INB,_velR);
    digitalWrite(_delanteraIzq,LOW);
    digitalWrite(_traseraIzq,HIGH);
}
