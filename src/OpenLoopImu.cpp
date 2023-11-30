#include <Arduino.h>
#include <Giroscopio.h>
#include <motores.h>


float angle;
int pwm=127;
int count=0;

motores motor(11, 8, 9, 10, 7, 6);
Giroscopio giros(1);

void setup(){
    Serial.begin(115200);
    motor.setup();  
    motor.setSpeedLeft(127);

    motor.setSpeedRight(pwm); 
    giros.setup(); 
}

void loop(){
    if(count==500){
        pwm=255;
        motor.setSpeedRight(pwm);
    }
    if(count==1000){
        pwm=127;
        motor.setSpeedRight(pwm);
    }
    motor.avanzar();
    angle=giros.getAngle("Yaw");
    Serial.print(pwm);
    Serial.print(",");
    Serial.println(angle);
    count++;
    delay(1);
}