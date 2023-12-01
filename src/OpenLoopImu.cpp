// #include <Arduino.h>
// #include <Giroscopio.h>
// #include <motores.h>
// #include<PID_v1.h>


// float angle;
// int pwm=127;
// int count=0;



// motores motor(11, 8, 9, 10, 7, 6);
// Giroscopio giros(1);

// double Setpoint=0;
// double Input, Output;

// PID myPID(&Input, &Output, &Setpoint, 6.4812, 5.3326, 0.00, DIRECT);


// void setup(){
//     Serial.begin(115200);
//     motor.setup();  
//     motor.setSpeedLeft(127);

//     motor.setSpeedRight(127); 
//     giros.setup(); 
//     myPID.SetMode(AUTOMATIC);
//     myPID.SetOutputLimits(75,255);
    
// }

// void loop(){
//     motor.avanzar();
//     angle=giros.getAngle("Yaw");
//     Input=angle;
//     myPID.Compute();
//     motor.setSpeedRight(Output);
// }