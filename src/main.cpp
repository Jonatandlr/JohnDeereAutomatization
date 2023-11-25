// #include <Arduino.h>
// #include <Giroscopio.h>

// Giroscopio giros(1);
// uint32_t LoopTimer;
// void setup()
// {
//     Serial.begin(115200);
//     pinMode(13, OUTPUT);
//     digitalWrite(13, HIGH);

//     giros.setup();
//     LoopTimer = micros();
// }

// void loop()
// {
//     Serial.println(giros.getRateYaw());
    
//     while (micros() - LoopTimer < 4000);
//     LoopTimer = micros();
// }