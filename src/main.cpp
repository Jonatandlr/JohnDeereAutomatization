// #include <Arduino.h>
// #include <motores.h>

// int encoderL = 3;
// int countLeft = 0;
// int tickPerRevolution = 48;

// motores motor(11, 8, 9, 10, 7, 6);
// const float pulsosPorRevolucion = 1000;  // Ajusta esto según la resolución de tu encoder
// const float radioRueda = 0.05;  // Ajusta esto según el radio de tus ruedas en metros
// unsigned long tiempoPrevio = 0;

// void countL()
// {
//     countLeft++;
//     // Serial.println("izquierda: " + String(countLeft));
// }

// void setup()
// {
//     Serial.begin(115200);
//     motor.setup();
//     attachInterrupt(digitalPinToInterrupt(encoderL), countL, RISING);
//     motor.setSpeed(200);
//     tiempoPrevio = millis();  // Iniciar el temporizador
// }

// float calcularRPM() {
//   const int ticksPorVuelta = 48;  // Ajusta esto según la configuración de tu encoder
//   const int milisegundosPorMinuto = 60000;

//   noInterrupts();  // Deshabilita interrupciones para leer la variable de manera segura
//   int ticks = countLeft;
//   countLeft = 0;  // Reinicia el contador de ticks
//   interrupts();  // Vuelve a habilitar interrupciones

//   float rpm = (ticks / (float)ticksPorVuelta) * milisegundosPorMinuto;  // Calcula las RPM

//   return rpm;
// }

// void loop()
// {
//     if (countLeft<48)
//     {
//         motor.avanzar();
//     }
//     else{
//         motor.stop();
//         Serial.println("izquierda: " + String(countLeft));
//     }
    
// }