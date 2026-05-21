#include <Arduino.h>
#include <TB6612_ESP32.h>

// Define motor driver pins
#define AIN1 14
#define AIN2 12
#define PWMA 13
#define BIN1 27
#define BIN2 26
#define PWMB 25

TB6612FNG motor(AIN1, AIN2, PWMA, BIN1, BIN2, PWMB);

void setup() {
  Serial.begin(115200);

  motor.begin();  // Initialize motor driver
}

void loop() {
  motor.setMotorPower(255); // Set motor power (0 to 255)
  motor.motorForward();     // Move forward
  delay(2000);
  
  motor.motorStop();        // Stop motor
  delay(1000);
  
  motor.motorBackward();    // Move backward
  delay(2000);
}
