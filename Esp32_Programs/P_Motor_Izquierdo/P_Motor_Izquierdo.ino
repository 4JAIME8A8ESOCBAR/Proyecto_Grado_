#define PWMA 32
#define AIN1 33
#define AIN2 25
#define STBY 13

#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

void setup() {

  //--------------------------------
  // PINES
  //--------------------------------

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  //--------------------------------
  // ACTIVAR DRIVER
  //--------------------------------

  digitalWrite(STBY, HIGH);

  //--------------------------------
  // DIRECCION
  //--------------------------------

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  //--------------------------------
  // PWM ESP32
  //--------------------------------

  ledcSetup(
      PWM_CHANNEL,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMA,
      PWM_CHANNEL);

  //--------------------------------
  // VELOCIDAD
  //--------------------------------

  ledcWrite(PWM_CHANNEL, 200);
}

void loop() {
}