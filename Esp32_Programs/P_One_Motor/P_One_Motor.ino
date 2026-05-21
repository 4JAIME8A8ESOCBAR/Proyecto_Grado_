#define PWMB 26 
#define BIN1 27 
#define BIN2 14 
#define STBY 13

#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

void setup() {

  //--------------------------------
  // PINES
  //--------------------------------

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  //--------------------------------
  // ACTIVAR DRIVER
  //--------------------------------

  digitalWrite(STBY, HIGH);

  //--------------------------------
  // DIRECCION
  //--------------------------------

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  //--------------------------------
  // PWM ESP32
  //--------------------------------

  ledcSetup(
      PWM_CHANNEL,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMB,
      PWM_CHANNEL);

  //--------------------------------
  // VELOCIDAD
  //--------------------------------

  ledcWrite(PWM_CHANNEL, 200);
}

void loop() {
}