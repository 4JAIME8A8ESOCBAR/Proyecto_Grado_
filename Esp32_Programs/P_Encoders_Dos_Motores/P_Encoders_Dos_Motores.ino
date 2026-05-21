//
//==================================================
// MOTOR DERECHO
//==================================================
//

#define PWMB 26
#define BIN1 27
#define BIN2 14

//
//==================================================
// MOTOR IZQUIERDO
//==================================================
//

#define PWMA 32
#define AIN1 33
#define AIN2 25

//
//==================================================
// STBY
//==================================================
//

#define STBY 13

//
//==================================================
// ENCODER DERECHO
//==================================================
//

#define ENCODER_RIGHT_A 34
#define ENCODER_RIGHT_B 35

//
//==================================================
// ENCODER IZQUIERDO
//==================================================
//

#define ENCODER_LEFT_A 39
#define ENCODER_LEFT_B 36

//
//==================================================
// PWM CONFIG
//==================================================
//

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

#define PWM_CHANNEL_RIGHT 0
#define PWM_CHANNEL_LEFT 1

//
//==================================================
// VARIABLES ENCODERS
//==================================================
//

volatile long encoderRightPos = 0;
volatile long encoderLeftPos = 0;

//
//==================================================
// PWM ACTUAL
//==================================================
//

int pwmRight = 225;
int pwmLeft = 180;

//
//==================================================
// SETUP
//==================================================
//

void setup() {

  Serial.begin(115200);

  //--------------------------------
  // MOTORES
  //--------------------------------

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  //--------------------------------
  // DIRECCIONES
  //--------------------------------

  // RIGHT
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);

  // LEFT
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);

  //--------------------------------
  // PWM
  //--------------------------------

  ledcSetup(
      PWM_CHANNEL_RIGHT,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMB,
      PWM_CHANNEL_RIGHT);

  ledcSetup(
      PWM_CHANNEL_LEFT,
      PWM_FREQ,
      PWM_RESOLUTION);

  ledcAttachPin(
      PWMA,
      PWM_CHANNEL_LEFT);

  //--------------------------------
  // VELOCIDADES
  //--------------------------------

  ledcWrite(
      PWM_CHANNEL_RIGHT,
      pwmRight);

  ledcWrite(
      PWM_CHANNEL_LEFT,
      pwmLeft);

  //--------------------------------
  // ENCODERS
  //--------------------------------

  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);

  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_A),
      handleRightEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_RIGHT_B),
      handleRightEncoderB,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_LEFT_A),
      handleLeftEncoderA,
      CHANGE);

  attachInterrupt(
      digitalPinToInterrupt(ENCODER_LEFT_B),
      handleLeftEncoderB,
      CHANGE);

  Serial.println("SYSTEM READY");
}

//
//==================================================
// LOOP
//==================================================
//

void loop() {

  static unsigned long lastTime = 0;

  static long lastRight = 0;
  static long lastLeft = 0;

  //--------------------------------
  // CADA 500 ms
  //--------------------------------

  if (millis() - lastTime >= 500) {

    long rightPos;
    long leftPos;

    noInterrupts();

    rightPos = encoderRightPos;
    leftPos = encoderLeftPos;

    interrupts();

    //--------------------------------
    // DELTAS
    //--------------------------------

    long deltaRight = rightPos - lastRight;
    long deltaLeft = leftPos - lastLeft;

    //--------------------------------
    // RPM
    //--------------------------------

    float rpmRight =
        (deltaRight / 4230.0) *
        (60000.0 / 500.0);

    float rpmLeft =
        (deltaLeft / 4230.0) *
        (60000.0 / 500.0);
    rpmLeft = -rpmLeft ; // Inversion Signo

    //--------------------------------
    // SERIAL
    //--------------------------------

    Serial.print("RIGHT -> ");

    Serial.print("Pulses: ");
    Serial.print(rightPos);

    Serial.print(" | RPM: ");
    Serial.print(rpmRight);

    Serial.print(" | PWM: ");
    Serial.print(pwmRight);

    Serial.print(" || ");

    Serial.print("LEFT -> ");

    Serial.print("Pulses: ");
    Serial.print(leftPos);

    Serial.print(" | RPM: ");
    Serial.print(rpmLeft);

    Serial.print(" | PWM: ");
    Serial.println(pwmLeft);

    //--------------------------------
    // UPDATE
    //--------------------------------

    lastRight = rightPos;
    lastLeft = leftPos;

    lastTime = millis();
  }
}

//
//==================================================
// INTERRUPTS RIGHT
//==================================================
//

void handleRightEncoderA() {

  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);

  if (A == B)
    encoderRightPos++;
  else
    encoderRightPos--;
}

void handleRightEncoderB() {

  bool A = digitalRead(ENCODER_RIGHT_A);
  bool B = digitalRead(ENCODER_RIGHT_B);

  if (A != B)
    encoderRightPos++;
  else
    encoderRightPos--;
}

//
//==================================================
// INTERRUPTS LEFT
//==================================================
//

void handleLeftEncoderA() {

  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);

  if (A == B)
    encoderLeftPos++;
  else
    encoderLeftPos--;
}

void handleLeftEncoderB() {

  bool A = digitalRead(ENCODER_LEFT_A);
  bool B = digitalRead(ENCODER_LEFT_B);

  if (A != B)
    encoderLeftPos++;
  else
    encoderLeftPos--;
}