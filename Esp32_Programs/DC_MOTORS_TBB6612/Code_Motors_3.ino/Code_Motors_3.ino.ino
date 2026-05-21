#define AIN1 26
#define AIN2 27
#define PWMA 25
#define STBY 33

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  ledcAttach(PWMA, 1000, 8); // frecuencia 1kHz, resolución 8 bits

  digitalWrite(STBY, HIGH);
}

void loop() {

  // adelante
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWMA, 200);

  delay(3000);

  // atrás
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWMA, 200);

  delay(3000);

  // stop
  ledcWrite(PWMA, 0);

  delay(2000);
}
