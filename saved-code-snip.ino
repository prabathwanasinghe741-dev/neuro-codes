// Motor driver pins
#define IN1 8   // Left Motor IN1
#define IN2 9   // Left Motor IN2
#define IN3 10  // Right Motor IN3
#define IN4 11  // Right Motor IN4

// IR sensor pins
#define IR_LEFT 2
#define IR_MID 3
#define IR_RIGHT 4

// Ultrasonics
#define US1_TRIG 5
#define US1_ECHO 6 // left

#define US2_TRIG 7
#define US2_ECHO 12 // mid

#define US3_TRIG 13
#define US3_ECHO A0 // right

// Variable to remember the last known position of the line
// -1 = Left, 0 = Center, 1 = Right
int lastLinePosition = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MID, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set pins
  pinMode(US1_TRIG, OUTPUT); pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT); pinMode(US2_ECHO, INPUT);
  pinMode(US3_TRIG, OUTPUT); pinMode(US3_ECHO, INPUT);
  Serial.begin(9600);
}

long getDistance(int trigPin, int echoPin) {
  // Send pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo duration
  long duration = pulseIn(echoPin, HIGH);

  // Convert to cm
  long distance = duration * 0.034 / 2;
  return distance;
}

void loop() {
  int left = digitalRead(IR_LEFT);
  int mid = digitalRead(IR_MID);
  int right = digitalRead(IR_RIGHT);

    // Read all 3 ultrasonic sensors
  int leftD = getDistance(US1_TRIG, US1_ECHO);
  int midD = getDistance(US2_TRIG, US2_ECHO);
  int rightD = getDistance(US3_TRIG, US3_ECHO);

  // Serial.print("left: ");
  // Serial.print(leftD);
  // Serial.println(" cm");
  // Serial.print("mid: ");
  // Serial.print(midD);
  // Serial.println(" cm");
  // Serial.print("right: ");
  // Serial.print(rightD);
  // Serial.println(" cm");

  // security pupose and ultra sonic sonic
  if(midD >= 15 && leftD >= 15 && rightD >= 15) {
    moveForward();
  } else if (midD < 15 || leftD < 15 || right < 15) {
    stopMotors();
    if(leftD > rightD) {
      turnLeftHIGHPOWER();
    }else if(leftD < rightD) {
      turnRightHIGHPOWER();
    }
  }


  // line position based dicitions

  // // code commenting: start
  // if(mid == HIGH && left == LOW && right == LOW) { //mid only
  //   moveForward();
  // }else if(mid == LOW && left == HIGH && right == LOW) { // left only
  //   turnLeft();
  // }else if(mid == LOW && left == LOW && right == HIGH) { // right only
  //   turnRight();
  // } else if(mid == LOW && left == LOW && right == LOW) { // all are out
  //   /// Write Ultra Sonic Codes
  // }else if(mid == HIGH && left == HIGH && right == HIGH) { // all are on line
  //   turnRight();

  // }else {
  //   stopMotors();
  // }
  // delay(10);
  // // code commenting: end
  delay(100);
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeftHIGHPOWER() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(500);
}

void turnRightHIGHPOWER() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(500);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
