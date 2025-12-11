// ===== MOTOR DRIVER PINS =====
#define IN1 8   // Left Motor IN1
#define IN2 9   // Left Motor IN2
#define IN3 10  // Right Motor IN3
#define IN4 11  // Right Motor IN4

// ===== ULTRASONIC SENSOR PINS =====
#define US_LEFT_TRIG 5
#define US_LEFT_ECHO 6
#define US_MID_TRIG 7
#define US_MID_ECHO 12
#define US_RIGHT_TRIG 13
#define US_RIGHT_ECHO 4   // changed from A0 → digital pin 4

// ===== IR SENSOR PINS =====
#define IR_LEFT A1
#define IR_MID  A2
#define IR_RIGHT A3

// ===== FUNCTION: GET DISTANCE =====
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20 ms timeout
  long distance = duration * 0.034 / 2;
  if (distance == 0) distance = 500; // assume far if no reading
  return distance;
}

// ===== MOTOR FUNCTIONS =====
void stopRobot() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnLeft() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

// ===== LINE FOLLOWING FUNCTION =====
void followLine() {
  int leftVal  = analogRead(IR_LEFT);
  int midVal   = analogRead(IR_MID);
  int rightVal = analogRead(IR_RIGHT);

  int threshold = 500; // adjust as needed

  bool leftBlack  = leftVal  > threshold;
  bool midBlack   = midVal   > threshold;
  bool rightBlack = rightVal > threshold;

  if (midBlack) {
    moveForward();
  } else if (leftBlack) {
    turnLeft();
  } else if (rightBlack) {
    turnRight();
  } else {
    stopRobot();
  }
}

void setup() {
  Serial.begin(9600);

  // Ultrasonic pins
  pinMode(US_LEFT_TRIG, OUTPUT); pinMode(US_LEFT_ECHO, INPUT);
  pinMode(US_MID_TRIG, OUTPUT);  pinMode(US_MID_ECHO, INPUT);
  pinMode(US_RIGHT_TRIG, OUTPUT); pinMode(US_RIGHT_ECHO, INPUT);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // IR sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MID, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

void loop() {
  int leftVal  = analogRead(IR_LEFT);
  int midVal   = analogRead(IR_MID);
  int rightVal = analogRead(IR_RIGHT);
  int threshold = 500;

  bool lineDetected = (leftVal > threshold || midVal > threshold || rightVal > threshold);

  if (lineDetected) {
    followLine();
  } else {
    long leftDist  = getDistance(US_LEFT_TRIG, US_LEFT_ECHO);
    long midDist   = getDistance(US_MID_TRIG, US_MID_ECHO);
    long rightDist = getDistance(US_RIGHT_TRIG, US_RIGHT_ECHO);

    Serial.print("LEFT: "); Serial.print(leftDist);
    Serial.print(" MID: "); Serial.print(midDist);
    Serial.print(" RIGHT: "); Serial.println(rightDist);

    if (leftDist < 10 || midDist < 10 || rightDist < 10) {
      stopRobot();
      delay(200);

      if (leftDist > rightDist) {
        turnLeft(); delay(400);
      } else {
        turnRight(); delay(400);
      }
    } else {
      moveForward();
    }
  }

  delay(50);
}
