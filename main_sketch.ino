// Motor driver pins
#define IN1 8   // Left Motor IN1
#define IN2 9   // Left Motor IN2
#define IN3 10  // Right Motor IN3
#define IN4 11  // Right Motor IN4

// IR sensor pins
#define IR_LEFT 2
#define IR_MID 3
#define IR_RIGHT 4

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
}

void loop() {
  int left = digitalRead(IR_LEFT);
  int mid = digitalRead(IR_MID);
  int right = digitalRead(IR_RIGHT);

  // line position based dicitions
  if(mid == HIGH && left == LOW && right == LOW) { //mid only
    moveForward();
  }else if(mid == LOW && left == HIGH && right == LOW) { // left only
    turnLeft();
  }else if(mid == LOW && left == LOW && right == HIGH) { // right only
    turnRight();
  } else if(mid == LOW && left == LOW && right == LOW) { // all are out
    /// Write Ultra Sonic Codes
  }else if(mid == HIGH && left == HIGH && right == HIGH) { // all are on line
    turnRight();
    
  }else {
    stopMotors();
  }
  delay(10);
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

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
