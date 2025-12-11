// --- PIN DEFINITIONS ---
// Motor Driver Pins (L298N)
const int ENA = 10;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENB = 5;

// Sensor Pins
const int trigPin = 11;
const int echoPin = 12;
const int sideSensorLeft = A3;  // Your extra left sensor
const int sideSensorRight = A4; // Your extra right sensor

// --- TUNING VARIABLES (Adjust these based on your robot's speed) ---
const int stopDistance = 15;    // Stop if wall is closer than 15cm
const int gapDistance = 25;     // If distance > 25cm, we found a gap
const int turnSpeed = 150;      // Speed for turning (0-255)
const int driveSpeed = 120;     // Speed for moving forward (0-255)
const int turnTime = 400;       // How long to turn to check for a gap (milliseconds)

void setup() {
  // Motor Pins Setup
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Ultrasonic Pins Setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Side Sensors Setup
  pinMode(sideSensorLeft, INPUT);
  pinMode(sideSensorRight, INPUT);

  Serial.begin(9600);
  delay(2000); // 2 second delay before starting to give you time to move away
}

void loop() {
  int distance = getDistance();
  
  // Debugging: Print distance to Serial Monitor
  Serial.print("Dist: ");
  Serial.println(distance);

  // --- LOGIC: FIREWALL GAUNTLET ---
  
  if (distance < stopDistance) {
    // 1. Wall Detected! Stop and find the gap.
    stopBot();
    delay(500); 
    findGap(); 
  } 
  else {
    // 2. Path is clear, move forward
    moveForward();
    
    // 3. Optional: Use side sensors to stay centered while moving through gap
    // If Left sensor detects obstacle (LOW typically means detected for IR)
    if (digitalRead(sideSensorLeft) == LOW) { 
       turnRight(); // Nudge right slightly
       delay(50);
    }
    // If Right sensor detects obstacle
    else if (digitalRead(sideSensorRight) == LOW) {
       turnLeft(); // Nudge left slightly
       delay(50);
    }
  }
  
  delay(50); // Small stability delay
}

// --- HELPER FUNCTIONS ---

void findGap() {
  // Strategy: Look Right first. If blocked, look Left.
  
  // 1. Turn Right and Check
  turnRight();
  delay(turnTime); 
  stopBot();
  delay(200);
  
  int rightDist = getDistance();
  
  if (rightDist > gapDistance) {
    // Gap found on the right! 
    // The loop will restart, see the clear path, and moveForward() automatically.
    return; 
  }
  
  // 2. If Right was blocked, Turn Left (Double time to return to center + turn left)
  turnLeft();
  delay(turnTime * 2); 
  stopBot();
  delay(200);
  
  int leftDist = getDistance();
  
  if (leftDist > gapDistance) {
    // Gap found on the left!
    // Loop will handle the movement.
    return;
  }
  
  // 3. If BOTH are blocked (Corner case), Spin around or reverse
  // For now, let's just turn right again to try a wider scan
  turnRight();
  delay(turnTime);
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Calculate distance in cm
  
  if (distance == 0) return 999; // If sensor times out, assume open space
  return distance;
}

// --- MOTOR MOVEMENTS ---

void moveForward() {
  analogWrite(ENA, driveSpeed);
  analogWrite(ENB, driveSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopBot() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void turnLeft() {
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH); // Left motor back
  digitalWrite(IN3, HIGH); // Right motor forward
  digitalWrite(IN4, LOW);
}

void turnRight() {
  analogWrite(ENA, turnSpeed);
  analogWrite(ENB, turnSpeed);
  digitalWrite(IN1, HIGH); // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  // Right motor back
  digitalWrite(IN4, HIGH);
}
