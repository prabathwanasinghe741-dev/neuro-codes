// ================================
//        PIN DEFINITIONS
// ================================

// Motor driver pins (L298N / L293D)
// මෝටර් මොඩියුල් පින් නිර්වචනය
#define IN1 13  // Left motor forward (වම් මෝටරයේ forward දිශාව)
#define IN2 12  // Left motor backward (වම් මෝටරයේ backward දිශාව)
#define IN3 14  // Right motor forward (දකුණු මෝටරයේ forward දිශාව)
#define IN4 27  // Right motor backward (දකුණු මෝටරයේ backward දිශාව)

// IR sensor pins (Digital input)
// IR සෙන්සර් වල reading ගන්න pins
#define IR_LEFT 32
#define IR_MID 33
#define IR_RIGHT 25

// Track last turn direction
// හැරීම් මතක තබා ගන්නා වේරියබල් එක
String lastTurn = "NONE";

void setup() {
  // Motor pins as output
  // මෝටර් පින් Output ලෙස සැකසීම
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // IR sensor pins as input
  // IR සෙන්සර් වලින් data ගන්න Input ලෙස සෙට් කිරීම
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MID, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Serial monitor start (Serial මොනිටරය ආරම්භ කරන්න)
  Serial.begin(115200); 
}

void loop() {
  // Read sensor values
  // සෙන්සර් වල values කියවීම සහ වෙන වෙනම ගබඩා කිරීම
  int left = digitalRead(IR_LEFT);
  int mid = digitalRead(IR_MID);
  int right = digitalRead(IR_RIGHT);

  // Print values (values print කරනවා අතවශ්‍ය නැත.) 
  Serial.print("L: "); Serial.print(left);
  Serial.print(" M: "); Serial.print(mid);
  Serial.print(" R: "); Serial.println(right);

  // ================================
  //        LINE FOLLOW LOGIC
  //        රේඛාව දිගේ ගමන් කරන ලොජික්
  // ================================

  // ----- TURN LEFT -----
  // වම් පැත්තේ IR එක line එක detect කරද්දි මැශින් එක වමට හැරේ
  // If left detects line → turn left
  if (left == HIGH || (left == HIGH && mid == HIGH)) {
    turnLeft();
    // Save last turn direction (අවසාන හැරීම් දිශාව වම ලෙස save කරන්න)
    lastTurn = "LEFT";
  }

  // ----- TURN RIGHT -----
  // දකුණු IR එක line එක detect කරද්දි bot එක දකුණට හැරේ
  // If right detects line → turn right
  else if (right == HIGH || (right == HIGH && mid == HIGH)) {
    turnRight();
    // Save last turn direction (අවසාන හැරීම් දිශාව දකුණ ලෙස save කරන්න)
    lastTurn = "RIGHT";
  }

  // ----- MOVE FORWARD -----
  // මැද IR එකට පමණක් line එක දකින්නම් → forward යනවා
  else if (mid == HIGH && left == LOW && right == LOW) {
    moveForward();
    lastTurn = "NONE";
  }

  // ----- LINE LOST: RECOVERY -----
  // sensors 3ම LOW නම් line එක නැතිවෙලා. එවිට මතකය පරික්ශා කරන්න.
  // All LOW → Lost line → try recovering using last turn
  else if (left == LOW && mid == LOW && right == LOW) {

    // If last turn was left → try turning right
    // අවසන් turn එක LEFT නම් → RIGHT හැරිලා යන්න.
    if (lastTurn == "LEFT") {
      turnRight();
      // විශාල කාලයක් හැරෙන්න එපා
      delay(400);
    }

    // If last turn was right → try turning left
    // අවසන් turn එක RIGHT නම් → LEFT හැරලා line එක සෙවන්න
    else if (lastTurn == "RIGHT") {
      turnLeft();
      delay(400);
    }

    stopMotors(); // motors stop (මෝටර නවතන්න)
  }

  delay(30); // Small delay to stabilize (reading stable කරන්න)
}


// ================================
//      MOVEMENT FUNCTIONS
//      චලනයේ functions
// ================================

// Move forward (මුහුණට යනවා)
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Turn left (වමට හැරෙනවා)
void turnLeft() {
  digitalWrite(IN1, LOW);   // Left motor backward
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);  // Right motor forward
  digitalWrite(IN4, LOW);
}

// Turn right (දකුණට හැරෙනවා)
void turnRight() {
  digitalWrite(IN1, HIGH);  // Left motor forward
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);   // Right motor backward
  digitalWrite(IN4, HIGH);
}

// Stop motors (මෝටර් නවත්වනවා)
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
