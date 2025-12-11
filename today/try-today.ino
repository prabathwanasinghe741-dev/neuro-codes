// --- 1. Include Libraries ---
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <ESP32Servo.h> // Recommended library for ESP32 PWM control

// --- 2. Pin Definitions ---

// Motor Driver (L298N - No PWM used for speed control)
#define MOT_A_IN1 16 // GPIO 16 (Motor A Direction 1)
#define MOT_A_IN2 17 // GPIO 17 (Motor A Direction 2)
#define MOT_B_IN3 18 // GPIO 18 (Motor B Direction 1)
#define MOT_B_IN4 19 // GPIO 19 (Motor B Direction 2)

// Servo Motors (Arm)
#define SERVO_1_PIN 25 // Base/Shoulder
#define SERVO_2_PIN 26 // Elbow
#define SERVO_3_PIN 27 // Wrist
#define SERVO_4_PIN 32 // Gripper/Claw

// Ultrasonic Sensors (Trigger/Echo)
#define US_FRONT_TRIG 5  // Front Trigger
#define US_FRONT_ECHO 15 // Front Echo (Requires Voltage Divider)
#define US_LEFT_TRIG 33  // Left Trigger
#define US_LEFT_ECHO 34  // Left Echo (Input-Only Pin, Requires Voltage Divider)
#define US_RIGHT_TRIG 35 // Right Trigger
#define US_RIGHT_ECHO 39 // Right Echo (Input-Only Pin, Requires Voltage Divider)

// IR Sensors (Line Follower/Digital Input)
#define IR_LEFT 36   // Input-Only Pin
#define IR_CENTER 4
#define IR_RIGHT 2

// I2C Pins for TCS34725 (SDA/SCL - Default ESP32 I2C Pins)
// SDA is GPIO 21, SCL is GPIO 22. These are handled by the Wire.h library.

// --- 3. Global Variables and Objects ---

// Color Sensor Object
// Integration time and gain are critical for color detection accuracy. 
// You may need to tune these values based on your robot's lighting conditions.
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Servo Objects
Servo servo1, servo2, servo3, servo4;

// Color Detection Thresholds (***CRITICAL FOR COMPETITION LOGIC***)
// These values must be calibrated by testing the sensor on Red and Blue obstacles.
const int RED_THRESHOLD = 5000;  // Example: High Red value, low Blue/Green
const int BLUE_THRESHOLD = 5000; // Example: High Blue value, low Red/Green

// --- 4. Motor Control Functions ---

void stopMotors() {
    digitalWrite(MOT_A_IN1, LOW);
    digitalWrite(MOT_A_IN2, LOW);
    digitalWrite(MOT_B_IN3, LOW);
    digitalWrite(MOT_B_IN4, LOW);
    // Note: ENA/ENB are permanently HIGH (or connected to 5V) in this setup
}

void moveForward() {
    digitalWrite(MOT_A_IN1, HIGH);
    digitalWrite(MOT_A_IN2, LOW);
    digitalWrite(MOT_B_IN3, HIGH);
    digitalWrite(MOT_B_IN4, LOW);
}

void turnLeft() {
    // Left motor reverse, Right motor forward (pivot turn)
    digitalWrite(MOT_A_IN1, LOW);
    digitalWrite(MOT_A_IN2, HIGH);
    digitalWrite(MOT_B_IN3, HIGH);
    digitalWrite(MOT_B_IN4, LOW);
}

void turnRight() {
    // Left motor forward, Right motor reverse (pivot turn)
    digitalWrite(MOT_A_IN1, HIGH);
    digitalWrite(MOT_A_IN2, LOW);
    digitalWrite(MOT_B_IN3, LOW);
    digitalWrite(MOT_B_IN4, HIGH);
}

// --- 5. Sensor Reading Functions ---

// Ultrasonic Sensor Reading
long readUltrasonic(int trigPin, int echoPin) {
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH for 10 microsecond
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin, HIGH);
    // Calculating the distance (cm)
    // Speed of sound is 343 m/s or 0.0343 cm/µs. Divide by 2 for one-way distance.
    long distance = duration * 0.034 / 2;
    return distance;
}

// IR Sensor Reading
int readIR(int irPin) {
    // Assuming IR sensor outputs LOW when over a black line (or object)
    // and HIGH otherwise.
    return digitalRead(irPin);
}

// Color Sensor Reading and Decoding
String readColor() {
    uint16_t r, g, b, c; // Red, Green, Blue, Clear
    
    // Read raw data from the sensor
    // You may need to tune the 'true' parameter (turn on LED for illumination)
    tcs.getRawData(&r, &g, &b, &c); 

    // Normalize or use raw data for comparison
    Serial.printf("Raw Color: R:%d, G:%d, B:%d, C:%d\n", r, g, b, c);

    // XBOTIX Logic: Identify Red or Blue
    // This logic needs to be calibrated in the actual environment.
    if (r > RED_THRESHOLD && b < BLUE_THRESHOLD) {
        return "RED";
    } else if (b > BLUE_THRESHOLD && r < RED_THRESHOLD) {
        return "BLUE";
    } else {
        return "OTHER"; // Or "NONE"
    }
}


// --- 6. Servo Control Function ---

void setArmPosition(int angle1, int angle2, int angle3, int angle4) {
    servo1.write(angle1);
    servo2.write(angle2);
    servo3.write(angle3);
    servo4.write(angle4);
    delay(500); // Wait for servos to move
}

// --- 7. Setup Function ---

void setup() {
    Serial.begin(115200);
    delay(100);

    // Initialize Motor Pins as Outputs
    pinMode(MOT_A_IN1, OUTPUT);
    pinMode(MOT_A_IN2, OUTPUT);
    pinMode(MOT_B_IN3, OUTPUT);
    pinMode(MOT_B_IN4, OUTPUT);
    stopMotors(); // Ensure motors are off at startup

    // Initialize Ultrasonic Sensor Pins
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);
    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);
    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);

    // Initialize IR Sensor Pins as Inputs (Input-Only pins like 36 don't need pinMode)
    pinMode(IR_CENTER, INPUT);
    pinMode(IR_RIGHT, INPUT);
    // GPIO 36 is input-only and defaults to INPUT, so pinMode is optional but safe.

    // Initialize Servos
    // Allow the ESP32 to allocate the required PWM channels
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3); 
    
    servo1.attach(SERVO_1_PIN);
    servo2.attach(SERVO_2_PIN);
    servo3.attach(SERVO_3_PIN);
    servo4.attach(SERVO_4_PIN);

    setArmPosition(90, 90, 90, 90); // Set arm to a safe, neutral position

    // Initialize Color Sensor (I2C)
    if (tcs.begin()) {
        Serial.println("TCS34725 Color Sensor Found!");
    } else {
        Serial.println("No TCS34725 found! Check wiring (SDA/SCL) and power.");
        while (1); // Halt robot if color sensor is critical (which it is for Section 2)
    }
    tcs.setInterrupt(false);  // Turn on the integrated LED light
    
    Serial.println("Setup Complete. Ready for Competition!");
}

// --- 8. Loop Function (Simple State Machine Structure) ---

// Define the competition stages
enum RobotState {
    NAVIGATION_SECTION_1,
    CRYPTIC_MAZE_SECTION_2,
    CIPHER_GATE_SECTION_3,
    DONE
};

RobotState currentState = NAVIGATION_SECTION_1;
String secretKey = ""; // Stores the 3-bit binary number (e.g., "101")
int obstacleCount = 0; // Counter for the 3 obstacles in Section 2

void loop() {
    long frontDist = readUltrasonic(US_FRONT_TRIG, US_FRONT_ECHO);
    int centerIR = readIR(IR_CENTER);
    
    // --- State Machine Logic ---
    switch (currentState) {

        case NAVIGATION_SECTION_1:
            // Example Logic: Simple obstacle avoidance
            if (frontDist < 20) { // If obstacle is too close (20 cm)
                stopMotors();
                // Check sides for clear path
                if (readUltrasonic(US_LEFT_TRIG, US_LEFT_ECHO) > readUltrasonic(US_RIGHT_TRIG, US_RIGHT_ECHO)) {
                    turnLeft();
                } else {
                    turnRight();
                }
                delay(500); // Turn for half a second
            } else {
                moveForward();
            }

            // Transition condition: Assuming the robot detects a line or marker to start Section 2
            // For example, if the center IR sensor detects a start line for 5 seconds.
            // Simplified transition for demonstration:
            // if (some_marker_is_detected) {
            //     currentState = CRYPTIC_MAZE_SECTION_2;
            // }
            break;

        case CRYPTIC_MAZE_SECTION_2:
            // Line following combined with color detection

            // 1. Line Follow (using IR sensors)
            if (centerIR == LOW) { // Assume LOW = Black Line
                moveForward();
            } else if (readIR(IR_LEFT) == LOW) {
                turnLeft(); // Minor adjustment
            } else if (readIR(IR_RIGHT) == LOW) {
                turnRight(); // Minor adjustment
            } else {
                // Robot lost the line, may be at an obstacle or end of line.
                stopMotors(); 
                
                // 2. Color Detection Logic
                String detectedColor = readColor();
                
                if (detectedColor == "RED" && obstacleCount < 3) {
                    // Xbotix Rule: Red -> Left Side (Bit 1)
                    secretKey += "1";
                    // Arm Sequence to clear path on the LEFT
                    setArmPosition(30, 120, 100, 10); // Example arm movement
                    // Move out of the way, continue maze
                    obstacleCount++;
                } 
                else if (detectedColor == "BLUE" && obstacleCount < 3) {
                    // Xbotix Rule: Blue -> Right Side (Bit 0)
                    secretKey += "0";
                    // Arm Sequence to clear path on the RIGHT
                    setArmPosition(150, 120, 100, 10); // Example arm movement
                    // Move out of the way, continue maze
                    obstacleCount++;
                }

                // 3. Transition Check
                if (obstacleCount >= 3) {
                    Serial.print("3-Bit Secret Key Obtained: ");
                    Serial.println(secretKey);
                    currentState = CIPHER_GATE_SECTION_3;
                }
            }
            break;

        case CIPHER_GATE_SECTION_3: {
            stopMotors();
            
            // Convert 3-bit binary (e.g., "101") to decimal (e.g., 5)
            int decimalCode = 0;
            if (secretKey.length() == 3) {
                // Convert binary string to decimal integer
                decimalCode = (secretKey[0] - '0') * 4 + (secretKey[1] - '0') * 2 + (secretKey[2] - '0') * 1;
                Serial.print("Cipher Gate Decimal Code: ");
                Serial.println(decimalCode);
            }

            // Logic to interact with the 7-segment display/gate using the decimalCode
            // This would involve precise movement and possibly servo control based on the code.
            
            currentState = DONE; // Finish the competition
            break;
        }

        case DONE:
            stopMotors();
            Serial.println("Mission Complete!");
            delay(5000); // Wait 5 seconds before re-checking (or just stop)
            break;
    }
    
    // Safety delay to prevent reading sensors/actuating too quickly
    delay(50);
}
