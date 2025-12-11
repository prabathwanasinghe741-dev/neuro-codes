// Motor Pins
const int ENA = 10;
const int IN1 = 9;
const int IN2 = 8;
const int IN3 = 7;
const int IN4 = 6;
const int ENB = 5;

// Sensor Pins
const int lineLeft = A0;
const int lineMid = A1;
const int lineRight = A2;
const int sideLeft = A3;
const int sideRight = A4;
const int trigPin = 11;
const int echoPin = 12;

void setup() {
  // Configure Motors as Output
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Configure Sensors as Input
  pinMode(lineLeft, INPUT);
  pinMode(lineMid, INPUT);
  pinMode(lineRight, INPUT);
  pinMode(sideLeft, INPUT);
  pinMode(sideRight, INPUT);
  
  // Configure Ultrasonic
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  Serial.begin(9600); // For debugging
}
