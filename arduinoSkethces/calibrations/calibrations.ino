#include <AFMotor.h> // Include motor shield library

// Create motor objects
AF_DCMotor motor1(4); // Front-Left Motor
AF_DCMotor motor2(3); // Front-Right Motor
AF_DCMotor motor3(1); // Rear-Left Motor
AF_DCMotor motor4(2); // Rear-Right Motor

// Ultrasonic Sensor Pins
const int trigFront = A0;
const int echoFront = A1;

const int trigLeft = A4;
const int echoLeft = A5;

const int trigRight = A2;
const int echoRight = A3;

// Threshold distance (in cm)
const int safeDistance  = 5;

// movement duration
#define ROTATION_DURATION 2000
#define FORWARD_DURATION 500

// Function to measure distance from ultrasonic sensor
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH);
  long distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}

// Setup
void setup() {
  // Initialize motors
  motor1.setSpeed(100); // Speed ranges from 0-255
  motor2.setSpeed(100);
  motor3.setSpeed(100);
  motor4.setSpeed(100);

  // Initialize ultrasonic sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  Serial.begin(9600); // For debugging
}

// Function to stop all motors
void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Function to move forward
void moveForward() {
  Serial.println("farward ");
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(FORWARD_DURATION);
  stopMotors();
}


// Function to rotate left
void rotateLeft() {
  Serial.println("left ");
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(ROTATION_DURATION);
  stopMotors();
}

// Function to rotate right
void rotateRight() {
  Serial.println("right ");
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(ROTATION_DURATION);
  stopMotors();
}
 bool turn = true;

// Loop
void loop() {
  // Measure distances
  long frontDistance = measureDistance(trigFront, echoFront);
  long leftDistance = measureDistance(trigLeft, echoLeft);
  long rightDistance = measureDistance(trigRight, echoRight);

  // Debugging distances
  Serial.print("Front: ");  Serial.print(frontDistance); Serial.print(" cm, ");
  Serial.print("Left: ");   Serial.print(leftDistance); Serial.print(" cm, ");
  Serial.print("Right: ");  Serial.println(rightDistance);
  
  delay(1000); // Small delay for stability
 
  if(turn){
    // Decision-makin
    rotateLeft();
    delay(1000);
    moveForward();
    delay(1000);
    rotateRight();
    delay(1000);
  }
  turn = false; 
  stopMotors();
  delay(100); // Small delay for stability
}
