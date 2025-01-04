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
const int safeDistance  = 7;

// movement duration
#define ROTATION_DURATION 150
#define FORWARD_DURATION 700

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
  motor1.setSpeed(150); // Speed ranges from 0-255
  motor2.setSpeed(150);
  motor3.setSpeed(150);
  motor4.setSpeed(150);

  // Initialize ultrasonic sensor pins
  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);

  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  Serial.begin(9600); // For debugging

  // delay(1000);
  // motor1.run(FORWARD);
  // delay(2000);
  // motor1.run(RELEASE);
  // delay(1000);
  // motor2.run(FORWARD);
  // delay(2000);
  // motor2.run(RELEASE);
  // delay(1000);
  // motor3.run(FORWARD);
  // delay(2000);
  // motor3.run(RELEASE);
  // delay(1000);
  // motor4.run(FORWARD);
  // delay(2000);
  // motor4.run(RELEASE);


}

// Function to stop all motors
void stopMotors() {
  Serial.println("stop");
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

//turn left test
void turnLeft(long leftDistance){
  long frontDistance = measureDistance(trigFront, echoFront);
  while(abs(frontDistance - leftDistance) >= 2){
    frontDistance = measureDistance(trigFront, echoFront);
    Serial.print("left distance: "); Serial.print(leftDistance); Serial.print(" "); Serial.print("front distance: "); Serial.println(frontDistance); 
    rotateLeft();
  }
  
}

bool turn = true;
// Loop
void loop() {
  // // Measure distances
  long frontDistance = measureDistance(trigFront, echoFront);
  long leftDistance = measureDistance(trigLeft, echoLeft);
  long rightDistance = measureDistance(trigRight, echoRight);

  // // Debugging distances
  // Serial.print("Front: ");  Serial.print(frontDistance); Serial.print(" cm, ");
  // Serial.print("Left: ");   Serial.print(leftDistance); Serial.print(" cm, ");
  // Serial.print("Right: ");  Serial.println(rightDistance);
  
 
  delay(12000);
  if(turn){
    turnLeft(leftDistance);
  }
  turn = false;
  delay(100); // Small delay for stability
}
