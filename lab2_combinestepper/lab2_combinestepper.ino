// Stepper
#include <Stepper.h>

// Number of steps per output rotation
int stepsPerRevolution = 2000;

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 2,8,9,10);
// free pins: 13, 10, 9, 2, 8

// Ultrasonic
const int trigPin = A1;
const int echoPin = A2;
long duration;
int distance;

void setup() {
  /// Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  /// Stepper
  // set the speed at 20 rpm:
  myStepper.setSpeed(20);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  /// Acquire Distance
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance); 
  
  // Adjust speed
//  if (motorSpeed > 0) {
//    myStepper.setSpeed(motorSpeed);
//    // step 1/100 of a revolution:
//    myStepper.step(stepsPerRevolution / 100);
//  }
//  stepsPerRevolution = 

  // Adjust Distance
  if distance <10{
    stepsPerRevolution = stepsPerRevolution *distance
  }
  else{
    stepsPerRevolution = stepsPerRevolution * 0.5*distance
  }
  
  /// Move
  // step one revolution in one direction:
  Serial.println("clockwise");
  myStepper.step(stepsPerRevolution);
  delay(500);

  // step one revolution in the other direction:
  Serial.println("counterclockwise");
  myStepper.step(-stepsPerRevolution);
  delay(500);
}
