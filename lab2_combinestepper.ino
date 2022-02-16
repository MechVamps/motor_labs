// Stepper
int right_forward = 10;
int right_reverse = 3;
int left_forward = 6;
int left_reverse = 9;
int delay_time_on = 100; // how long should each wheel turn?
int delay_time_off = 100; // delay between tests

// Ultrasonic
const int trigPin = 2;
const int echoPin = 8;
long duration;
int distance;

void setup() {
  /// Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  
  /// Stepper
  // Turn these pins on for PWM OUTPUT
  pinMode(right_forward, OUTPUT);
  pinMode(right_reverse, OUTPUT);
  pinMode(left_forward, OUTPUT);
  pinMode(left_reverse, OUTPUT);
  // turn all the motors off
  digitalWrite(right_forward, LOW);
  digitalWrite(right_reverse, LOW);
  digitalWrite(left_forward, LOW);
  digitalWrite(left_reverse, LOW);
  // for debugging.  The output will appear on the serial monitor
  // To open the serial monitor, click the magnafing glass icon in the upper right corner
  Serial.begin(9600);      // open the serial port at 9600 bps
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

  /// Alter delay depending on distance
  delay_time_on = delay_time_on + distance*0.1;
  delay_time_off = delay_time_off + distance*0.1;
  if (distance < 10){
    delay_time_on = 50;
    delay_time_off = 50;
  }
  if (distance > 10 && distance < 20){
    delay_time_on = 100;
    delay_time_off = 100;
  }

  if (distance > 20 && distance < 30){
    delay_time_on = 200;
    delay_time_off = 200;
  }

  if (distance > 30){
    delay_time_on = 500;
    delay_time_off = 500;
  }
  
  
  
  /// Move
  Serial.println("Right Forward Test");
  digitalWrite(right_forward, HIGH);
  delay(delay_time_on);
  digitalWrite(right_forward, LOW);
  delay(delay_time_off);

  Serial.println("Left Reverse Test");
  digitalWrite(left_reverse, HIGH);
  delay(delay_time_on);
  digitalWrite(left_reverse, LOW);
  delay(delay_time_off);


}
