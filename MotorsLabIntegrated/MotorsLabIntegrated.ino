#include <Servo.h>

// -----------------------------------------------------------------------------
// FSM INITIALIZATIONS
const int buttonPin = 12;

// Initialize States
bool R = 1; // start in READY, waiting for button to be pressed, no motors active
bool Gui = 0;    // motors controlled by GUI
bool S = 0; // motors controlled by sensors
bool P = 0;    // button is pressed, wait for long press to switch to GUI control or short press to stay in sensor control
bool Del = 0; // delay state if button pressed while in GUI control

// Initialize Variables
bool TMR = 0;
int TMR_duration = 1000; // 1 sec timer for long press to switch to and from GUI control       
bool B = 0;
bool F = 0;
bool I = 0; // used for TMR_enable
bool D = 0;
bool G = 0;
bool L = 0; // used for counter
bool CNT = 0; // used for counter
int count = 0;
bool RC = 0; // when true, RC Servomotor is active
bool DC = 0; // when true, DC Motor is active
bool SM = 0; // when true, Stepper Motor is active
String GUImessage; // string of data coming from GUI
int red_light_pin= 6;
int green_light_pin = 5;
int blue_light_pin = 4;

// -----------------------------------------------------------------------------
// STEPPER MOTOR INITIALIZATIONS (FROM AMY)
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

// -----------------------------------------------------------------------------
// RC SERVOMOTOR INITIALIZATIONS (FROM ADVAIT)

Servo servo;
int value;
int reading;
long force = 0;
// -----------------------------------------------------------------------------

// DC MOTOR INITIALIZATIONS (FROM JESSICA)

// -----------------------------------------------------------------------------

long read_force(int pin) // From sensors lab
{
  int fsrReading;     // the analog reading from the FSR resistor divider
  int fsrVoltage;     // the analog reading converted to voltage
  unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
  unsigned long fsrConductance; 
  long fsrForce;       // Finally, the resistance converted to force

  fsrReading = analogRead(pin);  // get raw counts from FSR
  // Serial.print("Analog reading = ");
  // Serial.println(fsrReading);
 
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);
  // Serial.print("Voltage reading in mV = ");
  // Serial.println(fsrVoltage);  
 
  if (fsrVoltage == 0) {
    // Serial.println("No pressure");  
    fsrForce = 0;
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    // Serial.print("FSR resistance in ohms = ");
    // Serial.println(fsrResistance, 4);
 
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;
    // Serial.print("Conductance in microMhos: ");
    // Serial.println(fsrConductance, 4);
 
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;  
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30;          
    }
  }

  return fsrForce; 
}

void setup() {
  // put your setup code here, to run once:
// FSM SETUP
  Serial.begin(9600);
  // Button
  pinMode(buttonPin, INPUT_PULLUP);
// -----------------------------------------------------------------------------
// STEPPER MOTOR SETUP (FROM AMY)
  /// Stepper
  // Turn these pins on for PWM OUTPUT
  pinMode(right_forward, OUTPUT);
  pinMode(right_reverse, OUTPUT);
  pinMode(left_forward, OUTPUT);
  pinMode(left_reverse, OUTPUT);
  // turn all the motor off
  digitalWrite(right_forward, LOW);
  digitalWrite(right_reverse, LOW);
  digitalWrite(left_forward, LOW);
  digitalWrite(left_reverse, LOW);
  /// Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
// -----------------------------------------------------------------------------
// RC SERVOMOTOR SETUP (FROM ADVAIT)
  servo.attach(11); //servo at digital pin 11
  servo.write(0); //initial point for servo
  // -----------------------------------------------------------------------------
  pinMode(red_light_pin, OUTPUT);
  pinMode(green_light_pin, OUTPUT);
  pinMode(blue_light_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // -----------------------------------------------------------------------------
  // START ULTRASONIC SENSOR CODE FROM AMY
  // Acquire Distance
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
      // END ULTRASONIC SENSOR CODE FROM AMY
   // -----------------------------------------------------------------------------
   
  force = read_force(A0);
  // Finite State Machine 4 Block Structure:
  // BLOCK 1: Inputs
  bool BTN = digitalRead(buttonPin); // read buttonPin
  bool TMR_enable = P||Del;  
  bool TMR = timer(TMR_enable, TMR_duration);

  if (D) {
    count = count + 1; // count transitions into sensor control
  }
  if (G) {
    count = 0; // reset counter
  }
  if (count == 1) {
    CNT = 0;
    RC = 1;
    DC = 0;
    SM = 0;
  } else if (count == 2) {
    RC = 0;
    DC = 1;
    SM = 0;
  } else if (count == 3) {
    RC = 0;
    DC = 0;
    SM = 1;
  } else if (count == 4) {
    CNT = 1;
  }

  // BLOCK 2: Transitions
  bool A = R&&!BTN; // Latch on READY
       B = R&&BTN; // Transition from READY to Press
  bool C = P&&BTN&&!TMR; // Latch on Press
       D = P&&!BTN&&!TMR&&!CNT; // Transition from Press to Sensor Control
  bool E = S&&!BTN&&!CNT; // Latch on Sensor Control
       F = S&&BTN; // Transition from Sensor Control to Press
       G = P&&TMR; // Transition from Press to GUI Control
  bool H = Gui&&!BTN; // Latch on GUI Control
       I = Gui&&BTN; // Transition from GUI Control to Delay
  bool J = Del&&BTN&&!TMR; // Latch on Delay
  bool K = Del&&!BTN; // Transition from Delay to GUI Control
       L = Del&&TMR; // Transition from Delay to Sensor Control
  bool M = S&&CNT; // Transition from Sensor Control to READY

  // BLOCK 3: End States
  R = A||M;
  P = B||C||F;
  S = D||E||L;
  Gui = G||H||K;
  Del = I||J;

  // BLOCK 4: Outputs
  if (R) {
    // set LED to green
    // all motors off
    count = 0;
    CNT = 0;
    TMR = 0;
  //  Serial.println("Ready");
  }

  if (S||P) {
    analogWrite(red_light_pin, 255);
    analogWrite(green_light_pin, 255);
    analogWrite(blue_light_pin, 0);
    //Serial.println(reading);
    char messageBuf[150];
    sprintf(messageBuf, "S:%d,I:%d,U:%d,F:%d", (Gui||Del), 0, distance, force);
    Serial.println(messageBuf);
    

    //Serial.print("status:"); Serial.print(Gui||Del); Serial.print(";"); 
    //Serial.print("i:"); Serial.print(0); Serial.print(";");
    //Serial.print("u:"); Serial.print(distance); Serial.print(";");
    //Serial.print("f:"); Serial.print(force); Serial.println(";");

    if (count == 0) {
      // all motors inactive
    }

    if (RC) {
      // -----------------------------------------------------------------------------
      // START OF RC SERVOMOTOR CODE FROM ADVAIT
      reading = analogRead(A0); //attached to analog 0
      value = map(reading, 300, 1023, 0, 255);
      servo.write(value);
   
  //    delay(100);
      // END OF RC SERVOMOTOR CODE FROM ADVAIT
      // -----------------------------------------------------------------------------

    } else if (DC) {
  //    Serial.println("DC Motor is active (Sensor Controlled)");
      // -----------------------------------------------------------------------------
      // START OF DC MOTOR CODE FROM JESSICA
      
      // END OF DC MOTOR CODE FROM JESSICA
      // -----------------------------------------------------------------------------

    } else if (SM) {
      // TODO
    }
    // set LED to blue, yellow, or pink depending on motor
  }


  if (Gui||Del) {
    analogWrite(red_light_pin, 255);
    analogWrite(green_light_pin, 255);
    analogWrite(blue_light_pin, 255);
    // Serial.println("GUI Controlled, all motors active");
    // set LED to white
    // use GUI output to control motors
    // Serial.println("GUI Coming");
    int sv;

    if (Serial.available() > 0) {
      analogWrite(red_light_pin, 0);
      analogWrite(green_light_pin, 0);
      analogWrite(blue_light_pin, 255);
      GUImessage = Serial.readStringUntil("\n");
      Serial.println("GUI COMING");

      int commaIndex = GUImessage.indexOf(",");
      int secondCommaIndex = GUImessage.indexOf(",", commaIndex + 1);
      int thirdCommaIndex = GUImessage.indexOf(",", secondCommaIndex + 1);
      int forthCommaIndex = GUImessage.indexOf(",", thirdCommaIndex+1);
      String firstValue = GUImessage.substring(0, commaIndex);
      String secondValue = GUImessage.substring(commaIndex + 1, secondCommaIndex);
      String thirdValue = GUImessage.substring(secondCommaIndex + 1,thirdCommaIndex);
      String forthValue = GUImessage.substring(thirdCommaIndex + 1,forthCommaIndex);
      String fifthValue = GUImessage.substring(forthCommaIndex + 1);
      int gui = firstValue.substring(2).toInt();
      int dc_mode = secondValue.substring(3).toInt();
      int dc_val = thirdValue.substring(3).toInt();
      sv = forthValue.substring(2).toInt();
      Serial.println(sv);

      analogWrite(red_light_pin, sv);
      analogWrite(green_light_pin, 0);
      analogWrite(blue_light_pin, 0);
      servo.write(sv);
      int st = fifthValue.substring(2).toInt();
    }

  }


}
