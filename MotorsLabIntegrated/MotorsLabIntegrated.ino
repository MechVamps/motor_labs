#include <SharpIR.h>
#include <Servo.h>
#include <Stepper.h>


// -----------------------------------------------------------------------------
// FSM INITIALIZATIONS
const int ledPin = 13;
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

// -----------------------------------------------------------------------------
// STEPPER MOTOR INITIALIZATIONS (FROM AMY)
// Stepper
// Number of steps per output rotation
// Number of steps per output rotation
int stepsPerRevolution = 10;
int deg = 0;
int prevdist = 0;

// Create Instance of Stepper library
Stepper myStepper(stepsPerRevolution, 2,8,9,10);
// free pins: 13, 10, 9, 2, 8

// Ultrasonic
const int trigPin = A1;
const int echoPin = A2;
long duration;
int distanceUS;


// -----------------------------------------------------------------------------
// RC SERVOMOTOR INITIALIZATIONS (FROM ADVAIT)

Servo servo;
int value;
int reading;
long force = 0;
// -----------------------------------------------------------------------------

// DC MOTOR INITIALIZATIONS (FROM JESSICA)
// IR

#define irPin A3 // A3
#define model 1080
int ir_distance;
float volts;
SharpIR mySensor = SharpIR(irPin, model);

// ENCODERS 

int encA = 3;
int encB = 4;

// MOTORS //

// DC

int pwmPin = 6;
int in_1 = 5;  //Initialize pin D10 to drive motor1
int in_2 = 7; //Initialize pin D11 to drive motor1
int pwm_speed;
int prev_dc_mode = 0;

const int maxpwm = 255;
const int minpwm = 90;
const int max_speed_windup = 1000;
const int max_pos_windup = 1000;
//Encoder encoder(encA, encB);

//PID controller variables for position controller

int target_pos = 0;
long curr_pos = 0;
double kp_p=0.001, ki_p=0.001, kd_p=5;
int e_pos_sum = 0;
int e_pos_last = 0;

//PID controller variables for speed controller

double curr_speed, target_speed;
double kp_s=0.35, ki_s=0.001, kd_s=0.15;
int e_speed_sum = 0;
int e_speed_last = 0;


// SYSTEM STATE

String gui="";
long debounce = 500;
long c_time = 0;
int sys_state = 0;  
String myString="";
int c_button2_st;
int p_button2_st = 0;
int np=1;
int prev_gui = 0;

long prev_pos = 0;
long curr_pos_sp = 0;
unsigned long prev_time = millis();
int last_pwm = 0;
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
  // set the speed at 20 rpm:
  myStepper.setSpeed(20);
  /// Ultrasonic
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
// -----------------------------------------------------------------------------
// RC SERVOMOTOR SETUP (FROM ADVAIT)
  servo.attach(11); //servo at digital pin 11
  servo.write(0); //initial point for servo
// -----------------------------------------------------------------------------
// DC MOTOR SETUP (FROM JESSICA)
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP); 
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(pwmPin, OUTPUT); 
  
  target_pos = 0;
  curr_pos = 0;
    
  // DC motor off
  analogWrite(pwmPin, 0);
  digitalWrite(in_1, LOW);
  digitalWrite(in_2, LOW);
// -----------------------------------------------------------------------------
  pinMode(ledPin, OUTPUT);
//  pinMode(green_light_pin, OUTPUT);
//  pinMode(blue_light_pin, OUTPUT);
}

void loop() {
//  digitalWrite(ledPin , HIGH);
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
  distanceUS = duration * 0.034 / 2;
  deg = stepsPerRevolution*1.8;
      // END ULTRASONIC SENSOR CODE FROM AMY
   // -----------------------------------------------------------------------------
   
  force = read_force(A0);
  ir_distance = mySensor.distance();
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
    analogWrite(pwmPin, 0);
    servo.write(0);
    digitalWrite(ledPin , LOW);
//    Serial.println("Ready");
  }

  if (S||P) {
    
    //Serial.println(reading);
    char messageBuf[150];
    sprintf(messageBuf, "S:%d,I:%d,U:%d,F:%d", (Gui||Del), ir_distance, distanceUS, force);
    Serial.println(messageBuf);
    

    if (count == 0) {
      // all motors inactive
    }

    if (RC) {
      digitalWrite(ledPin , LOW);
      // -----------------------------------------------------------------------------
      // START OF RC SERVOMOTOR CODE FROM ADVAIT
      reading = analogRead(A0); //attached to analog 0
      value = map(reading, 300, 1023, 0, 255);
      servo.write(value);
//      Serial.println("RC Servo");
   
  //    delay(100);
      // END OF RC SERVOMOTOR CODE FROM ADVAIT
      // -----------------------------------------------------------------------------

    } else if (DC) {
      digitalWrite(ledPin , LOW);
  //    Serial.println("DC Motor is active (Sensor Controlled)");
      // -----------------------------------------------------------------------------
      // START OF DC MOTOR CODE FROM JESSICA
      //  DC MOTOR CONTROL USING IR SENSOR 
//      Serial.println("DC Motor");
//      Serial.println(pwm_speed);
      if (ir_distance >= 10 && ir_distance <= 30){
      pwm_speed = map(ir_distance, 3, 30, 0, 255);
      analogWrite(pwmPin, pwm_speed);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2 , LOW);
//      digitalWrite(ledPin,HIGH);
    }
    else{
      pwm_speed = 0;
      analogWrite(pwmPin, 0);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2 , HIGH);
      
    }
      // END OF DC MOTOR CODE FROM JESSICA
      // -----------------------------------------------------------------------------

    } else if (SM) {
        digitalWrite(ledPin,HIGH);
        /// Move
        if (distanceUS < prevdist){
          myStepper.step(stepsPerRevolution);
//          Serial.println(deg);
        }
        if (distanceUS == prevdist){
//          Serial.println(deg);
        }
        if (distanceUS > prevdist){
          myStepper.step(-stepsPerRevolution);
//          Serial.println(-deg);
        }
      
        prevdist = distanceUS; 
      }
  }


  if (Gui||Del) {

//     Serial.println("GUI Controlled, all motors active");
    // use GUI output to control motors
    // Serial.println("GUI Coming");
    int sv;
    int st;
    int steps_from_gui;
    digitalWrite(ledPin , HIGH);

    if (Serial.available() > 0) {
//      analogWrite(red_light_pin, 0);
//      analogWrite(green_light_pin, 0);
//      analogWrite(blue_light_pin, 255);
      Serial.println("receiving string");

      GUImessage = Serial.readStringUntil("#");
      Serial.println(GUImessage);

//      Serial.println("GUI COMING");

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
//      Serial.println(sv);

//      analogWrite(red_light_pin, sv);
//      analogWrite(green_light_pin, 0);
//      analogWrite(blue_light_pin, 0);
      servo.write(sv);
      st = fifthValue.substring(2).toInt();
      steps_from_gui = st/1.8;
      myStepper.step(steps_from_gui);
    }

  }

//Serial.print("R: "); Serial.print(R); Serial.print("\t");
//Serial.print("P: "); Serial.print(P); Serial.print("\t");
//Serial.print("S: "); Serial.print(S); Serial.print("\t");
//Serial.print("Gui: "); Serial.print(Gui); Serial.print("\t");
//Serial.print("Del: "); Serial.print(Del); Serial.print("\t");
//Serial.print("count = "); Serial.println(count);
}
