#include <Servo.h>
#include <SharpIR.h>
//#include <Encoder.h>

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

void setup() {

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
Serial.begin(9600);

}

long prev_pos = 0;
long curr_pos_sp = 0;
unsigned long prev_time = millis();
int last_pwm = 0;

void loop() {

  // Read data from IR sensor 
 
   ir_distance = mySensor.distance();

////  DC MOTOR CONTROL USING Ultra-sonic SENSOR 
     
    if (ir_distance >= 10 && ir_distance <= 30){
      pwm_speed = map(ir_distance, 3, 30, 0, 255);
      analogWrite(pwmPin, pwm_speed);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2 , LOW);
    }
    else{
      pwm_speed = 0;
      analogWrite(pwmPin, 0);
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2 , HIGH);
      
    }
  Serial.print(ir_distance);
  Serial.print(" ");
  Serial.print(pwm_speed);
  Serial.println();
}
    
