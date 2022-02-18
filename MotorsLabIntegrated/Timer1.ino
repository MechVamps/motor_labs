//initialize boolean variables for the timer
boolean TMR_waiting = true;  // waiting state. By setting this true the FSM will BEGIN in waiting!
boolean TMR_timing = false;  // timing state

unsigned long TMR_starttime = millis(); 
int TMR_elapsed = 0;

bool timer(bool TMR_enable, int TMR_duration) {
  
// Timer Block 1

// Timer Block 2

bool TMR_A = TMR_waiting&&TMR_enable;  // waiting to timing
bool TMR_B = TMR_waiting&&!TMR_enable; // latch on waiting
bool TMR_C = TMR_timing&&!TMR_enable;  // timing to waiting
bool TMR_D = TMR_timing&&TMR_enable;   // latch on timing

// Timer Block 3

TMR_waiting = TMR_B||TMR_C;
TMR_timing = TMR_A||TMR_D;

// Timer Block 4

if (TMR_waiting) {
  TMR_starttime = millis();
  TMR_elapsed = 0;
  TMR = false;
}

if (TMR_timing) {     
  TMR = false;
  TMR_elapsed = millis() - TMR_starttime;
  if (TMR_elapsed >= TMR_duration) {
  TMR = true;
}
}
return TMR;
}
