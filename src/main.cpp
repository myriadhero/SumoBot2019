/* SER300 Sumo Robot
Kirill Duplyakin  217182546

Trimester 1, 2019
___________________________

Microcontroller:        ATMEGA328P in an Arduino Nano package
Motor drivers:          2x VNH3SP30
Time of Flight sensors: 2x VL53L0X
IR sensors:             4x TCRT5000, has both a digital and an analog output
Motors:                 2x JGB37-520B, Magnetic quadrature encoder included

___________________________
I/O

Microcontroller has these external inputs:
- 4 IR line sensors, 2 on the front and 2 on the back, 
  - IR sensor digital outputs are connected to Arduino on PCINT pins
- 2 Time of Flight sensors on the front, I2C
- 2 Encoders on the motors, PCINT as well, same bank of pins as IR
- 1 Button, in pullup config (when not pressed, reads 1)

- Additionally it controls a PWM extender module via I2C that returns back timing signal on interrupt INT0

10 Outputs are controlled through PWM extender module, namely:
- 6 motor channels, 3 left and 3 right
- 4 LEDs

To be able to initialise the two ToFs, one of them needs to be put in reset state, 
while the other ToF is given a non-default address 
- 1 digital output pin is used for that

___________________________

Programming approach used is state machine programming. 

- Task 1: seek and attack, avoid lines - 3 states
- Task 2: seek, evade, and avoid arena edge - 3 states
- Competition: seek, avoid lines, attack, evade - 4 states

-> First a simple selection is implemented at the end of setup function,
      to let the user pick the Task/Competition
      Initially red light is flashing, indicating that the user hasn't made a selection
      - the button can be pressed to select a different mode
      - once the button is pressed, the countdown starts for the game
      - if button is pressed again during the countdown, 
          the countdown is reset and next mode is selected
      - Yellow light indicates Task 1
      - Green light  inidicates Task 2
      - Blue light indicates competition mode

-> Task 1
  - The bot starts this task facing away from dummy opponent, it has to turn around 180, then go into seek mode
  - once seek mode identifies potential enemy, attack mode is activated
  - both modes can be interrupted by detecting arena edge, in this case, bot goes back into seek after 
      avoiding the line


-> Task 2
  - The bot starts this task in seek mode
  - once seek mode identifies potential enemy, evade mode is activated
  - evade mode basically does a 90 degree turn away from the enemy and proceeds to the arena edge
  - once arena edge is reached, seek mode is activated again


-> Competition
  - The bot starts this task in seek mode
  - once seek mode identifies potential enemy, there's a different probability for attack and evade modes
    - it may be a good strategy to evade on initial startup, so the robot always tries to evade on the very first run
  - seek and attack can be interrupted by detecting arena edge, after avoiding line goes back to seek mode


// Safety functions include:
- motor stall detection is implemented as a difference between 


*/

//___________________________
// Libraries

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

//___________________________
// Library objects

//TOF object
VL53L0X ToF_Left;
VL53L0X ToF_Right;

// PCA9685 board var, called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// I used OLED display for debugging, because it was easier than serial (no need to stay tethered)
// however the program runs a lot slower with having to update the display
// so final version has it commented out
// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// #define OLED_ADDR   0x3C
// Adafruit_SSD1306 display(-1); // this is fixed, but just in case of reinstall or something
// #if (SSD1306_LCDHEIGHT != 64)  // check whether the settings are set up correctly
// #error("Height incorrect, please fix Adafruit_SSD1306.h!");
// #endif


//___________________________
// #defines

// ----------------------------
// bitwise operations, macro functions that simplify reading the program a bit
#define BV(x)              (1 << x) // get the register bit
#define setBit(P, B)       (P |= BV(B))
#define clearBit(P, B)     (P &= ~BV(B))
#define toggleBit(P, B)    (P ^= BV(B))
// ----------------------------


// arduino pin for timer
#define PTIMER_2ARD PD2 

// PCA9685 pinount
#define PTIMER 0

#define PMOTOR_Rpwm 6
#define PMOTOR_R1 4 
#define PMOTOR_R2 5

#define PMOTOR_Lpwm 3
#define PMOTOR_L1 2
#define PMOTOR_L2 1

#define PLED_BLUE 12
#define PLED_YELLOW 11
#define PLED_GREEN 15
#define PLED_RED 8

// LED MAX pwm defines, some LEDS are too bright, so this should be their maximum out of 4096
#define PLEDPWMMAX_BLUE 2500
#define PLEDPWMMAX_GREEN 500
#define PLEDPWMMAX_RED 3000
#define PLEDPWMMAX_YELLOW 4096

// Button pin
#define BUTTON PC1 // right now the button is connected to A1 = PC1

/* Game mode select
task 1 = 0 // YLW
task 2 = 1 // GRN
game = 2   // BLU
initial statup = 3 // RED
*/
#define TASK1 0
#define TASK2 1
#define GAME  2
#define INITSTART 3

/* Game States
  These are states in game that determine what the robot is doing now
  default is seek = 0 // BLU
  attack = 1  // GRN
  evade = 2   // YLW
  moveFromEdge = 3 // RED
*/
#define SEEK    0
#define ATTACK  1
#define EVADE   2
#define MOVEFROMEDGE 3


// Motor related defines, encoder, PID etc
#define ENC_LEFT PB0
#define ENC_RIGHT PB1
#define FORWARD true
#define BACKWARD false
#define RIGHT true
#define LEFT false
#define OFF 0
#define DISABLE true

#define KP 10     // I'm going to use encoder value/time, at 4 rps (240 rpm, and ~1m/s), and ~650 clicks per revolution
#define KI 20     // i'm expecting a value of around 260 and for full error I want an output of 3000-4000 for PWM.
#define KD 0      // that means proportional value of around 11-15

#define SPEEDFULL 320         // 300rpm -> 5rps -> (~650 clicks/rev) 3250clicks/s -> ~325 clicks/0.1s 
#define SPEEDHALF SPEEDFULL/2 // i'm refreshing encoder value at 10 Hz 
#define SPEEDQ SPEEDFULL/4

#define STALLTIME 3000  // maximum amount of time a motor is allowed to stall for
#define STALLSPEED 20   // if motors are below this speed, start the timer
#define STALLMAXCOUNT 5 // number of times the robot is allowed to stall per round
#define STALLRECOVERYTIME 15000 // wait for 15 sec to cool down

// Motor function converting helpers
#define TURNCONST 40        // degrees to encoder clicks for turning on spot
#define TURNEVADE 9         // these are in degrees*10, so 9 = 90 degrees
#define TURNSWEEP1 12
#define TURNSWEEP2 TURNSWEEP1*2
#define TURNAFTERSWEEP 6
#define TURNIRFRONT 15
#define TURNIRBACK 3
#define TURNAROUND 18

#define STRAIGHTCONST 50    // helper const to convert cm to encoder clicks for going straight
#define MOVEFROMEDGEDIST 5  // move *cm from edge

// used to simplify disabling motors a bit
#define MOTORSDISABLE lMotorOn(OFF,OFF,DISABLE);rMotorOn(OFF,OFF,DISABLE)

// IR defines
#define IRFRONT_LEFT PB2
#define IRFRONT_RIGHT PB3
#define IRBACK_LEFT PB4
#define IRBACK_RIGHT PB5

// ToF sensor XSHUT pin needed to set address
#define TOFLEFT_XSHUT PC2

// I2C addresses
#define TOFLEFT_ADDR   0x31
#define TOFRIGHT_ADDR   0x30

// Timing
#define PTIMEFREQ 1000        // desired timer freq, Hz
#define TSAMP_ENC 100         // how often the encoder value should be sampled, ms
#define ROUNDSTARTTIME 5000   // 5000 ms after a mode is selected to start the round
#define PMAXPULSE 4096        // maximum PCA9685 PWM value, full ON
#define PHALFPULSE 2048       // this is half width for PWM, max is 4096

// Time of Flight sensor ranges, mm
#define TOFMAXRANGE 1200 // ToF sensor max meaningful reading
#define TOFMINDIST 2 // ToF sensor reading when the shell is lifted

// Random
#define EVADECHANCE 2 // chance of program initiating evade instead of attack after seek, out of 0-9


//___________________________
// global variables

// timer variables
volatile uint16_t timer_INT = 0;    // timer as per interrupt
volatile uint16_t timer_stall = 0;  // time the motor has been in stall
volatile uint8_t stallCounter = 0;  // number of times the robot stalled in this game
volatile bool is_motorBeganStall = false;
volatile bool is_motorStalled = false;

// motor encoders & speed control
volatile uint16_t encR = 0; // counter for encoder clicks
volatile uint16_t encL = 0;
volatile uint16_t encL_prev = 0;  // used for PID calcs
volatile uint16_t encR_prev = 0;

// PID control for motors
double          rMotorInput, rMotorOutput, rMotorSetpoint, 
                lMotorInput, lMotorOutput, lMotorSetpoint;
PID rMotorPID(&rMotorInput, &rMotorOutput, &rMotorSetpoint, KP, KI, KD, DIRECT);
PID lMotorPID(&lMotorInput, &lMotorOutput, &lMotorSetpoint, KP, KI, KD, DIRECT);

// motor encoder (& IR sensor interrupts)
volatile uint8_t pcint_prevState = 0;     // byte of states
volatile bool irFront_left_state = false; // false is not triggered
volatile bool irFront_right_state = false;
volatile bool irBack_left_state = false;
volatile bool irBack_right_state = false;
volatile bool ir_isAnyDetected = false;    // overall check to compare whether a detection has been made

// ToF helpers
volatile uint16_t ToF_Left_distmm = 0;  // toDo: see if these variables are needed
volatile uint16_t ToF_Right_distmm = 0;
volatile bool ToF_Left_detect = false;  // false means sensor is showing > TOFMAXRANGE
volatile bool ToF_Right_detect = false; // false means sensor is showing > TOFMAXRANGE
volatile bool ToF_last_detect = false;  // remember last known position, 
                                        // whether enemy was to the right or left
volatile bool ToF_shellLifted = false;  // detect whether robot shell is on the radar

//__________________________
// Game Mode
volatile uint8_t gameMode = INITSTART; 

// Game states
volatile uint8_t gameState = SEEK;

// setup blinker
volatile uint16_t PledBlinkState = 0;  // used to create a cool blinking animation
volatile bool PledBlinkIncre = 1;     // true is increasing

// Task 1 specific:
// turn 180 on first run of task1
volatile bool is_t1_firstRun = true;

// Task 2 specific:


// Competition mode specific:
volatile bool is_gameFirstRun = true;




//___________________________
// functions

// this simply collects distance readings from ToF sensors and updates related flags
void checkToFdist(){
  // get range
  if(!ToF_Right.timeoutOccurred()) ToF_Right_distmm = ToF_Right.readRangeContinuousMillimeters();
  if(!ToF_Left.timeoutOccurred()) ToF_Left_distmm = ToF_Left.readRangeContinuousMillimeters();
  
  // when distance is further than 1.1-1.2 meters, it outputs ~8980, 
  // otherwise seems pretty reliable up to distances of 1m
  // so i need to close the gap of 20-40cm by meandering


  // there's a small range where ToF will detect the shell and I can use it to get away from 
  // the opponent, but I need to make sure this distance is small enough so that it's not mistaken for 
  // pushing the opponent.
  if(ToF_Left_distmm < TOFMINDIST && ToF_Left_distmm < TOFMINDIST) ToF_shellLifted = true;
  else ToF_shellLifted = false;

  // check if detection is within range
  if(ToF_Left_distmm < TOFMAXRANGE) ToF_Left_detect = true;
  else ToF_Left_detect = false;
  if(ToF_Right_distmm < TOFMAXRANGE) ToF_Right_detect = true;
  else ToF_Right_detect = false;

  // save last known detection, used to figure out wihch way to scan/evade
  if (ToF_Left_detect && !ToF_Right_distmm) ToF_last_detect = LEFT;
  if (!ToF_Left_detect && ToF_Right_distmm) ToF_last_detect = RIGHT;
}

//___________________________
// motor control functions
void updatePID(){
  // update interval of 100 ms
  if(timer_INT<TSAMP_ENC) return; // if less than 100ms has passed, no need to update
  
  // update timer first to make sure time is taken accurately
  timer_INT = 0;

  // stall protection (a little recursive action from further functions, more description below)
  if(is_motorStalled)return;

  // calculate the error
  lMotorInput = (double)(encL - encL_prev);
  rMotorInput = (double)(encR - encR_prev);
  encL_prev = encL;
  encR_prev = encR;

  // calculate PID output
  // setpoint is set in the caller function
  rMotorPID.Compute();
  lMotorPID.Compute();

  //_________________
  // Stall protection
  // If motor speed is set to some reasonable value, 
  // we need to check whether motor is running atleast at minimum speed
  if((lMotorSetpoint > STALLSPEED && encL_prev < STALLSPEED) || 
                (rMotorSetpoint > STALLSPEED && encR_prev < STALLSPEED))
    // stall condition detected for one of the wheels
    is_motorBeganStall = true;
  else is_motorBeganStall = false;

  if(is_motorBeganStall) timer_stall += TSAMP_ENC;
  else timer_stall = 0;

  if(timer_stall > STALLTIME){
    // maximum stall time has been reached
    is_motorStalled = true;
    pwm.setPin(PMOTOR_L1, OFF);
    pwm.setPin(PMOTOR_L2, OFF);
    pwm.setPin(PMOTOR_Lpwm, OFF);
    pwm.setPin(PMOTOR_R1, OFF);
    pwm.setPin(PMOTOR_R2, OFF);
    pwm.setPin(PMOTOR_Rpwm, OFF);
    // need to try to do something else!
    // try to evade!
    gameState = EVADE;
    timer_stall = STALLTIME - STALLTIME/2; // resetting this should be ok because the maneuver is going to be going back
    // I've also implemented max amount of stalls allowed
  }
  ///________________

  

}
// right motor control function
void rMotorOn(bool forward, uint16_t speedClicks, bool disable = false){
  // this function can be used for disabling the motors
  if (disable){
    pwm.setPin(PMOTOR_R1, OFF);
    pwm.setPin(PMOTOR_R2, OFF);
    pwm.setPin(PMOTOR_Rpwm, OFF);
    rMotorSetpoint = 0;
    return;
  }

  // set speed in click/0.1s, calculate PID
  rMotorSetpoint = speedClicks;
  updatePID();

  // stall protection
  if(is_motorStalled)return;

  // forward motion
  if(forward){
    pwm.setPin(PMOTOR_R1, PMAXPULSE);
    pwm.setPin(PMOTOR_R2, OFF);
  }
  else{ // backward motion
    pwm.setPin(PMOTOR_R1, OFF);
    pwm.setPin(PMOTOR_R2, PMAXPULSE);
  }
  // PWM signal is set from PID output
  pwm.setPin(PMOTOR_Rpwm, rMotorOutput);
}
// same as above, but for left motor
void lMotorOn(bool forward = true, uint16_t speedClicks = 0, bool disable = false){
  if (disable){
    pwm.setPin(PMOTOR_L1, OFF);
    pwm.setPin(PMOTOR_L2, OFF);
    pwm.setPin(PMOTOR_Lpwm, OFF);
    lMotorSetpoint = 0;
    return;
  }

  lMotorSetpoint = speedClicks;
  updatePID();

  // stall protection
  if(is_motorStalled)return;

  if(forward){
    pwm.setPin(PMOTOR_L1, PMAXPULSE);
    pwm.setPin(PMOTOR_L2, OFF);
  }
  else{
    pwm.setPin(PMOTOR_L1, OFF);
    pwm.setPin(PMOTOR_L2, PMAXPULSE);
  }
  pwm.setPin(PMOTOR_Lpwm, lMotorOutput);
}
// this function combines the above to turn around
void turnOnSpot(uint8_t degX10,bool turnRight, uint16_t turnSpeedClicks){
  
  // ~650 clicks per rev, i need to turn the wheels 
  uint16_t clicksToTurn = degX10*TURNCONST;

  // reset encoder 
  encL = 0;
  encR = 0;
  encL_prev = 0;
  encR_prev = 0;
  // timer_INT = 0; // don't have to null time here because updatePID() function takes care of it ok

  
  
  if(turnRight){ // left motor forward, right motor back
    while((encL+encR) / 2 < clicksToTurn){
      // avoid arena edge
      if(ir_isAnyDetected && gameState != MOVEFROMEDGE){
        gameState = MOVEFROMEDGE;
        return;
      }

      if (encL <= clicksToTurn) lMotorOn(turnRight,turnSpeedClicks);
      else lMotorOn(OFF,OFF,DISABLE);
      
      if(encR <= clicksToTurn) rMotorOn(!turnRight,turnSpeedClicks);
      else rMotorOn(OFF,OFF,DISABLE);

      // stall protection
      if(is_motorStalled)return;

      // if robot is in seek mode, interrupt this function on detection
      checkToFdist(); // this is placed outside the below statement to keep checking and help seek 
      if(gameState == SEEK && !is_t1_firstRun){ // in the right direction after evade()
        if (ToF_Left_detect || ToF_Right_detect) break;
      }
    }

  } 
  else{ // right motor forward, left motor back
    while((encL+encR) / 2 < clicksToTurn){

      // avoid arena edge
      if(ir_isAnyDetected && gameState != MOVEFROMEDGE){
        gameState = MOVEFROMEDGE;
        return;
      }

      if (encL < clicksToTurn) lMotorOn(turnRight,turnSpeedClicks);
      else lMotorOn(OFF,OFF,DISABLE);
      
      if(encR < clicksToTurn) rMotorOn(!turnRight,turnSpeedClicks);
      else rMotorOn(OFF,OFF,DISABLE);

      // stall protection
      if(is_motorStalled)return;

      checkToFdist();
      if(gameState == SEEK && !is_t1_firstRun){
        if (ToF_Left_detect || ToF_Right_detect) break;
      }
    }
  }
  // turn motors off again, just in case
  MOTORSDISABLE;
}
// this function combines the above to drive straight forwards or backwards
void driveStraight(bool forward, uint16_t driveSpeedClicks, uint8_t distCm = 0){
  
  // reset encoder 
  encL = 0;
  encR = 0;
  encL_prev = 0;
  encR_prev = 0;
  
  // if distance is set, follow distance
  if (distCm>0){
    //~650 clicks per 22cm, up to a maximum of 150cm
    uint16_t numClicks = distCm * STRAIGHTCONST;
    
    // drive forward all those clicks with disregard for IR
    while((encL+encR)/2 < numClicks){
      if(encL < numClicks) lMotorOn(forward,driveSpeedClicks);
      else lMotorOn(OFF,OFF,DISABLE);
      if(encR < numClicks) rMotorOn(forward,driveSpeedClicks);
      else rMotorOn(OFF,OFF,DISABLE);
      
      // stall protection
      if(is_motorStalled)return;
    }
    MOTORSDISABLE;
    return;
  }
  //else
  // drive til edge!
  while(!ir_isAnyDetected){ 
    lMotorOn(forward,driveSpeedClicks);
    rMotorOn(forward,driveSpeedClicks);

    // stall protection
    if(is_motorStalled)return;
  }
  
  // turn motors off again, just in case
  MOTORSDISABLE;
}


//___________________________
// Behaviour functions


//___________________________
// purpose of this function is to identify potential enemy and go to attack or evade
// avoiding line should be treated with priority as well
//___________________________
void seek(){
  // indicate seek mode:
  pwm.setPin(PLED_BLUE, PLEDPWMMAX_BLUE, true);
  pwm.setPin(PLED_YELLOW, OFF,true);
  pwm.setPin(PLED_GREEN, OFF,true);
  pwm.setPin(PLED_RED, OFF, true);

  // update ToF measurements
  checkToFdist();

  // scan the field by turning around ~240 degrees, so turn one direction 120, and then turn other 240
  // if nothing found, change position and try again
  
  // if both sensors show above 1200, means nothing is within range
  // turn on spot for a number of clicks, but while detecting opponent
  if(!ToF_Left_detect && !ToF_Right_detect)turnOnSpot(TURNSWEEP1, ToF_last_detect, SPEEDQ);
  if(!ToF_Left_detect && !ToF_Right_detect)turnOnSpot(TURNSWEEP2, !ToF_last_detect, SPEEDQ);
    // if nothing is detected after these sweeps, we want to turn and move until line
  if(!ToF_Left_detect && !ToF_Right_detect){
    turnOnSpot(TURNAFTERSWEEP, ToF_last_detect, SPEEDHALF);
    
    // look for IR sensor detection
    driveStraight(FORWARD, SPEEDHALF);
    
    gameState = MOVEFROMEDGE;
    return;

  }

  
  else { // one of the ToFs picked something UP!
    // attack or evade!
    switch (gameMode)
    {
    case TASK1:
      gameState = ATTACK;
      break;
    
    case TASK2:
      gameState = EVADE;
      break;
    
    case GAME:
      int randNum = random(10);
      if(randNum<EVADECHANCE) gameState = EVADE;
      else gameState = ATTACK;
      break;
    }
    
  }



}

//___________________________
// drive towards the enemy in a straight line
// avoiding line should be treated with priority as well
//___________________________
void attack(){
  // indicate mode
  pwm.setPin(PLED_BLUE, OFF, true);
  pwm.setPin(PLED_YELLOW, OFF,true);
  pwm.setPin(PLED_GREEN, PLEDPWMMAX_GREEN,true);
  pwm.setPin(PLED_RED, OFF, true);

  // reset encoder 
  encL = 0;
  encR = 0;
  encL_prev = 0;
  encR_prev = 0;

  // track opponent by correcting for ToF readings
  while (ToF_Left_detect || ToF_Right_detect){
    if(ir_isAnyDetected){
      gameState = MOVEFROMEDGE; // move away from line
      return;
    }
    if(ToF_shellLifted) {
      MOTORSDISABLE;
      gameState = EVADE;
      return;
    }
    
    checkToFdist();
    // whichever ToF shows, we need to go that way
    // if both of them detect, then these expressions will account for that
    uint16_t lspeed = SPEEDHALF*ToF_Right_detect;
    uint16_t rspeed = SPEEDHALF*ToF_Left_detect;
    lMotorOn(FORWARD,lspeed);
    rMotorOn(FORWARD,rspeed);

    // stall protection
    if(is_motorStalled)return;

  }
  
  MOTORSDISABLE;
  gameState = SEEK; // back to seek

  
}
//___________________________
// react to IR sensors, generally retract from edge and turn around, exit into seek
//___________________________
void moveFromEdge(){
  // indicate mode
  pwm.setPin(PLED_BLUE, OFF, true);
  pwm.setPin(PLED_YELLOW, OFF,true);
  pwm.setPin(PLED_GREEN, OFF,true);
  pwm.setPin(PLED_RED, PLEDPWMMAX_RED, true);

  MOTORSDISABLE;

  // which sensors were triggered?
  // front
  if(irFront_left_state){
    // drive back for a few cm, then turn on spot
    driveStraight(BACKWARD,SPEEDHALF,MOVEFROMEDGEDIST);
    turnOnSpot(TURNIRFRONT, RIGHT, SPEEDHALF); // turn right
    
  }
  else if(irFront_right_state){
    // drive back for a few cm, then turn on spot
    driveStraight(BACKWARD,SPEEDHALF,MOVEFROMEDGEDIST);
    turnOnSpot(TURNIRFRONT, LEFT, SPEEDHALF); // turn left
  }
  else if(irBack_left_state){
    // drive forward a few cm, then turn on spot
    driveStraight(FORWARD,SPEEDHALF,MOVEFROMEDGEDIST);
    turnOnSpot(TURNIRBACK, RIGHT, SPEEDHALF); // turn right
  }
  else if(irBack_right_state){
    driveStraight(FORWARD,SPEEDHALF,MOVEFROMEDGEDIST);
    turnOnSpot(TURNIRBACK, LEFT, SPEEDHALF); // turn left
  }

  // check if any of the IRs are still active
  if(irBack_right_state||irBack_left_state||irFront_right_state||irFront_left_state) return;
  
  // if not, we can move on to the next state
  ir_isAnyDetected = false;
  
  // stall protection
  if(is_motorStalled)return;
  
  MOTORSDISABLE;

  // return to seek
  gameState = SEEK;



}

//___________________________
// goal of this function is to go to edge perpendicular to last enemy detection
//___________________________
void evade(){
  // indicate mode
  pwm.setPin(PLED_BLUE, OFF, true);
  pwm.setPin(PLED_YELLOW, PLEDPWMMAX_YELLOW,true);
  pwm.setPin(PLED_GREEN, OFF,true);
  pwm.setPin(PLED_RED, OFF, true);   

  // first I want to turn in a direction opposite of last known enemy position
  // 90 degrees
  turnOnSpot(TURNEVADE,ToF_last_detect, SPEEDHALF);
  

  // then drive straight to the line
  while(!ir_isAnyDetected){
    driveStraight(BACKWARD, SPEEDHALF);
  }
  // then back away from line a bit
  driveStraight(FORWARD, SPEEDHALF, MOVEFROMEDGEDIST);

  // reset ir detection
  if(irBack_right_state||irBack_left_state||irFront_right_state||irFront_left_state){
    gameState = MOVEFROMEDGE;
    return;
  } 
  ir_isAnyDetected = false;

  // then seek the target again to see what's what
  gameState = SEEK;

}


//___________________________
//___________________________
// main function entry, initialisation of the robot, selection of task/game mode
void setup() {

  _delay_ms(10);
  
  // set inputs, encoders, IR, button, Timer
  DDRB = 0;
  PORTB = 0;
  DDRC = 0;
  PORTC = 0;
  DDRD = 0;
  PORTD = 0;


  // interrupt prepare
  cli();

  // enable INT0 for timer
  EIMSK |= (1 << INT0); 
  EICRA |= (1 << ISC01); 

  // enable pin change interrupts, enable interrupts for encoder and IRs
  setBit(PCICR, PCIE0); 
  setBit(PCMSK0, PCINT0); 
  setBit(PCMSK0, PCINT1); 
  setBit(PCMSK0, PCINT2); 
  setBit(PCMSK0, PCINT3); 
  setBit(PCMSK0, PCINT4); 
  setBit(PCMSK0, PCINT5); 

  setBit(PCIFR, PCIF0);
  
  _delay_ms(10);
  sei();
  
  // I2C
  Wire.begin();
  _delay_ms(100);

  // pwm PCA9685 prepare
  pwm.begin();
  pwm.setPWMFreq(PTIMEFREQ);
      
  // disable all PWM
  pwm.setPin(PLED_BLUE, 0, true);
  pwm.setPin(PLED_YELLOW, 0,true);
  pwm.setPin(PLED_GREEN, 0,true);
  pwm.setPin(PLED_RED, 0, true);
  
  //__________________________
  // ToF prepare
  setBit(DDRC, TOFLEFT_XSHUT); // set as output to shutdown the ToF
  setBit(PORTC, TOFLEFT_XSHUT); 
  _delay_ms(10);
  clearBit(PORTC, TOFLEFT_XSHUT); // shut LEFT ToF down
  _delay_ms(10);

  ToF_Right.setAddress(TOFRIGHT_ADDR);
  _delay_ms(10);

  if(!ToF_Right.init())  
  {// debug method, if not initialised, it'll turn one light on, try to restart
    pwm.setPin(PLED_BLUE, PLEDPWMMAX_BLUE, true);
    while(1);
  }
  
  _delay_ms(10);

  ToF_Right.setTimeout(100);

  _delay_ms(10);
  
  setBit(PORTC, TOFLEFT_XSHUT);
  clearBit(DDRC, TOFLEFT_XSHUT);  // set as INPUT again, this becomes pulled up as well, 
                                  // the second sensor is now on
  _delay_ms(10);

  ToF_Left.setAddress(TOFLEFT_ADDR);

  _delay_ms(10);

  if(!ToF_Left.init()) 
  {// debug method, if not initialised, it'll turn one light on, try to restart
    pwm.setPin(PLED_YELLOW, PLEDPWMMAX_YELLOW, true);
    while(1);
  }

  _delay_ms(10);

  ToF_Left.setTimeout(100);
  
  _delay_ms(20);

  ToF_Right.startContinuous();
  ToF_Left.startContinuous();

  //__________________________


  _delay_ms(100);
 
  pwm.setPin(PTIMER, PHALFPULSE); // start the timer trigger pin
  _delay_ms(20);

  // Motor quick PID set up
  rMotorPID.SetOutputLimits(0,PMAXPULSE);
  lMotorPID.SetOutputLimits(0,PMAXPULSE);

  //__________________________
  // task/Competition selection routine
  while(timer_INT <= ROUNDSTARTTIME){
    
    // check for button presses, change game mode
    if(!(PINC & BV(BUTTON))){ // button is in pullup config, means it shows 1 when not pressed
      // reset the timer
      timer_INT = 0;
      //simple debounce, just wait
      _delay_ms(200);
      
      // increment the mode
      gameMode++;
      // reset mode to 0 if it's more than 2, 
      // don't let it go into initial state 3
      if(gameMode>2) gameMode = 0;

      // reset LEDs
      pwm.setPin(PLED_RED, 0, true);
      pwm.setPin(PLED_BLUE, 0, true);
      pwm.setPin(PLED_GREEN, 0, true);
      pwm.setPin(PLED_YELLOW, 0, true);
    }

    // indicate the current selected game mode
    switch(gameMode){
      case TASK1:
        pwm.setPin(PLED_YELLOW, PledBlinkState, true);
      break;
      case TASK2:
        pwm.setPin(PLED_GREEN, PledBlinkState, true);
      break;
      case GAME:
        pwm.setPin(PLED_BLUE, PledBlinkState, true);
      break;
      case INITSTART:
        pwm.setPin(PLED_RED, PledBlinkState, true);
      break;
    }
    PledBlinkState += (PledBlinkIncre - !PledBlinkIncre);
    if(PledBlinkState> PHALFPULSE) PledBlinkIncre = false;
    else if(PledBlinkState < 1) PledBlinkIncre = true;

    if(gameMode == INITSTART) timer_INT = 0;

  }
  timer_INT = 0;

  // WE ARE NOW READY FOR THIS

  // __________________________
  // debug OLED output
  // disp
  // display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
	// display.clearDisplay();
	// display.display();
  // display.setTextSize(3);
	// display.setTextColor(WHITE,BLACK);
}

//___________________________
// __________________________
// logic loop
void loop() {
  
  // stall protection
  if(is_motorStalled) { // returned from all the functions and now I can try to evade
    is_motorStalled = false; // reset current stall flag
    
    //count total amount of stalls and see if it's reached max
    if(++stallCounter>=STALLMAXCOUNT){
      timer_INT = 0;
      while(timer_INT<STALLRECOVERYTIME){ // turn off motors and do nothing to cool down
        MOTORSDISABLE;

        // indicate max number of stalls reached
        pwm.setPin(PLED_BLUE, OFF, true);
        pwm.setPin(PLED_YELLOW, PLEDPWMMAX_YELLOW,true);
        pwm.setPin(PLED_GREEN, OFF,true);
        pwm.setPin(PLED_RED, PLEDPWMMAX_RED, true);
      }
      stallCounter = 0;
      timer_INT = 0;
    }
  } 

  // switch statement for Tasks and States
  switch (gameMode)
  {
    case TASK1: // Task 1
      switch (gameState)
      {
        case SEEK:
          if(is_t1_firstRun) turnOnSpot(TURNAROUND,RIGHT,SPEEDHALF);
          is_t1_firstRun = false;
          seek();
          break;

        case ATTACK:
          attack();
          break;

        case MOVEFROMEDGE:
          moveFromEdge();
          break;
        
        default:
          evade(); // for stalling condition
          gameState = SEEK;
          break;
      }
    break;

    case TASK2: // task 2
      switch (gameState)
      {
        case SEEK:
          seek();
          break;

        case EVADE:
          evade();
          break;

        case MOVEFROMEDGE:
          moveFromEdge();
          break;
        
        default:
          gameState = SEEK;
          break;
      }
    break;

    case GAME: // Game
      if(is_gameFirstRun) {
        gameState = EVADE;
        is_gameFirstRun = false;
      }
      switch (gameState)
      {
        case SEEK:
          seek();
          break;
        
        case ATTACK:
          attack();
          break;

        case EVADE:
          evade();
          break;

        case MOVEFROMEDGE:
          moveFromEdge();
          break;
        
        default:
          gameState = SEEK;
          break;
      }
    break;
  }
  


  // __________________________
  // debug OLED output
  // display.setCursor(0,00);display.print(bluled);display.print("    ");
  // display.setCursor(0,30);display.print(grnled);display.print("    ");
  // display.display();
}




//___________________________
// __________________________
// interrupt routines

//Timer interrupt
ISR(INT0_vect) { 
  timer_INT+=1; // quite simply count time up
}

// Pin Change interrupts
// any of the pins' changes will trigger the entire interrupt
ISR(PCINT0_vect){
  // if this bit is not at its previous state, it has changed
  // 00, 01, 10, 11 -> 4 states, i know that they have to have changed to trigger interrupt
  // 0b000000_00 -> 
  // 0b000000_01
  // XOR will return only changed bit
  uint8_t pcint_current = PINB; // read the pin states in
  uint8_t pcint_changes = pcint_current ^ pcint_prevState; // figure out which pin just changed
  if(pcint_changes & BV(ENC_LEFT)) encL++; // compare all 6 pins 1 at a time
  if(pcint_changes & BV(ENC_RIGHT)) encR++;
  
  if(pcint_changes & BV(IRBACK_LEFT)) irBack_left_state = !(pcint_current & BV(IRBACK_LEFT));
  if(pcint_changes & BV(IRBACK_RIGHT)) irBack_right_state = !(pcint_current & BV(IRBACK_RIGHT));
  if(pcint_changes & BV(IRFRONT_LEFT)) irFront_left_state = !(pcint_current & BV(IRFRONT_LEFT));
  if(pcint_changes & BV(IRFRONT_RIGHT)) irFront_right_state = !(pcint_current & BV(IRFRONT_RIGHT));
  if(irBack_left_state || irBack_right_state || irFront_left_state || irFront_right_state) 
            ir_isAnyDetected = true;

  pcint_prevState = pcint_current;  
  //_delay_ms(100); // debug delay

}