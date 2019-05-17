/* SER300 Sumo Robot
Kirill Duplyakin  217182546

Trimester 1, 2019
____________________

I/O

Microcontroller has these external inputs:
- 4 IR line sensors, 2 on the front and 2 on the back, 
- 2 Time of Flight sensors on the front
- 2 Encoders on the motors
- 1 Button, in pullup config (when not pressed, reads 1)

- Additionally it controls a PWM extender module via I2C that returns back timing signal on interrupt INT0

10 Outputs are controlled through PWM extender module, namely:
- 6 motor channels, 3 left and 3 right
- 4 LEDs

To be able to initialise the two ToFs, one of them needs to be put in reset state, 
while the other ToF is given a non-default address 
- 1 digital output pin is used for that

_____________________

Programming approach used is state machine programming. 

- Task 1: seek and attack, avoid lines - 3 states
- Task 2: seek, evade following the line carefully, and avoid arena edge - 3 states
- Competition: seek, avoid lines, attack, evade - 4 states

-> First a simple selection is implemented at the end of setup function,
      to let the user pick the Task/Competition
      - the button can be pressed to select a different mode
      - once the button is pressed, the countdown starts for the game
      - if button is pressed again during the countdown, 
          the countdown is reset and next mode is selected

-> Task 1
  - The bot starts this task in seek mode, it has to then turn around 180
  - once seek mode identifies potential enemy, attack mode is activated
  - both modes can be interrupted by detecting arena edge, in this case, bot goes back into seek after 
      avoiding the line


-> Task 2
  - The bot starts this task in seek mode
  - once seek mode identifies potential enemy, evade mode is activated
    - evade mode incorporates it's own logic for following the line, so only
    - seek mode can be interrupted by detecting arena edge, after avoiding line goes back to seek mode


-> Competition
  - The bot starts this task in seek mode
  - once seek mode identifies potential enemy, there's a different probability for attack and evade modes
    - it may be a good strategy to evade on initial startup  
  - seek and attack can be interrupted by detecting arena edge, after avoiding line goes back to seek mode


all 4 states/bahaviours share commonality between tasks/game



*/


#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>

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


//TOF vars
VL53L0X ToF_Left;
VL53L0X ToF_Right;


// PCA9685 board var, called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


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

// Encoder defines, motor related
#define ENC_LEFT PB0
#define ENC_RIGHT PB1
#define FORWARD true
#define BACKWARD false
#define RIGHT true
#define LEFT false

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

// program specific
#define PTIMEFREQ 1000 // desired timer freq, Hz
#define TSAMP_ENC 100 // how often the encoder value should be sampled, ms
#define ROUNDSTARTTIME 5000 // 5000 ms after a mode is selected to start the round
#define PMAXPULSE 4096 // maximum PCA9685 PWM value, full ON
#define PHALFPULSE 2048 // this is half width for PWM, max is 4096

#define TURNCONST 36 // helper constant to convert degrees to encoder clicks for turning on spot


#define TOFMAXRANGE 1200 // ToF sensor max meaningful reading

//___________________________
// variables

// timer variables
volatile uint32_t timer_INT = 0; // timer as per interrupt
volatile uint32_t timer_prev = 0;

// motor encoders & speed control
volatile uint16_t encR = 0; // if these are connected in opposite, simply switch them in ISR
volatile uint16_t encL = 0;
volatile uint16_t encL_prev = 0;
volatile uint16_t encR_prev = 0;



// motor encoder (& IR sensor interrupts)
volatile uint8_t pcint_prevState = 0;   // byte of states
volatile bool irFront_left_state = false; // false is not triggered
volatile bool irFront_right_state = false;
volatile bool irBack_left_state = false;
volatile bool irBack_right_state = false;
volatile bool is_lineDetected = false;

// ToF helpers
volatile uint16_t ToF_Left_distmm = 0; // toDo: see if these variables are needed
volatile uint16_t ToF_Right_distmm = 0;
volatile bool ToF_Left_detect = false; // false means sensor is showing > TOFMAXRANGE
volatile bool ToF_Right_detect = false; // false means sensor is showing > TOFMAXRANGE

//__________________________
/* Game mode select
task 1 = 0
task 2 = 1
game = 2
initial statup = 3
*/
volatile uint8_t gameMode = 3; 
volatile uint8_t PledBlinkState = 0; // used to create a cool blinking animation
volatile bool PledBlinkIncre = 1; // true is increasing

/* Game States
  These are states in game that determine what the robot is doing now
  default is seek = 0
  attack = 1
  evade = 2
  moveFromLine = 3
*/
volatile uint8_t gameState = 0;

// Task 1 specific:
// turn 180 on first run of task1
volatile bool is_t1_firstRun = true;

// Competition mode specific:
volatile bool is_gameFirstRun = true;




//___________________________
// functions

// this simply collects distance readings from ToF sensors
void checkToFdist(){
  if(!ToF_Right.timeoutOccurred()) ToF_Right_distmm = ToF_Right.readRangeContinuousMillimeters();
  if(!ToF_Left.timeoutOccurred()) ToF_Left_distmm = ToF_Left.readRangeContinuousMillimeters();
  if(ToF_Left_distmm < TOFMAXRANGE) ToF_Left_detect = true;
  else ToF_Left_detect = false;
  if(ToF_Right_distmm < TOFMAXRANGE) ToF_Right_detect = true;
  else ToF_Right_detect = false;
}

// motor control functions
void rMotorOn(bool forward = true, uint16_t pwmvalue = 0, bool disable = false){
  if (disable){
    pwm.setPin(PMOTOR_R1, 0);
    pwm.setPin(PMOTOR_R2, 0);
    pwm.setPin(PMOTOR_Rpwm, 0);
    return;
  }
  if(forward){
    pwm.setPin(PMOTOR_R1, PMAXPULSE);
    pwm.setPin(PMOTOR_R2, 0);
  }
  else{
    pwm.setPin(PMOTOR_R1, 0);
    pwm.setPin(PMOTOR_R2, PMAXPULSE);
  }
  pwm.setPin(PMOTOR_Rpwm, pwmvalue);
}
void lMotorOn(bool forward = true, uint16_t pwmvalue = 0, bool disable = false){
  if (disable){
    pwm.setPin(PMOTOR_L1, 0);
    pwm.setPin(PMOTOR_L2, 0);
    pwm.setPin(PMOTOR_Lpwm, 0);
    return;
  }
  if(forward){
    pwm.setPin(PMOTOR_L1, PMAXPULSE);
    pwm.setPin(PMOTOR_L2, 0);
  }
  else{
    pwm.setPin(PMOTOR_L1, 0);
    pwm.setPin(PMOTOR_L2, PMAXPULSE);
  }
  pwm.setPin(PMOTOR_Lpwm, pwmvalue);
}
void turnOnSpot(uint8_t degX10,bool turnRight = true, uint16_t turnSpeed = 1000, bool is_seeking = false){
  
  // ~650 clicks per rev, i need to turn the wheels 
  uint16_t clicksToTurn = degX10*TURNCONST;

  // reset encoder and time values 
  encL = 0;
  encR = 0;
  // encL_prev = 0;
  // encR_prev = 0;
  // timer_INT = 0;
  // timer_prev = 0;

  
  
  if(turnRight){ // left motor forward, right motor back
    while((encL+encR) / 2 < clicksToTurn){
      // toDo: implement PID later
      // avoid arena edge
      if(is_lineDetected){
        gameState = 3;
        return;
      }

      if (encL < clicksToTurn) lMotorOn(turnRight,turnSpeed);
      else lMotorOn(0,0,1);
      
      if(encR < clicksToTurn) rMotorOn(!turnRight,turnSpeed);
      else rMotorOn(0,0,1);

      if(is_seeking){
        checkToFdist();
        if (ToF_Left_detect || ToF_Right_detect) break;
      }
    }

  } 
  else{ // right motor forward, left motor back
    while((encL+encR) / 2 < clicksToTurn){

      // avoid arena edge
      if(is_lineDetected){
        gameState = 3;
        return;
      }

      if (encL < clicksToTurn) lMotorOn(turnRight,turnSpeed);
      else lMotorOn(0,0,1);
      
      if(encR < clicksToTurn) rMotorOn(!turnRight,turnSpeed);
      else rMotorOn(0,0,1);

      if(is_seeking){
        checkToFdist();
        if (ToF_Left_detect || ToF_Right_detect) break;
      }
    }
  }
  // turn motors off again, just in case
  lMotorOn(0,0,1);
  rMotorOn(0,0,1);
}
// void driveStraight(bool forward = true, uint16_t distCm){
//   //650 clicks per 22 cm

//   // toDo: implement PID
  



// }


// purpose of this function is to identify potential enemy and go to attack or evade
// avoiding line should be treated with priority as well
void seek(){
  // indicate seek mode:
  pwm.setPin(PLED_BLUE, PLEDPWMMAX_BLUE, true);
  pwm.setPin(PLED_YELLOW, 0,true);
  pwm.setPin(PLED_GREEN, 0,true);
  pwm.setPin(PLED_RED, 0, true);

  // scan the field by turning around ~240 degrees, so turn one direction 120, and then turn other 240
  // if nothing found, change position and try again
  
  // if both sensors show above 1200, means nothing is within range
  // turn on spot for a number of clicks, but while detecting opponent
  if(!ToF_Left_detect && !ToF_Right_detect)turnOnSpot(12, RIGHT, 500, 1);
  if(!ToF_Left_detect && !ToF_Right_detect)turnOnSpot(24, LEFT, 500, 1);
    // if nothing is detected after these sweeps, we want to turn and move until line
  if(!ToF_Left_detect && !ToF_Right_detect){
    turnOnSpot(6, RIGHT, 1000);
    // look for IR sensor detection
    while(!is_lineDetected){ 
      rMotorOn(FORWARD, 1000); // same direction
      lMotorOn(FORWARD, 1000);
    }
    gameState = 3;
    return;

  }

  
  else { // one of the ToFs picked something UP!
    // attack or evade!
    switch (gameMode)
    {
    case 0:
      gameState = 1;
      break;
    
    case 1:
      gameState = 2;
      break;
    
    case 2:
      uint8_t randNum = random(10);
      if(randNum<2) gameState = 2;
      else gameState = 1;
      break;
    
    default:
      break;
    }
    
  }



}

// drive towards the enemy in a straight line
// avoiding line should be treated with priority as well
void attack(){
  // indicate mode
  pwm.setPin(PLED_BLUE, 0, true);
  pwm.setPin(PLED_YELLOW, 0,true);
  pwm.setPin(PLED_GREEN, PLEDPWMMAX_GREEN,true);
  pwm.setPin(PLED_RED, 0, true);
  
}

// react to IR sensors, generally 
void moveFromLine(){
  // indicate mode
  pwm.setPin(PLED_BLUE, 0, true);
  pwm.setPin(PLED_YELLOW, 0,true);
  pwm.setPin(PLED_GREEN, 0,true);
  pwm.setPin(PLED_RED, PLEDPWMMAX_RED, true);

  lMotorOn(0,0,1);
  rMotorOn(0,0,1);

  // which sensors were triggered?
  // front
  if(irFront_left_state || irFront_left_state){
    // drive back for a few cm, then turn on spot
  }
  else if(irBack_left_state || irBack_right_state){
    // drive forward a few cm, then turn on spot
  }
  


}

// goal of this function is to go to edge, drive along it enough to try to evade the enemy
void evade(){
  // indicate mode
  pwm.setPin(PLED_BLUE, 0, true);
  pwm.setPin(PLED_YELLOW, PLEDPWMMAX_YELLOW,true);
  pwm.setPin(PLED_GREEN, 0,true);
  pwm.setPin(PLED_RED, 0, true);   

}

//____________________
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


  //__________________________
  // task/Competition selection routine
  while(timer_INT - timer_prev <= ROUNDSTARTTIME){
    
    // check for button presses, change game mode
    if(!(PINC & BV(BUTTON))){ // button is in pullup config, means it shows 1 when not pressed
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
      
      // reset the timer
      timer_INT = 0;
    }

    // indicate the current selected game mode
    switch(gameMode){
      case 0:
        pwm.setPin(PLED_YELLOW, PledBlinkState, true);
      break;
      case 1:
        pwm.setPin(PLED_GREEN, PledBlinkState, true);
      break;
      case 2:
        pwm.setPin(PLED_BLUE, PledBlinkState, true);
      break;
      case 3:
        pwm.setPin(PLED_RED, PledBlinkState, true);
      break;
    }
    PledBlinkState += (PledBlinkIncre - !PledBlinkIncre);
    if(PledBlinkState> 4095) PledBlinkIncre = false;
    else if(PledBlinkState < 1) PledBlinkIncre = true;

    if(gameMode == 3) timer_INT = 0;

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


// __________________________
// logic loop
void loop() {
  
  // update ToF measurements, i want to grab only good values 
  checkToFdist();
  // when distance is further than 1.1-1.2 meters, it outputs ~8980, 
  // otherwise seems pretty reliable up to distances of 1m
  // so i need to close the gap of 20-40cm by meandering

  //also can get current motor speed here?

  // 
  switch (gameMode)
  {
    case 0: // Task 1
      switch (gameState)
      {
        case 0:
          if(is_t1_firstRun) turnOnSpot(18);
          is_t1_firstRun = false;
          seek();
          break;

        case 1:
          attack();
          break;

        case 3:
          moveFromLine();
          break;
        
        default:
          gameState = 0;
          break;
      }
    break;

    case 1: // task 2
      switch (gameState)
      {
        case 0:
          seek();
          break;

        case 2:
          evade();
          break;

        case 3:
          moveFromLine();
          break;
        
        default:
          gameState = 0;
          break;
      }
    break;

    case 2: // Game
      if(is_gameFirstRun) {
        gameState = 2;
        is_gameFirstRun = false;
      }
      switch (gameState)
      {
        case 0:
          seek();
          break;
        
        case 1:
          attack();
          break;

        case 2:
          evade();
          break;

        case 3:
          moveFromLine();
          break;
        
        default:
          gameState = 0;
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
            is_lineDetected = true;

  pcint_prevState = pcint_current;  
  //_delay_ms(100); // debug delay

}