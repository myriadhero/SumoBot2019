/*
this is a prototype program for the sumobot 2019 competition

this will probably only include high level functions without specific implementation



*/

// includes



// I2C and other initiations

indicators indics;
motorControl Mcontrol;

// constants

#define ARENADIAM 1500
#define FRONT 1
#define BACK 0
#define ADVSEEK 1



// global variables
// toDo: volatile everything


volatile bool gameOn = 0;
volatile bool gameMode = 0; // simple mode for phase1 and phase 2
volatile bool phase = 0;

volatile bool borderNotDetected = 1; // toDo: think about border detection 
volatile bool borderFR = 0;         // behaviours; it's important
volatile bool borderFL = 0;         // since it's the losing criteria
volatile bool borderF = 0;
volatile bool borderBR = 0;
volatile bool borderBL = 0; 
volatile bool borderB = 0;

// prototype functions

void locate(bool frontBack); // find an arena edge from front or back
void hide(int hideKind);    // maneuvers that involve evasion

int seek(bool leftRight, bool advanced = 0){ // returns direction if found, otherwise 0
    int angle1 = 0;
    int angle2 = 0;
    //start sweeping the servo in the direction of turnDegrees
    // +1 degree, collect radar info
    // if radar info == P2, collect first angle
    // when radar info != P2, collect second angle
    // sweep back to confirm, correct angles if needed
    return ((angle1+angle2)/2);
}
bool isP2coming(); // returns 1 for yes, quick check whether p2 is approaching fast
void attack();
void celebrate();
// void borderISR();
// void encodersISR();

class motorControl{
    public:

    int initParams = 0;
    unsigned int encRight = 0;
    unsigned int encLeft = 0;


    void setMotor(bool leftRight, int motorPWM);

    void driveStraight(bool forwardBackward);
    void turnDegrees(bool leftRight, int degrees);
    void driveArc(bool leftRight, int severity); // abandon if hard to implement
    void evadeAboutPerim(bool leftRight);


};

class indicators{
    public:

    void initInd(){
        // when the robot is ready to go
    }

    void gameStart(){

    }

    void aboutTown(){

    }

    void seek(){

    }

    void attack(){

    }

    void celebrate(){

    }


};


int main() {

    // initialise
    



    indics.initInd(); //ready to go


    while(!gameOn){
        // wait for start command
    }
    
    while(gameOn){

        locate(FRONT); // drive forward and activate both sensors
        hide(1);
        if(seek()){
            if(isP2coming()){
                // 75% chance to initiate evade, 25% chance to initiate attack
            }
            else{
                // 10% chance to initiate evade, 90% chance to initiate attack
            }
        }
        
        else{
            if(seek(ADVSEEK)){
                if(isP2coming()){
                    // 75% chance to initiate evade, 25% chance to initiate attack
                }
                else{
                    // 10% chance to initiate evade, 90% chance to initiate attack
                }
            }
            else{
                celebrate(); // celebrate and set loop values to exit
                break;
            }
        }


        
    } // close tag for while(gameOn)
    
    //end of the game routine


}