/*
this is a prototype program for the sumobot 2019 competition

this will probably only include high level functions without specific implementation



*/

// prototype functions


void locate(bool frontBack);
void hide(){

}

void seek();
void evade();
void attack();

class motorControl{
    public:

    int initParams = 0;


    void setMotor(bool leftRight, int motorPWM);

    void driveStraight(bool forwardBackward);
    void turnDegrees(bool leftRight, int degrees);
    void driveArc(bool leftRight, int severity); // abandon if hard to implement
    void evadeAboutPerim(bool leftRight);


};


int main() {
    



}