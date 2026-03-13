#include "main.h"

Autonomous auton;

void initialize() {
    auton.initFunc();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    //auton.rightSideAutonControl();
    auton.rightSideAutonControl();
    //auton.test();
<<<<<<< HEAD
    //auton.purePursuitTest();
=======
    auton.AditdaGoat();
>>>>>>> e7d5a43924a3f0f2aaeb5298de8bf3a7706a012c
}

void opcontrol() {
    arcadeControl();
    
    //auton.purePursuitTest(); 
}
