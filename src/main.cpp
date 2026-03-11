#include "main.h"

Autonomous auton;

void initialize() {
    auton.initFunc();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    //auton.rightSideAutonControl();
    //auton.rightSideAutonControl();
    //auton.test();
    auton.test();
}

void opcontrol() {
    arcadeControl();
    
    //auton.purePursuitTest(); 
}
