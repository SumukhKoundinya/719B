#include "main.h"

Autonomous auton;

void initialize() {
    auton.initFunc();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    //auton.leftSideAutonControl();
    auton.selfEmbodimentOfPerfection();
    //auton.test();
    //auton.purePursuitTest();
    //auton.AditdaGoat();
}

void opcontrol() {
    arcadeControl();
    
    //auton.purePursuitTest(); 
}
