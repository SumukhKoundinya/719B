#include "main.h"

Autonomous auton;

void initialize() {
    auton.initFunc();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    auton.purePursuitTest();
}

void opcontrol() {
    arcadeControl();
    // auton.purePursuitTest(); 
}
