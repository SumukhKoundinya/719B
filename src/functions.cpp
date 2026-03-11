#include "main.h"

void moveIntake(int speed) {
    intake1.move_voltage(speed * 12000 / 127);
    intake2.move_voltage(speed * 12000 / 127);
}

void intakeFunction() {
    if (master.get_digital(DIGITAL_R1)) {
        moveIntake(127);
    } else if (master.get_digital(DIGITAL_R2)) {
        moveIntake(-127);
    } else {
        moveIntake(0);
    }
}

void ppR1() {
    if (master.get_digital(DIGITAL_R1)) {
        pistonA.set_value(1);
        moveIntake(127);
    }
}

void ppR2() {
    if (master.get_digital(DIGITAL_R2)) {
        moveIntake(-127);
    }
}

void ppL1() {
    if (master.get_digital(DIGITAL_L1)) {
        pistonA.set_value(0);
        pistonB.set_value(1);
        moveIntake(127);
    }
}

void ppL2() {
    if (master.get_digital(DIGITAL_L2)) {
        pistonA.set_value(1);
        pistonB.set_value(1);
        moveIntake(127);
    }
}

void ppUp() {
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        matchLoader1.set_value(1);
        matchLoader2.set_value(1);
    }
}

void mainOpControl() {
    
    int intakeSpeed = 0;

    // ---- INTAKE ----
    if (master.get_digital(DIGITAL_R2)) intakeSpeed = -127;
    if (master.get_digital(DIGITAL_L1)) intakeSpeed = 127;
    if (master.get_digital(DIGITAL_R1)) intakeSpeed = 127;
    if (master.get_digital(DIGITAL_L2)) intakeSpeed = 127;

    moveIntake(intakeSpeed);

    // ---- PNEUMATICS ----
    pistonA.set_value(master.get_digital(DIGITAL_L2));
    pistonB.set_value(master.get_digital(DIGITAL_R1) || master.get_digital(DIGITAL_L2));

    // ---- MATCH LOADER ----
    static bool button_press_matchLoaderPiston = false;
    static bool piston_extend_matchLoaderPiston = false;

    // Match Loader
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        if (!button_press_matchLoaderPiston) {
            piston_extend_matchLoaderPiston = !piston_extend_matchLoaderPiston;
            matchLoader1.set_value(piston_extend_matchLoaderPiston);
            matchLoader2.set_value(piston_extend_matchLoaderPiston);
            button_press_matchLoaderPiston= true;
        }
    } else {
        button_press_matchLoaderPiston= false;
    }
}

void toggleMatchLoader() {
    static bool button_press_matchL = false;
    static bool piston_extend_matchL = false;

    static bool button_press_pistonA = false;
    static bool piston_extend_pistonA = false;

    static bool button_press_pistonB = false;
    static bool piston_extend_pistonB = false;

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        if (!button_press_matchL) {
            piston_extend_matchL = !piston_extend_matchL;
            matchLoader1.set_value(piston_extend_matchL);
            matchLoader2.set_value(piston_extend_matchL);
            button_press_matchL = true;
        }
    } else {
        button_press_matchL = false;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        moveIntake(127);
        if (!button_press_pistonA) {
            piston_extend_pistonA = !piston_extend_pistonA;
            pistonA.set_value(piston_extend_pistonA);
            button_press_pistonA = true;
        }
    } else {
        button_press_pistonA = false;
    }

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (!button_press_pistonB) {
            piston_extend_pistonB = !piston_extend_pistonB;
            pistonB.set_value(piston_extend_pistonB);
            button_press_pistonB = true;
        }
    } else {
        button_press_pistonB = false;
    }
}

void toggleDescore() {
    static bool button_press_descore = false;
    static bool piston_extend_descore = false;

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        if (!button_press_descore) {
            piston_extend_descore = !piston_extend_descore;
            descore.set_value(piston_extend_descore);
            button_press_descore = true;
        }
    } else {
        button_press_descore = false;
    }
}
