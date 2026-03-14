#include "main.h"

double integralT = 0.0;
double lastError = 0.0;

double antiTip(double pitch) {
    double error = targetPitchTip - pitch;

    integralT += error;
    if (integralT > integralLimitTip) integralT = integralLimitTip;
    else if (integralT < -integralLimitTip) integralT = -integralLimitTip;

    double derivative = error - lastError;
    lastError = error;

    double output = (kPT * error) + (kIT * integralT) + (kDT * derivative);

    if (output > maxOutputTip) output = maxOutputTip;
    else if (output < -maxOutputTip) output = -maxOutputTip;

    return output;
}

void arcadeControl() {
    while (true) {
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        left_mg.moveMotors(dir - turn);
        right_mg.moveMotors(dir + turn);

        mainOpControl();
        toggleDescore();
        pros::delay(20);
    }
}

void arcadeTipControl() {
    while (true) {
        int dir = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        double pitch = imu.get_pitch();
        double corr = antiTip(pitch);

        dir -= corr;

        int leftSpeed  = dir - turn;
        int rightSpeed = dir + turn;

        leftSpeed  = std::clamp(leftSpeed, -127, 127);
        rightSpeed = std::clamp(rightSpeed, -127, 127);

        left_mg.moveMotors(leftSpeed);
        right_mg.moveMotors(rightSpeed);

        mainOpControl();
        toggleDescore();
        pros::delay(20);
    }
}

void tankDriveControl() {
    while (true) {
        int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        left_mg.moveMotors(left);
        right_mg.moveMotors(right);

        master.print(0, 1, "Left Heat: %d, Right Heat: %d", left_mg.getMotorTemp(), right_mg.getMotorTemp());

        mainOpControl();
        toggleDescore();
        pros::delay(20);
    }
}  
