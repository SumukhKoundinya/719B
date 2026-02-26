#pragma once

#include "main.h"

class CustomMotorGroup {
  private : 
    std::vector<pros::Motor> motors;
  public :    
    CustomMotorGroup(std::vector<pros::Motor> motor_list);

    void moveMotors(int voltage);
    void moveMotorsVelocity(int velocity);
    void moveMotorsPrecise(int milliVoltage);
    void moveMotorDegrees(int degrees, int velocity);
    void setBrakeMode(pros::motor_brake_mode_e brakeMode);
    void resetMotorTracker();
    double getMotorTrackerVal();
    double getMotorRPM();
    double getMotorTemp();
};
