#include "main.h"

CustomMotorGroup::CustomMotorGroup(std::vector<pros::Motor> motor_list) : motors(motor_list){
  for(pros::Motor motor : motor_list){
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
}

void CustomMotorGroup::moveMotors(int voltage) {
  for(pros::Motor motor : this->motors){
    motor.move(voltage);
  }
}

void CustomMotorGroup::moveMotorsVelocity(int velocity) {
  for(pros::Motor motor : this->motors){
    motor.move_velocity(velocity);
  }
}

void CustomMotorGroup::moveMotorsPrecise(int milliVoltage) {
  for(pros::Motor motor : this->motors){
    motor.move_voltage(milliVoltage);
  }
}

void CustomMotorGroup::moveMotorDegrees(int degrees, int velocity) {
  for(pros::Motor motor : this->motors){
    motor.move_relative(degrees, velocity);
  }
}

void CustomMotorGroup::setBrakeMode(pros::motor_brake_mode_e brakeMode) {
  for (pros::Motor motor : this->motors) {
    motor.set_brake_mode(brakeMode);
  }
}

void CustomMotorGroup::resetMotorTracker() {
  for (pros::Motor motor : this->motors) {
    motor.tare_position();
  }
}
double CustomMotorGroup::getMotorTrackerVal() {
  double avgPosition = 0;

  for(pros::Motor motor : this->motors) {
    avgPosition = avgPosition + motor.get_position();
  }

  return avgPosition / this->motors.size();
}

double CustomMotorGroup::getMotorRPM(){
  double avgSpeed = 0;

  for (pros::Motor motor : this->motors) {
    avgSpeed = avgSpeed + motor.get_actual_velocity();
  }

  return avgSpeed / 3;
}

double CustomMotorGroup::getMotorTemp() {
  double avgTemp = 0;

  for (pros::Motor motor : this->motors) {
    avgTemp = avgTemp + motor.get_temperature();
  }

  return avgTemp / this->motors.size();
}
