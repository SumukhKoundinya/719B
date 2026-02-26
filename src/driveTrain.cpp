#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Motor rightBackMotor(rightBackMotorPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);
pros::Motor rightMiddleMotor(rightMiddleMotorPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);
pros::Motor rightFrontMotor(rightFrontMotorPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);

pros::Motor leftBackMotor(leftBackMotorPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);
pros::Motor leftMiddleMotor(leftMiddleMotorPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);
pros::Motor leftFrontMotor(leftFrontMotorPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);

CustomMotorGroup left_mg = CustomMotorGroup({
    rightBackMotor,
    rightMiddleMotor,
    rightFrontMotor
});

CustomMotorGroup right_mg = CustomMotorGroup({
    leftBackMotor,
    leftMiddleMotor,
    leftFrontMotor
});
