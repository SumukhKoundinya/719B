#include "main.h"

int moveDistanceEncoders(double inches) {
    double distance_in_ticks = abs(inches) / distancePerTicks;

    double velocity;
    double threshold = 0.5;
    double speed = 120;

    int direction = (inches >= 0) ? 1 : -1;

    while (true) {
        double left_pos = left_mg.getMotorTrackerVal();
        double right_pos = right_mg.getMotorTrackerVal();

        double left_remaining = distance_in_ticks - left_pos * direction;
        double right_remaining = distance_in_ticks - right_pos * direction;

        if (left_remaining <= 0 && right_remaining <= 0) {
            left_mg.moveMotorsVelocity(0);
            right_mg.moveMotorsVelocity(0);
            break;
        }

        if (left_remaining <= threshold * distance_in_ticks)
            velocity = direction * speed * (left_remaining / (threshold * distance_in_ticks));
        else
            velocity = direction * speed;

        left_mg.moveMotorsVelocity(velocity);
        right_mg.moveMotorsVelocity(velocity);

        std::string alpha_text = "Motor pos: " + std::to_string(right_mg.getMotorTrackerVal()) + ";" + std::to_string(left_mg.getMotorTrackerVal());
        pros::lcd::print(1, "%s", alpha_text);

        pros::delay(20);
    }

    return 0;
}

int turnEncoders(double degrees) {

    double angle_in_ticks = abs(degrees) * degreesPerTick;

    left_mg.resetMotorTracker();
    right_mg.resetMotorTracker();

    int direction = (degrees >= 0) ? 1 : -1;

    left_mg.moveMotors(-direction * 50);
    right_mg.moveMotors(direction * 50);

    while (true) {
        double left_pos = abs(left_mg.getMotorTrackerVal());
        double right_pos = abs(right_mg.getMotorTrackerVal());

        pros::delay(50); 

        if (left_pos >= angle_in_ticks && right_pos >= angle_in_ticks) {
            break;
        }
    }

    left_mg.moveMotors(0);
    right_mg.moveMotors(0);

    return 0;
}


void moveDegrees(int degrees) {
    double wheelTravel = (TRACK_WIDTH * M_PI * degrees) / 360;
    double motorDegrees = (wheelTravel * 360) / (M_PI * wheelDiam);

    left_mg.resetMotorTracker();
    right_mg.resetMotorTracker(); 


    left_mg.moveMotorDegrees(-motorDegrees, 100);
    right_mg.moveMotorDegrees(motorDegrees, 100);

    while (fabs(left_mg.getMotorTrackerVal()) < fabs(motorDegrees)) {  
        pros::delay(10);
    }

    left_mg.moveMotorsVelocity(0);
    right_mg.moveMotorsVelocity(0);
}
