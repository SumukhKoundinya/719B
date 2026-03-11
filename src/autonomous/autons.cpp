#include "main.h"

Odometry odom;
LinearPID drivePID(kPL, kIL, kDL, odomPodWheelDiam, tcksPerRev);
RotationalPID rotPID(kPR, kIR, kDR, toleranceR, -127, 127);
ParticleFilter pf(100);

std::vector<PathPoint> path;
PurePursuit controller({}, 20.0, 13.75);

double prevX = 0;
double prevY = 0;
double prevTheta = 0;

#define standardAmplification 1.25

void pfTask(void*) {
    while (true) {
        odom.update();

        pf.update(
            odom.getDeltaS(),
            odom.getDeltaTheta(),
            odom.getFrontDist(),
            odom.getBackDist(),
            odom.getLeftDist(),
            odom.getRightDist()
        );

        pros::delay(20);
    }
}

void Autonomous::initFunc() {
    pros::lcd::initialize();

    left_mg.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    right_mg.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    imu.reset();

    master.clear();

    pros::lcd::print(0, "Robot Activated!");

    pros::delay(200);
    while (imu.is_calibrating()) {
        pros::lcd::set_text(1, "IMU is Calibrating : DO NOT TOUCH");
        pros::delay(100);

        pros::lcd::set_text(1, "IMU is Done Calibrating");
    }
    pistonB.set_value(1);

    odom.init();
    pf.init(0, 0, 0);
    odom.reset(0, 0, 0);

    path = loadPath(nullptr);
    controller = PurePursuit(path, 6.0, 13.5);

    pros::Task pf_task(pfTask);
    
    /*odom.initializePose();

    pros::Task odom_task(odomTask);*/
}

double getTargetOrientation(double targetX, double targetY){
    double currX = pf.getY();
    double currY = pf.getX();

    double deltaX = targetX - currX;
    double deltaY = targetY - currY;

    double absoluteAngle = atan2(deltaX, deltaY) * 180.0 / M_PI;
    
    double currentHeading = imu.get_rotation();
    double relativeAngle = absoluteAngle - currentHeading;

    // normalize to shortest path
    while (relativeAngle > 180) relativeAngle -= 360;
    while (relativeAngle < -180) relativeAngle += 360;

    return relativeAngle;
}

double getTargetDistance(double xInches, double yInches) {
    double deltaX = xInches - pf.getY();
    double deltaY = yInches - pf.getX();

    return sqrt(pow(deltaX, 2) + pow(deltaY, 2));
}

static void bezierPoint(double p0x, double p0y,
                        double p1x, double p1y,
                        double p2x, double p2y,
                        double p3x, double p3y,
                        double t, double& outX, double& outY) {
    double mt = 1 - t;
    outX = mt*mt*mt*p0x + 3*mt*mt*t*p1x + 3*mt*t*t*p2x + t*t*t*p3x;
    outY = mt*mt*mt*p0y + 3*mt*mt*t*p1y + 3*mt*t*t*p2y + t*t*t*p3y;
}

void moveToPointCurve(double targetX, double targetY, double maxSpeed = 100.0) {
    double startX = pf.getY();
    double startY = pf.getX();

    double headingRad = imu.get_rotation() * M_PI / 180.0;

    double dist = hypot(targetX - startX, targetY - startY);
    if (dist < 1.0) return;

    double tension = dist / 3.0;

    double cp1x = startX + tension * sin(headingRad);
    double cp1y = startY + tension * cos(headingRad);

    double angleToTarget = atan2(targetX - startX, targetY - startY);
    double cp2x = targetX - tension * sin(angleToTarget);
    double cp2y = targetY - tension * cos(angleToTarget);

    const int NUM_POINTS = 50;  // increased from 20
    std::vector<PathPoint> curvePath;

    for (int i = 0; i <= NUM_POINTS; i++) {
        double t = (double)i / NUM_POINTS;
        double px, py;
        bezierPoint(startX, startY, cp1x, cp1y, cp2x, cp2y, targetX, targetY, t, px, py);
        bool isLast = (i == NUM_POINTS);
        double speed = isLast ? 0.0 : sin(t * M_PI) * maxSpeed;
        if (!isLast && speed < 25.0) speed = 25.0;
        curvePath.push_back({px, py, speed, 0.0, isLast});
    }

    PurePursuit pp(curvePath, 20.0, 13.75);  // increased lookahead from 6 to 10
    while (!pp.isFinished()) {
        Pose currentPose = {
            pf.getY(),
            pf.getX(),
            imu.get_rotation() * M_PI / 180.0
        };
        pp.step(currentPose);
        pros::delay(20);
    }
}

void moveToPointCurveOrientation(double targetX, double targetY, double finalHeading) {
    moveToPointCurve(targetX, targetY);
    rotPID.rotateTo(finalHeading);
}

void moveToPoint(double targetX, double targetY) {
    /*for(int i = 0; i <= 6; i++) pros::lcd::clear_line(i);
    double targetTheta = getTargetOrientation(xInches, yInches);
    double targetDistance = getTargetDistance(xInches, yInches);

    pros::lcd::set_text(5, "target theta " +  std::to_string(targetTheta));
    pros::lcd::set_text(6, "target distance " +  std::to_string(targetDistance));
    pros::lcd::set_text(7, "final position " +  std::to_string(xInches) + " , " + std::to_string(yInches) );

    if(targetDistance < 1) return;

    rotPID.rotateTo(targetTheta);
    drivePID.moveDistance(targetDistance, 1.0);*/

    for (int i = 0; i<= 6; i++) pros::lcd::clear_line(i);
    
    double targetDistance = getTargetDistance(targetX, targetY);
    double targetTheta = getTargetOrientation(targetX, targetY);

    pros::lcd::set_text(5, "Target Theta: " + std::to_string(targetTheta));
    pros::lcd::set_text(6, "Target Distance: " + std::to_string(targetDistance));
    pros::lcd::set_text(7, "Final Pos: " + std::to_string(targetX) + ", " + std::to_string(targetY));

    if (targetDistance < 1.0) return;

    rotPID.rotateTo(targetTheta);
    drivePID.moveDistance(targetDistance, 1);
}

void moveToPointOrientation(int xInches, int yInches, double finalOrientation) {
    moveToPoint(xInches, yInches);
    rotPID.rotateTo(finalOrientation);
}

void intakeOnceTask(void*) { // idle death gamble
    // change delay here
    pros::delay(500);
    moveIntake(-127);
    pros::delay(100);
    moveIntake(127);
}
void intakeTwiceTask(void*) { // idle death gamble
    // change delay here
    pros::delay(400);
    pistonB.set_value(0);
    moveIntake(127);
}

void smartAuton() {
    uint32_t start_time = pros::millis();
    
    while (pros::millis() - start_time < 15000) {  // 15s auton
        double x, y, theta = pf.getPose();  // YOUR ODOM FUNCTION
        
        // SIMULATE ENEMY BLOCK (change per trial)
        bool loader_blue_blocked = true;  // Set TRUE for test
        double ex = 95, ey = 15;           // Loader Blue position
        
        if (pros::millis() - start_time < DeltaNim::PRESET_TIME) {
            // FIRST 3s: YOUR PRESET PATHS
            if (x < -10) moveToPoint(12, 30);  // Loader Red
            else moveToPoint(-24, 12);          // Loader Blue (might be blocked)
        } else {
            // 3s+: Δ-NIM SMART RECOVERY
            auto heaps = DeltaNim::fieldToHeaps(x, y, theta, ex, ey, loader_blue_blocked);
            auto [target_goal, remove_amount] = DeltaNim::optimalMove(heaps);
            
            if (target_goal >= 0) {
                auto [gx, gy] = DeltaNim::getGoalPosition(target_goal);
                moveToPoint(gx, gy);  // YOUR PID
                pros::lcd::set_text(0, "DNIM: Goal " + std::to_string(target_goal));
            }
        }
        pros::delay(5);  // 200Hz
    }
    moveToPoint(0, 0);  // Return to safe position
}

void Autonomous::purePursuitTest() {
    /*controller.reset();
    while (!controller.isFinished()) {
        Pose pose = {pf.getX(), pf.getY(), odom.getHeadingRad()};
        controller.step(pose);
        pros::delay(20);
    }
    left_mg.moveMotors(0);
    right_mg.moveMotors(0);*/
    /*moveToPoint(5, 24);
    rotPID.rotateTo(60);
    moveToPoint(0, 10);*/
    //drivePID.moveDistance(15, 0.9);

    //moveToPointCurve(15, 24);
    /*while(true) {
        master.print(0, 0, "X: %.2f", pf.getY());
        master.print(1, 0, "Y: %.2f", pf.getX());
        master.print(2, 0, "T: %.2f", imu.get_rotation());
        pros::delay(20);
    }*/
    /*moveIntake(127);
    drivePID.moveDistance(24, 0.85);
    rotPID.rotateTo(90);
    drivePID.moveDistance(12, 1);*/
    // smartAuton();
}

void Autonomous::test() {
    //drivePID.moveDistance(24, 1);
    moveIntake(127);
    drivePID.moveDistance(15, standardAmplification);
    rotPID.rotateTo(14);
    drivePID.moveDistance(15, standardAmplification);
    pros::delay(800);
    rotPID.rotateTo(-57);
    drivePID.moveDistance(13.5, standardAmplification);
    rotPID.rotateTo(-8);
    moveIntake(-127);
    pros::delay(1200);
    moveIntake(0);
    drivePID.moveDistance(-39, 1);
    rotPID.rotateTo(-132);
    drivePID.moveDistance(-20, 1);
}

void Autonomous::timeCellMoonPalace() { // ADIT EDIT THIS
    //drivePID.moveDistance(12, 0.9);pkjio0---------------------
    //drivePID.moveDistance(8, 0.9); 
    moveIntake(127);
    drivePID.moveDistance(15, 1);
    rotPID.rotateTo(14);
    drivePID.moveDistance(19, 0.72);
    pros::delay(500);
    rotPID.rotateTo(105);
    moveIntake(0);
    drivePID.moveDistance(20.5, 0.87);
    rotPID.rotateTo(53);
    drivePID.moveDistance(-9, 1);
    pistonB.set_value(0);
    moveIntake(-127);
    pros::delay(150);
    moveIntake(127);

    /*pros::delay(2500);
    descore.set_value(1);
    drivePID.moveDistance(14, 1);
    rotPID.rotateTo(135);
    drivePID.moveDistance(25, 1);
    descore.set_value(0);*/
}

void moveMotorsTogether(int speed) {
    left_mg.moveMotors(speed);
    right_mg.moveMotors(speed);
}

void Autonomous::selfEmbodimentOfPerfection() {
    /*descore.set_value(1);
    //drivePID.moveDistance(12, 0.9);
    //drivePID.moveDistance(8, 0.9); 
    moveIntake(127);
    drivePID.moveDistance(15, 1);
    rotPID.rotateTo(15);
    drivePID.moveDistance(19, 0.8);
    pros::delay(500);
    rotPID.rotateTo(105);
    moveIntake(0);
    drivePID.moveDistance(23, 1);
    rotPID.rotateTo(52.5);
    matchLoader2.set_value(1);
    drivePID.moveDistance(-6, 1);
    pistonB.set_value(0);
    moveIntake(127);
    pros::delay(1500);
    pistonB.set_value(1);
    drivePID.moveDistance(25, 1);*/
    pistonB.set_value(0);
    moveIntake(127);
    moveMotorsTogether(-127);
    pros::delay(250);
    moveMotorsTogether(127);
    pros::delay(2500);
    moveMotorsTogether(0);
}   

void Autonomous::testGamble() {
    imu.tare_rotation();

    //pistonA.set_value(1);
    //pros::delay(100);
    pros::delay(150);
    moveIntake(127);
    rotPID.rotateTo(16);
    drivePID.moveDistance(16, 0.9);
    drivePID.moveDistance(10, 0.9);

    rotPID.rotateTo(105);
    moveIntake(0);
    drivePID.moveDistance(32, 0.8);
    pros::delay(500);
    pistonB.set_value(0);
    rotPID.rotateTo(49);

    // test
    pros::Task intakeTask(intakeOnceTask);
    drivePID.moveDistance(-18.5, 1);
    pros::delay(750);
    moveIntake(0);

    // test phase 3
    drivePID.moveDistance(10, 0.9);
    rotPID.rotateTo(125);
    drivePID.moveDistance(16, 0.95);
    rotPID.rotateTo(56);
    descore.set_value(1);
    drivePID.moveDistance(3.5, 1);
    descore.set_value(0);
    drivePID.moveDistance(15.5, 1);
}

void Autonomous::leftSideAutonControl() {
    
}

void Autonomous::rightSideAutonControl() {
    //drivePID.moveDistance(12, 0.9);
    //drivePID.moveDistance(8, 0.9); 
    moveIntake(127);
    drivePID.moveDistance(15, 1);
    rotPID.rotateTo(14);
    drivePID.moveDistance(19, 0.72);
    pros::delay(500);
    rotPID.rotateTo(105);
    moveIntake(0);
    drivePID.moveDistance(20.5, 0.87);
    rotPID.rotateTo(53);
    drivePID.moveDistance(-9, 1);
    pistonB.set_value(0);
    moveIntake(-127);
    pros::delay(150);
    moveIntake(127);

    pros::delay(2500);
    descore.set_value(1);
    drivePID.moveDistance(14, 1);
    rotPID.rotateTo(135);
    drivePID.moveDistance(25, 1);
    descore.set_value(0);

}

