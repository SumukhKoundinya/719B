#include "main.h"

pros::Imu imu(IMUPort);
pros::Rotation vertPod(vertOdomPodPort);
pros::Rotation horizPod(horizOdomPodPort);
pros::Distance vertFrontDist(vertFrontDistPort);
pros::Distance horizRightDist(horizRightDistPort);
pros::Distance horizLeftDist(horizLeftDistPort);
pros::Distance vertBackDist(vertBackDistPort);
