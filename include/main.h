/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2024, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

/**
 * If defined, some commonly used enums will have preprocessor macros which give
 * a shorter, more convenient naming pattern. If this isn't desired, simply
 * comment the following line out.
 *
 * For instance, E_CONTROLLER_MASTER has a shorter name: CONTROLLER_MASTER.
 * E_CONTROLLER_MASTER is pedantically correct within the PROS styleguide, but
 * not convenient for most student programmers.
 */
#define PROS_USE_SIMPLE_NAMES

/**
 * If defined, C++ literals will be available for use. All literals are in the
 * pros::literals namespace.
 *
 * For instance, you can do `4_mtr = 50` to set motor 4's target velocity to 50
 */
#define PROS_USE_LITERALS

#include "api.h"
#include "CustomMotorGroup.hpp"
#include "driveTrain.hpp"
#include "autonomous/Tracking/odometrySensors.hpp"
#include "intakeMotors.hpp"
#include "functions.hpp"
#include "pneumatics.hpp"
#include "driveFunctions.hpp"
#include "autonomous/Control/encoderFunctions.hpp"
#include "autonomous/Tracking/Odom.hpp"
#include "autonomous/Control/linearPID.hpp"
#include <cmath>
#include "autonomous/Control/rotationalPID.hpp"
#include "autonomous/Tracking/ekf.hpp"
#include "autonomous/Tracking/quaternion.hpp"
#include <cstring>
#include "autonomous/Tracking/PurePursuit.hpp"
#include "autonomous/Tracking/Point.hpp"
#include <vector>
#include "autonomous/Tracking/FileLoader.hpp"
#include "autonomous/Tracking/particle_filter.hpp"
#include "autonomous/autons.hpp"
#include "autonomous/Control/delta_nim.hpp"
#include <cstdint>
#include <algorithm>
#include <numeric>
#include "autonomous/control/follow.hpp"

#define leftBackMotorPort -11
#define leftMiddleMotorPort -7
#define leftFrontMotorPort -6

#define rightBackMotorPort 4
#define rightMiddleMotorPort 3
#define rightFrontMotorPort 2

#define IMUPort 14
#define vertOdomPodPort 8
#define horizOdomPodPort 21

#define intake1MotorPort 9
#define intake2MotorPort -10

#define vertDistPort 17
#define horizDistPort 15

#define PISTON_A_PORT 'C'
#define PISTON_B_PORT 'B'
#define MATCH_LOADER_PISTON_PORT_1 'A'
#define MATCH_LOADER_PISTON_PORT_2 'D'
#define DESCORE_PISTON_PORT 'E' // change

#define distancePerTicks 0.02291
#define degreesPerTick 1.25
#define TRACK_WIDTH 13.5
#define wheelDiam 3.5

#define odomPodWheelDiam 2
#define tcksPerRev 360.0
#define distancePerTicksOdom ((M_PI * odomPodWheelDiam) / ticksPerRev)

#define kPL 1.68
#define kIL 0.0002
#define kDL 1.5

#define kPR 5.0
#define kIR 0.5
#define kDR 0.5
#define toleranceR 3.0

#define kPT 1.0
#define kIT 0
#define kDT 0.5

#define targetPitchTip 0.0
#define integralLimitTip 30.0
#define maxOutputTip 50.0

/**6
 * You should add more #includes here
 */
//#include "okapi/api.hpp"

/**
 * If you find doing pros::Motor() to be tedious and you'd prefer just to do
 * Motor, you can use the namespace with the following commented out line.
 *
 * IMPORTANT: Only the okapi or pros namespace may be used, not both
 * concurrently! The okapi namespace will export all symbols inside the pros
 * namespace.
 */
// using namespace pros;
// using namespace pros::literals;
// using namespace okapi;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/**
 * You can add C++-only headers here
 */
//#include <iostream>
#endif

#endif  // _PROS_MAIN_H_
