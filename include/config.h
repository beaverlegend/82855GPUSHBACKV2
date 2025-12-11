#include "pros/misc.hpp"
#pragma once
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "device/mockimu.h"


#define DRIVE_GEARSET pros::E_MOTOR_GEARSET_06
#define WHEEL_DIAMETER 3.25
#define DRIVE_RPM 450

#define LEFT_FRONT_DRIVE -2
#define LEFT_MIDDLE_DRIVE -4
#define LEFT_BACK_DRIVE -3

#define RIGHT_FRONT_DRIVE 7
#define RIGHT_MIDDLE_DRIVE 8
#define RIGHT_BACK_DRIVE 9

#define VERTICAL_ODOM -10
#define IMU 16


#define IntakeTopRoller -1
#define IntakeLastWheel 5

#define TONGUE 'A' // change this
#define WINGS 'B'
#define INTAKE 'H'

#define DIST_FRONT 11
#define DIST_BACK 12
#define DIST_LEFT 13
#define DIST_RIGHT 14

inline MockIMU imu(IMU, 359.3 / 360.0);
inline pros::MotorGroup left_mg({LEFT_FRONT_DRIVE, LEFT_MIDDLE_DRIVE, LEFT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup right_mg({RIGHT_FRONT_DRIVE, RIGHT_MIDDLE_DRIVE, RIGHT_BACK_DRIVE}, pros::MotorGearset::blue);
inline pros::MotorGroup Intake_High_mg({IntakeTopRoller, IntakeLastWheel});
inline pros::MotorGroup Intake_Index_mg({IntakeTopRoller, -IntakeLastWheel});
inline pros::MotorGroup Intake_Bottom({-IntakeTopRoller});
inline pros::MotorGroup Intake_Top({-IntakeLastWheel});
inline pros::Rotation vertical_odom(VERTICAL_ODOM);

// pneumatics

inline pros::adi::Pneumatics tongue(TONGUE, false);
inline pros::adi::Pneumatics wings(WINGS, false);
inline pros::adi::Pneumatics intakeFinal(INTAKE, false);

// distance sensors
inline pros::Distance dist_front(DIST_FRONT);
inline pros::Distance dist_back(DIST_BACK);
inline pros::Distance dist_left(DIST_LEFT);
inline pros::Distance dist_right(DIST_RIGHT);

