/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      7/27/2026, 6:11:07 PM                                     */
/*    Description:  DR4B V5 project - driver control override                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
/*
  Ports:
    11 - VEXnet/V5 Radio (no code needed)
    12 - Back Left drive motor
    13 - Back Right drive motor
    14 - Claw motor
    15 - Front Left drive motor
    16 - Front Right drive motor
    17 - Left Lift (DR4B arm) motor
    18 - Right Lift (DR4B arm) motor

  Controls:
    Left Stick  Y (Axis3) - Drive forward / backward
    Right Stick X (Axis1) - Turn left / right
    L1  - Close claw
    L2  - Open claw
    R1  - Raise arm
    R2  - Lower arm
*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// ---------------------------------------------------------------------------
// define your global instances of motors and other devices here
// ---------------------------------------------------------------------------
brain      Brain;
controller Controller1 = controller(primary);

// Drivetrain motors -- right side motors are physically mirrored, so reverse them
motor BackLeft   (PORT12, ratio18_1, false);
motor BackRight  (PORT13, ratio18_1, true);
motor FrontLeft  (PORT15, ratio18_1, false);
motor FrontRight (PORT16, ratio18_1, true);

// DR4B arm lift motors (left/right sides move together)
motor LeftLift  (PORT17, ratio36_1, false);
motor RightLift (PORT18, ratio36_1, true);

// Claw motor
motor Claw(PORT14, ratio18_1, false);

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------
const int DEADBAND       = 5;    // joystick deadband, percent
const int DRIVE_SCALE    = 100;  // max drive percent

const int ARM_BASE_SPEED = 55;   // percent speed target for the arm, up AND down
const int ARM_GRAVITY_FF = 18;   // percent voltage added/removed to fight/assist gravity

const int CLAW_SPEED     = 60;   // percent speed for opening/closing claw

// ---------------------------------------------------------------------------
// Helper: apply a simple deadband to a joystick axis
// ---------------------------------------------------------------------------
int applyDeadband(int value) {
  if (value > -DEADBAND && value < DEADBAND) return 0;
  return value;
}

// ---------------------------------------------------------------------------
// Drivetrain: split-arcade, coast stopping so the bot doesn't jerk/topple
// cups or pins when the sticks are released or reversed quickly
// ---------------------------------------------------------------------------
void driveControl() {
  int throttle = applyDeadband(Controller1.Axis3.position());  // left stick Y
  int turn     = applyDeadband(Controller1.Axis1.position());  // right stick X

  int leftPower  = throttle + turn;
  int rightPower = throttle - turn;

  // Clamp to +/-100
  if (leftPower  >  DRIVE_SCALE) leftPower  =  DRIVE_SCALE;
  if (leftPower  < -DRIVE_SCALE) leftPower  = -DRIVE_SCALE;
  if (rightPower >  DRIVE_SCALE) rightPower =  DRIVE_SCALE;
  if (rightPower < -DRIVE_SCALE) rightPower = -DRIVE_SCALE;

  if (leftPower == 0 && rightPower == 0) {
    // Coast = drivetrain rolls freely instead of snapping to a stop,
    // preventing knocked-over pins/cups when the driver lets go of the stick
    FrontLeft.stop(coast);
    BackLeft.stop(coast);
    FrontRight.stop(coast);
    BackRight.stop(coast);
  } else {
    FrontLeft.spin (forward, leftPower,  percent);
    BackLeft.spin  (forward, leftPower,  percent);
    FrontRight.spin(forward, rightPower, percent);
    BackRight.spin (forward, rightPower, percent);
  }
}

// ---------------------------------------------------------------------------
// DR4B Arm: gravity-compensated so up-speed == down-speed
//
//   Going UP:   gravity opposes the motors -> add feedforward power
//   Going DOWN: gravity assists the motors -> subtract feedforward power
//   Idle:       hold brake so the arm doesn't sag/drop under its own weight
// ---------------------------------------------------------------------------
void armControl() {
  bool raise = Controller1.ButtonR1.pressing();
  bool lower = Controller1.ButtonR2.pressing();

  if (raise && !lower) {
    int upSpeed = ARM_BASE_SPEED + ARM_GRAVITY_FF;
    LeftLift.spin (forward, upSpeed, percent);
    RightLift.spin(forward, upSpeed, percent);
  }
  else if (lower && !raise) {
    int downSpeed = ARM_BASE_SPEED - ARM_GRAVITY_FF;
    if (downSpeed < 0) downSpeed = 0;   // safety floor
    LeftLift.spin (reverse, downSpeed, percent);
    RightLift.spin(reverse, downSpeed, percent);
  }
  else {
    // Hold position -- prevents the arm from falling under gravity
    LeftLift.stop(hold);
    RightLift.stop(hold);
  }
}

// ---------------------------------------------------------------------------
// Claw: hold brake so grip is maintained on gripped objects
// ---------------------------------------------------------------------------
void clawControl() {
  bool openBtn  = Controller1.ButtonL2.pressing();
  bool closeBtn = Controller1.ButtonL1.pressing();

  if (closeBtn && !openBtn) {
    Claw.spin(forward, CLAW_SPEED, percent);
  }
  else if (openBtn && !closeBtn) {
    Claw.spin(reverse, CLAW_SPEED, percent);
  }
  else {
    Claw.stop(hold);
  }
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Set default stopping modes
  LeftLift.setStopping(hold);
  RightLift.setStopping(hold);
  Claw.setStopping(hold);
  FrontLeft.setStopping(coast);
  FrontRight.setStopping(coast);
  BackLeft.setStopping(coast);
  BackRight.setStopping(coast);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    driveControl();
    armControl();
    clawControl();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}