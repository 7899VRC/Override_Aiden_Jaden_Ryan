/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      7/27/2026, 6:11:07 PM                                     */
/*    Description:  DR4B V5 project - driver control (basic, no extras)      */
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
    R1  - Raise arm (hold)
    R2  - Lower arm (hold)
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
const int DEADBAND         = 5;    // joystick deadband, percent
const int DRIVE_SPEED      = 70;   // drive speed cap, percent

const int ARM_BASE_SPEED   = 55;   // percent speed target for the arm, up AND down
const int ARM_GRAVITY_FF   = 18;   // percent voltage added/removed to fight/assist gravity

const int CLAW_SPEED       = 60;   // percent speed for opening/closing claw

// ---------------------------------------------------------------------------
// Helper: apply a simple deadband to a joystick axis
// ---------------------------------------------------------------------------
int applyDeadband(int value) {
  if (value > -DEADBAND && value < DEADBAND) return 0;
  return value;
}

// ---------------------------------------------------------------------------
// Drivetrain: basic split-arcade control.
// ---------------------------------------------------------------------------
void driveControl() {
  int throttle = applyDeadband(Controller1.Axis3.position());  // left stick Y
  int turn     = applyDeadband(Controller1.Axis1.position());  // right stick X

  throttle = throttle * DRIVE_SPEED / 100;
  turn     = turn     * DRIVE_SPEED / 100;

  int leftPower  = throttle + turn;
  int rightPower = throttle - turn;

  if (leftPower  >  DRIVE_SPEED) leftPower  =  DRIVE_SPEED;
  if (leftPower  < -DRIVE_SPEED) leftPower  = -DRIVE_SPEED;
  if (rightPower >  DRIVE_SPEED) rightPower =  DRIVE_SPEED;
  if (rightPower < -DRIVE_SPEED) rightPower = -DRIVE_SPEED;

  if (leftPower == 0 && rightPower == 0) {
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
// DR4B Arm: gravity-compensated manual control (hold R1 to raise, R2 to lower).
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
    if (downSpeed < 0) downSpeed = 0;
    LeftLift.spin (reverse, downSpeed, percent);
    RightLift.spin(reverse, downSpeed, percent);
  }
  else {
    LeftLift.stop(hold);
    RightLift.stop(hold);
  }
}

// ---------------------------------------------------------------------------
// Claw: manual open/close, hold brake otherwise.
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

  // Zero the arm's encoders here -- only accurate if the arm is physically
  // resting at a known position at power-on.
  LeftLift.resetPosition();
  RightLift.resetPosition();
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
  while (1) {
    driveControl();
    armControl();
    clawControl();

    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
