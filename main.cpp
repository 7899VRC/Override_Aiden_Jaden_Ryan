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
    Left Stick  Y (Axis3) - Drive forward / backward (curved response)
    Right Stick X (Axis1) - Turn left / right (curved response)
    L1  - Close claw
    L2  - Open claw
    R1  - Hold: raise arm.  Double-tap: jump to MAX preset height
    R2  - Hold: lower arm.  Double-tap: jump to MIN preset height
    X   - Hold for TURBO drive (max speed)
    B   - Hold for TURTLE drive (precision/slow speed)
    A   - Press to TOGGLE panic stop (cuts + coasts all motors until pressed again)
    Y   - Press to TOGGLE reverse-camera drive (swaps forward/back)

  NOTE ON ARM PRESETS: these use each V5 motor's built-in relative encoder,
  NOT an external sensor. That means the arm MUST start from the same known
  physical position (e.g. resting on its hard stop) every time the robot is
  powered on / the encoder is reset in pre_auton(), or the preset heights
  will be wrong. Measure your own MAX/MIN degree values on the real robot
  and update ARM_MAX_PRESET_DEG / ARM_MIN_PRESET_DEG below.
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
const int    DEADBAND            = 5;     // joystick deadband, percent

const int    CRUISE_SCALE        = 70;    // normal driving max speed, percent
const int    TURBO_SCALE         = 100;   // X held - max speed, percent
const int    TURTLE_SCALE        = 35;    // B held - precision speed, percent

const int    SLEW_RATE           = 8;     // max percent change in drive power PER LOOP (20ms)

const double DRIVE_CURVE_STRENGTH = 0.75; // 0 = fully linear, 1 = fully cubic.
                                           // Blend gives fine control near center,
                                           // full speed still reachable at full stick.

const int    ARM_BASE_SPEED      = 55;    // percent speed target for the arm, up AND down
const int    ARM_GRAVITY_FF      = 18;    // percent voltage added/removed to fight/assist gravity
const int    ARM_PRESET_SPEED    = 70;    // percent speed used for double-tap preset moves

const double ARM_MAX_PRESET_DEG  = 850;   // TUNE ME: encoder degrees at full extension
const double ARM_MIN_PRESET_DEG  = 10;    // TUNE ME: encoder degrees near the bottom (with clearance)

const int    TAP_MAX_DURATION_MS = 250;   // press shorter than this counts as a "tap"
const int    DOUBLE_TAP_WINDOW_MS = 400;  // 2nd tap must land within this of the 1st tap's release

const int    CLAW_SPEED          = 60;    // percent speed for opening/closing claw
const double CLAW_AUTO_OPEN_AMPS = 1.5;   // current spike (amps) that suggests the claw hit something
const double ARM_LOW_HEIGHT_DEG  = 60;    // only auto-open near the bottom of arm travel,
                                           // so squeezing an object hard mid-air doesn't trigger it

const double MOTOR_WARN_TEMP_C   = 50.0;  // celsius - warn driver before thermal shutdown

const int    DASHBOARD_PERIOD_LOOPS = 10; // update controller screen every N loops (~200ms)

// ---------------------------------------------------------------------------
// Persistent state (must survive across loop iterations)
// ---------------------------------------------------------------------------
int    currentLeftPower   = 0;      // last commanded left-side power (for slew limiting)
int    currentRightPower  = 0;      // last commanded right-side power (for slew limiting)

bool   panicActive         = false; // true = all motors cut/coasting
bool   prevPanicBtnState   = false; // previous loop's A-button state, for edge detection

bool   reverseDriveMode    = false; // true = forward/back swapped
bool   prevReverseBtnState = false; // previous loop's Y-button state, for edge detection

bool   armAutoActive       = false; // true = arm is auto-driving to a preset via spinToPosition

bool   r1WasPressed        = false;
double r1PressStartMs      = 0;
double r1LastTapEndMs      = -100000;

bool   r2WasPressed        = false;
double r2PressStartMs      = 0;
double r2LastTapEndMs      = -100000;

bool   clawAutoOpenLatched = false; // prevents auto-open from re-triggering every loop while held

int    loopCounter         = 0;     // for throttling the dashboard print

// ---------------------------------------------------------------------------
// Helper: apply a simple deadband to a joystick axis
// ---------------------------------------------------------------------------
int applyDeadband(int value) {
  if (value > -DEADBAND && value < DEADBAND) return 0;
  return value;
}

// ---------------------------------------------------------------------------
// Helper: blend linear + cubic response so small stick movements give fine
// control near center, while full deflection still reaches full speed
// ---------------------------------------------------------------------------
int applyDriveCurve(int raw) {
  double x = raw / 100.0;
  double curved = DRIVE_CURVE_STRENGTH * (x * x * x) + (1.0 - DRIVE_CURVE_STRENGTH) * x;
  return (int)(curved * 100.0);
}

// ---------------------------------------------------------------------------
// Helper: limit how fast a value can change per loop, to smooth acceleration
// ---------------------------------------------------------------------------
int slewLimit(int current, int target, int maxStep) {
  int delta = target - current;
  if (delta >  maxStep) delta =  maxStep;
  if (delta < -maxStep) delta = -maxStep;
  return current + delta;
}

// ---------------------------------------------------------------------------
// Helper: pick the active drive speed cap based on Turbo/Turtle buttons
// ---------------------------------------------------------------------------
int currentDriveScale() {
  if (Controller1.ButtonX.pressing()) return TURBO_SCALE;
  if (Controller1.ButtonB.pressing()) return TURTLE_SCALE;
  return CRUISE_SCALE;
}

// ---------------------------------------------------------------------------
// Reverse-camera toggle: Y flips which end of the robot is "forward" for
// driving purposes, without the driver needing to mentally invert the stick
// ---------------------------------------------------------------------------
void reverseModeCheck() {
  bool pressed = Controller1.ButtonY.pressing();
  if (pressed && !prevReverseBtnState) {
    reverseDriveMode = !reverseDriveMode;
    Controller1.rumble(".");
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(reverseDriveMode ? "Reverse drive: ON" : "Reverse drive: OFF");
  }
  prevReverseBtnState = pressed;
}

// ---------------------------------------------------------------------------
// Drivetrain: split-arcade with drive curve, Turbo/Turtle speed modes,
// slew-rate limited acceleration, reverse-camera mode, and coast stopping
// so the bot doesn't jerk/topple cups or pins.
// ---------------------------------------------------------------------------
void driveControl() {
  int scale = currentDriveScale();

  int rawThrottle = applyDeadband(Controller1.Axis3.position());  // left stick Y
  int rawTurn      = applyDeadband(Controller1.Axis1.position());  // right stick X

  int throttle = applyDriveCurve(rawThrottle);
  int turn     = applyDriveCurve(rawTurn);

  if (reverseDriveMode) throttle = -throttle;

  throttle = throttle * scale / 100;
  turn     = turn     * scale / 100;

  int targetLeft  = throttle + turn;
  int targetRight = throttle - turn;

  if (targetLeft   >  scale) targetLeft   =  scale;
  if (targetLeft   < -scale) targetLeft   = -scale;
  if (targetRight  >  scale) targetRight  =  scale;
  if (targetRight  < -scale) targetRight  = -scale;

  // Slew-rate limit: ramp toward the target instead of snapping to it.
  // Turbo mode skips the ramp on the way UP (needs to feel immediate),
  // but still ramps DOWN so releasing the stick at full turbo doesn't jerk.
  int upStep = (scale == TURBO_SCALE) ? scale : SLEW_RATE;
  currentLeftPower  = slewLimit(currentLeftPower,  targetLeft,  upStep);
  currentRightPower = slewLimit(currentRightPower, targetRight, upStep);

  if (currentLeftPower == 0 && currentRightPower == 0 && targetLeft == 0 && targetRight == 0) {
    FrontLeft.stop(coast);
    BackLeft.stop(coast);
    FrontRight.stop(coast);
    BackRight.stop(coast);
  } else {
    FrontLeft.spin (forward, currentLeftPower,  percent);
    BackLeft.spin  (forward, currentLeftPower,  percent);
    FrontRight.spin(forward, currentRightPower, percent);
    BackRight.spin (forward, currentRightPower, percent);
  }
}

// ---------------------------------------------------------------------------
// Helper: detects a double-tap on a button. Call every loop with the
// button's current pressed state and its tracking variables (passed by
// reference). Returns true exactly once, on the loop the 2nd tap releases.
// ---------------------------------------------------------------------------
bool detectDoubleTap(bool pressedNow, bool &wasPressed, double &pressStartMs, double &lastTapEndMs) {
  double nowMs = Brain.Timer.time(msec);
  bool doubleTap = false;

  if (pressedNow && !wasPressed) {
    // rising edge - press just started
    pressStartMs = nowMs;
  }
  else if (!pressedNow && wasPressed) {
    // falling edge - press just ended; was it short enough to be a "tap"?
    double pressDuration = nowMs - pressStartMs;
    if (pressDuration <= TAP_MAX_DURATION_MS) {
      if ((nowMs - lastTapEndMs) <= DOUBLE_TAP_WINDOW_MS) {
        doubleTap = true;
        lastTapEndMs = -100000; // reset so 3 quick taps don't chain into extra triggers
      } else {
        lastTapEndMs = nowMs;
      }
    }
  }
  wasPressed = pressedNow;
  return doubleTap;
}

// ---------------------------------------------------------------------------
// DR4B Arm: gravity-compensated so up-speed == down-speed.
//   Hold R1/R2        -> manual proportional control (cancels any preset move)
//   Double-tap R1/R2  -> auto-drive to MAX/MIN preset height
//   Idle               -> hold brake so the arm doesn't sag under its own weight
// ---------------------------------------------------------------------------
void armControl() {
  bool raise = Controller1.ButtonR1.pressing();
  bool lower = Controller1.ButtonR2.pressing();

  bool raiseDoubleTap = detectDoubleTap(raise, r1WasPressed, r1PressStartMs, r1LastTapEndMs);
  bool lowerDoubleTap = detectDoubleTap(lower, r2WasPressed, r2PressStartMs, r2LastTapEndMs);

  if (raiseDoubleTap) {
    armAutoActive = true;
    LeftLift.spinToPosition (ARM_MAX_PRESET_DEG, rotationUnits::deg, ARM_PRESET_SPEED, velocityUnits::pct, false);
    RightLift.spinToPosition(ARM_MAX_PRESET_DEG, rotationUnits::deg, ARM_PRESET_SPEED, velocityUnits::pct, false);
    Controller1.rumble(".");
  }
  if (lowerDoubleTap) {
    armAutoActive = true;
    LeftLift.spinToPosition (ARM_MIN_PRESET_DEG, rotationUnits::deg, ARM_PRESET_SPEED, velocityUnits::pct, false);
    RightLift.spinToPosition(ARM_MIN_PRESET_DEG, rotationUnits::deg, ARM_PRESET_SPEED, velocityUnits::pct, false);
    Controller1.rumble(".");
  }

  // Any real hold (not the tap itself) cancels an in-progress preset move
  // and returns control to the driver.
  if ((raise || lower) && !raiseDoubleTap && !lowerDoubleTap) {
    armAutoActive = false;
  }

  if (armAutoActive) {
    return; // let spinToPosition drive (and then hold) the arm on its own
  }

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
// Claw: hold brake so grip is maintained. Auto-opens if the claw is closed
// and near the bottom of the arm's travel and its motor current spikes
// (a rough "it just hit something" heuristic). This is a heuristic, not a
// real sensor -- a limit switch or distance sensor in the claw would be
// more reliable if you have the ports free for one.
// ---------------------------------------------------------------------------
void clawControl() {
  bool openBtn  = Controller1.ButtonL2.pressing();
  bool closeBtn = Controller1.ButtonL1.pressing();

  if (closeBtn && !openBtn) {
    Claw.spin(forward, CLAW_SPEED, percent);
    clawAutoOpenLatched = false; // driver is actively closing, re-arm auto-open
  }
  else if (openBtn && !closeBtn) {
    Claw.spin(reverse, CLAW_SPEED, percent);
    clawAutoOpenLatched = false;
  }
  else {
    // Idle / holding a grip -- check for the auto-open condition
    bool armIsLow    = LeftLift.position(rotationUnits::deg) < ARM_LOW_HEIGHT_DEG;
    bool currentSpike = Claw.current(currentUnits::amp) > CLAW_AUTO_OPEN_AMPS;

    if (armIsLow && currentSpike && !clawAutoOpenLatched) {
      Claw.spin(reverse, CLAW_SPEED, percent);
      clawAutoOpenLatched = true;   // don't spam-retrigger every loop
      Controller1.rumble(".");
    } else if (!currentSpike) {
      Claw.stop(hold);
    }
  }
}

// ---------------------------------------------------------------------------
// Panic button: A is a TOGGLE (not hold). Press once -> everything cuts and
// coasts and stays that way. Press again -> control resumes normally.
// ---------------------------------------------------------------------------
bool panicCheck() {
  bool pressed = Controller1.ButtonA.pressing();

  if (pressed && !prevPanicBtnState) {
    panicActive = !panicActive;

    if (panicActive) {
      Controller1.rumble("-");
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("** PANIC STOP **");
    } else {
      Controller1.rumble(".");
      Controller1.Screen.clearLine(1);
      currentLeftPower  = 0;
      currentRightPower = 0;
    }
  }
  prevPanicBtnState = pressed;

  if (panicActive) {
    FrontLeft.stop(coast);
    BackLeft.stop(coast);
    FrontRight.stop(coast);
    BackRight.stop(coast);
    LeftLift.stop(coast);
    RightLift.stop(coast);
    Claw.stop(coast);
  }

  return panicActive;
}

// ---------------------------------------------------------------------------
// Dashboard: shows brain + controller battery and warns about hot motors.
// Throttled so it doesn't spam the controller screen every 20ms.
// ---------------------------------------------------------------------------
void updateDashboard() {
  if (panicActive) return; // don't overwrite the panic message on line 1

  loopCounter++;
  if (loopCounter % DASHBOARD_PERIOD_LOOPS != 0) return;

  // NOTE: the V5 controller class does not expose its own battery level
  // through this API -- only the robot (brain) battery is readable.
  int brainBattery = Brain.Battery.capacity(percent);

  Controller1.Screen.clearLine(2);
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Robot Batt: %d%%", brainBattery);

  // Find the hottest motor
  double maxTemp = 0;
  const char* hottestName = "";

  struct NamedMotor { motor* m; const char* name; };
  NamedMotor motors[] = {
    {&FrontLeft,  "FL"}, {&FrontRight, "FR"},
    {&BackLeft,   "BL"}, {&BackRight,  "BR"},
    {&LeftLift,   "LLift"}, {&RightLift, "RLift"},
    {&Claw,       "Claw"}
  };

  for (auto &nm : motors) {
    double t = nm.m->temperature(celsius);
    if (t > maxTemp) {
      maxTemp = t;
      hottestName = nm.name;
    }
  }

  Controller1.Screen.clearLine(3);
  Controller1.Screen.setCursor(3, 1);
  if (maxTemp >= MOTOR_WARN_TEMP_C) {
    Controller1.Screen.print("HOT: %s %.0fC", hottestName, maxTemp);
    Controller1.rumble("-");
  } else {
    Controller1.Screen.print("Temp OK (%.0fC max)", maxTemp);
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

  // Zero the arm's encoders here ONLY IF the arm is physically resting at
  // its known bottom hard-stop at power-on -- required for the MAX/MIN
  // preset heights (double-tap R1/R2) to be accurate.
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
    reverseModeCheck();

    // Panic check runs first every loop. If active, it takes over all
    // motors and we skip normal control entirely for this iteration.
    if (!panicCheck()) {
      driveControl();
      armControl();
      clawControl();
    }

    updateDashboard();

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
