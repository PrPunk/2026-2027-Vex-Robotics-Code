/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Kobe Lebron and Netenyahu */
/*    Created:      Sat 23 Aug 2025                                           */
/*    Description:  Team 986B bot for 2025-2026                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "brain-display.cpp"
#include <algorithm>
#include <cmath>
using namespace vex;
BrainDisplay brainDisplay; 

// A global instance of competition
competition Competition;

vex::timer accelTimer;

const int accelStep = 10;
const int accelSec = 300;
const int armdeg = 200;
const double WRISTDEG = 180*5;   // change this to how far the wrist should rotate (degrees)

float dSpeed = 1;
// 48 teeth red
// 36 teeth green
// 10.22 inch circumference
bool slowDrive = false;
double leftVeloc = 0;
double rightVeloc = 0;
int autonMode = 1;
bool intakeOn = false;
bool outakeOn = false;
// float degPerInch = 25; // new robot 158.5127201; // old robot 47.012
float inchPerDeg = 0.09; // new robot 0.0063086; // old robot 0.1069014

bool elbowsMoving = false;
float elbowAdjust = 0;

int getSpeed(int percentComplete) {
  return 100 - ((percentComplete^8)*(10^14));
}

int sign(int x) {
  return (x > 0) - (x < 0);
}

float accelSpeed(int speed) {
  return speed/20 + 100;
}

void spinIntake() {
  intake.spin(fwd, 100, pct);
  //outakeChainLift.spin(fwd, 100, pct);
  // intakeR.spin(fwd, 100, pct);
  // outakeBottom.spin(fwd, 100, pct);

}

void stopIntake() {
  intake.stop(brake);
  //outakeChainLift.stop(brake);
  // intakeR.stop(brake);
  // outakeBottom.stop(brake);
}

struct SpinArgs {
    vex::motor* m;
    double position;
    int speed;
};

void spinThenHoldTask(void* arg) {
    SpinArgs* args = static_cast<SpinArgs*>(arg);

    args->m->spinToPosition(args->position, vex::rotationUnits::deg, args->speed, vex::velocityUnits::pct, true);
    args->m->stop(hold);

    delete args;
}

void spinThenHold(vex::motor& m, double position, int speed) {
    SpinArgs* args = new SpinArgs{&m, position, speed};
    vex::thread(spinThenHoldTask, args);
}

void spinIntakePush(int msecs) {
  intake.spin(fwd, 100, pct);
  // intakeR.spin(fwd, 100, pct);
  // outakeBottom.spin(fwd, 100, pct);
  driveTrain.spin(fwd, 10, pct);
  wait (msecs, vex::timeUnits::msec);
  intake.stop(brake);
  // intakeR.stop(brake);
  // outakeBottom.stop(brake);
  driveTrain.stop(brake);
}

// void moveStraight(float distance, int speed) {
//   if (distance == 0) return;

//   int direction = (distance > 0) ? 1 : -1;
//   int degreesToTurn = abs(degPerInch * distance);

//   leftSide.resetPosition();
//   rightSide.resetPosition();
//   wait(50, msec);

//   while ((abs(leftSide.position(deg)) + abs(rightSide.position(deg))) / 2 < degreesToTurn) {
//     int percent = (abs(leftSide.position(deg)) * 100) / degreesToTurn;
//     int calcSpeed = speed * (100 - percent) / 100;
//     calcSpeed = std::max(10, calcSpeed);

//     leftSide.spin(fwd, direction * calcSpeed, pct);
//     rightSide.spin(fwd, direction * calcSpeed, pct);

//     wait(10, msec);
//   }

//   leftSide.stop(brake);
//   rightSide.stop(brake);
// }

void elbow_up (void){
  elbow1.spin(fwd,75,pct);
  elbow2.spin(fwd,-75,pct);

}
void elbow_down (void){
  elbow1.spin(fwd,-75,pct);
  elbow2.spin(fwd,-75,pct);
}
void elbow_stop (void){
  elbow1.stop(hold);
  elbow2.stop(hold);



}
void moveTime(float seconds, int speed) {
  driveTrain.spin(fwd, speed, pct);
  wait(seconds * 1000, vex::timeUnits::msec);
  driveTrain.stop(brake);
}

// ============================================================================
//  PASTE THIS BLOCK in main.cpp, right after moveTime() and before pre_auton().
//  (See the notes in chat for what to delete / change elsewhere.)
// ============================================================================

// ---------------------------------------------------------------------------
//  Drivetrain geometry
// ---------------------------------------------------------------------------
const double PI_CONST          = 3.14159265358979;
const double WHEEL_DIAMETER_IN = 3.25;
const double WHEEL_CIRC_IN     = PI_CONST * WHEEL_DIAMETER_IN;   // 10.21 in
const double TRACK_RADIUS_IN   = 10.25;      // robot center to wheels
const double MOTOR_TEETH       = 36.0;       // gear on the motor
const double WHEEL_TEETH       = 48.0;       // gear on the wheel
// Motor encoder degrees per 1 wheel degree (36T drives 48T -> 48/36).
// The encoder reads the motor output shaft (after the 18:1 green cartridge),
// so the cartridge doesn't change this number as long as the motors are
// configured as ratio18_1 (green) in the robot config.
const double MOTOR_DEG_PER_WHEEL_DEG = WHEEL_TEETH / MOTOR_TEETH;  // 1.3333

// Motor degrees needed to travel one inch (= 47.01)
const double degPerInch = 360.0 * MOTOR_DEG_PER_WHEEL_DEG / WHEEL_CIRC_IN;

// Tuning knobs
double turnScale = 1.0;   // raise if the robot under-turns, lower if it over-turns
const double MIN_DRIVE_PCT  = 12;    // slowest speed during the final approach
const double SYNC_KP        = 0.3;   // pct per degree of left/right mismatch

// ---------------------------------------------------------------------------
//  Core encoder-based mover used by both moveStraight and turnRobot.
//  targetDeg is how far each side should go (always positive).
//  leftDir/rightDir are +1 or -1.
// ---------------------------------------------------------------------------
static void driveEncoder(double targetDeg, int maxSpeed, int leftDir, int rightDir) {
  if (targetDeg <= 0 || maxSpeed <= 0) return;

  leftSide.resetPosition();
  rightSide.resetPosition();
  wait(50, msec);

  const double accelDeg = std::min(targetDeg * 0.25, 100.0);
  const double decelDeg = std::min(targetDeg * 0.50, 200.0);

  // Safety timeout: 200 rpm motor = 1200 deg/s at 100%.
  double expectedMs = targetDeg / (12.0 * maxSpeed) * 1000.0;
  double timeoutMs  = expectedMs * 2.0 + 500;

  vex::timer t;
  t.clear();

  while (t.time(msec) < timeoutMs) {
    double l = fabs(leftSide.position(deg));
    double r = fabs(rightSide.position(deg));
    double traveled  = (l + r) / 2.0;
    double remaining = targetDeg - traveled;
    if (remaining <= 0) break;

    double v = maxSpeed;
    if (accelDeg > 0 && traveled < accelDeg)
      v = MIN_DRIVE_PCT + (maxSpeed - MIN_DRIVE_PCT) * (traveled / accelDeg);
    if (decelDeg > 0 && remaining < decelDeg)
      v = std::min(v, MIN_DRIVE_PCT + (maxSpeed - MIN_DRIVE_PCT) * (remaining / decelDeg));
    v = std::max(v, MIN_DRIVE_PCT);
    v = std::min(v, (double)maxSpeed);

    // Keep both sides covering the same distance so it drives straight / turns evenly.
    double corr = std::max(-10.0, std::min(10.0, (l - r) * SYNC_KP));

    leftSide.spin(fwd,  leftDir  * (v - corr), pct);
    rightSide.spin(fwd, rightDir * (v + corr), pct);
    wait(10, msec);
  }

  leftSide.stop(brake);
  rightSide.stop(brake);
  wait(100, msec);
}

// Drive straight. Positive = forward, negative = backward. Distance in inches.
void moveStraight(float distance, int speed) {
  if (distance == 0) return;
  int dir = (distance > 0) ? 1 : -1;
  driveEncoder(fabs(distance) * degPerInch, speed, dir, dir);
}

// Turn in place. Positive degrees = left side forward / right side backward
// (same direction as the original code).
void turnRobot(float degrees, int speed) {
  if (degrees == 0) return;
  int dir = (degrees > 0) ? 1 : -1;

  // Each wheel travels an arc of radius * angle (radians).
  double arcInches = TRACK_RADIUS_IN * fabs(degrees) * PI_CONST / 180.0 * turnScale;
  driveEncoder(arcInches * degPerInch, speed, dir, -dir);
}

// ---------------------------------------------------------------------------
//  Elbow / arm autonomous support
// ---------------------------------------------------------------------------
const double ELBOW_OUT_DEG       = 450.0;      // "out" position (in = 0)
const double ELBOW_COMP_RATIO    = 2.0 / 3.0;
const double ELBOW_KP            = 5.0;
const double ELBOW_DEADBAND      = 4.0;
const double ELBOW_MAX_DPS       = 600;
const double ELBOW_ARM_MOVE_DPS  = 5;
const double ELBOW_TOLERANCE     = 8.0;        // "close enough" for auton waits

const double ARM_MIN_DEG         = -30;        // same limits as driver control
const double ARM_MAX_DEG         = 260;
const double ARM_TOLERANCE       = 3.0;
const double ARM_DOWN_MAX_PCT    = 30;         // driver control lowers at 30%

// Shared between autonomous and driver control
double elbowRef1  = 0;      // elbow encoder positions when the elbow is "in"
double elbowRef2  = 0;
bool   elbowIsOut = false;  // current commanded elbow state

// Wrist: home position captured at startup, flipped = home + WRISTDEG
double wristHomeDeg    = 0;
bool   wristIsFlipped  = false;

// Hand (claw jaws). Driver control: Y drives toward -225, Down drives toward -10.
// I assumed -225 = closed and -10 = open. If it's backwards on your robot,
// just swap these two numbers.
const double HAND_CLOSED_DEG = -10;
const double HAND_OPEN_DEG   = -225;
const double HAND_TOLERANCE  = 6.0;

// Call once at startup (pre_auton) with the elbow pointing in and the arm down.
void captureReferences() {
  arm1.resetPosition();
  arm2.resetPosition();
  elbowRef1 = elbow1.position(deg);
  elbowRef2 = elbow2.position(deg);
  elbowIsOut = false;
  wristHomeDeg = wrist.position(deg);
  wristIsFlipped = false;
  elbow1.setStopping(hold);
  elbow2.setStopping(hold);
}

// One control step for the elbows (same math as driver control).
// Returns the larger of the two elbow errors in degrees.
static double elbowStep() {
  double armPos = (arm1.position(deg) + arm2.position(deg)) / 2.0;
  double base   = elbowIsOut ? ELBOW_OUT_DEG : 0.0;
  double comp   = armPos * ELBOW_COMP_RATIO;

  double target1 = elbowRef1 + base - comp;
  double target2 = elbowRef2 - base + comp;

  double armVel = (arm1.velocity(dps) + arm2.velocity(dps)) / 2.0;
  bool armMoving = fabs(armVel) > ELBOW_ARM_MOVE_DPS;
  if (!armMoving) armVel = 0;

  double ff1 = -armVel * ELBOW_COMP_RATIO;
  double ff2 =  armVel * ELBOW_COMP_RATIO;

  double err1 = target1 - elbow1.position(deg);
  double err2 = target2 - elbow2.position(deg);

  if (!armMoving && fabs(err1) < ELBOW_DEADBAND) {
    elbow1.stop(hold);
  } else {
    double s = std::max(-ELBOW_MAX_DPS, std::min(ELBOW_MAX_DPS, ff1 + err1 * ELBOW_KP));
    elbow1.spin(fwd, s, vex::velocityUnits::dps);
  }
  if (!armMoving && fabs(err2) < ELBOW_DEADBAND) {
    elbow2.stop(hold);
  } else {
    double s = std::max(-ELBOW_MAX_DPS, std::min(ELBOW_MAX_DPS, ff2 + err2 * ELBOW_KP));
    elbow2.spin(fwd, s, vex::velocityUnits::dps);
  }
  return std::max(fabs(err1), fabs(err2));
}

// Run the elbow controller until it's settled (or the timeout hits).
static void settleElbows(int timeoutMs) {
  vex::timer t;
  t.clear();
  while (t.time(msec) < timeoutMs) {
    if (elbowStep() < ELBOW_TOLERANCE) break;
    wait(20, msec);
  }
  elbow1.stop(hold);
  elbow2.stop(hold);
}

// Elbow to the OUT position (450 deg from in). Blocks until it gets there.
void clawOut() {
  elbowIsOut = true;
  settleElbows(3000);
}

// Elbow to the IN position (0 deg). Blocks until it gets there.
void clawIn() {
  elbowIsOut = false;
  settleElbows(3000);
}

// Move the arm to angleDeg (same units as the arm encoders in driver control:
// 0 = start, max 260, min -30). The elbows are driven the whole time so they
// compensate exactly like they do in driver control. Blocks until done.
void armMove(double angleDeg, int speed = 100) {
  angleDeg = std::max(ARM_MIN_DEG, std::min(ARM_MAX_DEG, angleDeg));

  vex::timer t;
  t.clear();
  while (t.time(msec) < 4000) {
    double pos = (arm1.position(deg) + arm2.position(deg)) / 2.0;
    double err = angleDeg - pos;
    if (fabs(err) <= ARM_TOLERANCE) break;

    // Slow down as we approach the target.
    double v = std::min(1.0, std::max(0.2, fabs(err) / 60.0)) * speed;
    if (err < 0) v = -std::min(v, ARM_DOWN_MAX_PCT);

    arm1.spin(fwd, v, pct);
    arm2.spin(fwd, v, pct);
    elbowStep();
    wait(20, msec);
  }
  arm1.stop(hold);
  arm2.stop(hold);

  settleElbows(1500);   // let the elbows finish catching up
}

// ---------------------------------------------------------------------------
//  Hand (claw jaws)
// ---------------------------------------------------------------------------
static void moveHand(double targetDeg, int speed, int timeoutMs) {
  vex::timer t;
  t.clear();
  while (t.time(msec) < timeoutMs) {
    double err = targetDeg - hand.position(deg);
    if (fabs(err) <= HAND_TOLERANCE) break;
    hand.spin(fwd, (err > 0 ? 1 : -1) * speed, pct);
    wait(10, msec);
  }
}

// Close the claw. Holds the motor afterward so it keeps gripping.
// If it stalls on a game piece it gives up after the timeout and keeps squeezing.
void clawClose(int speed = 100, int timeoutMs = 1500) {
  moveHand(HAND_CLOSED_DEG, speed, timeoutMs);
  hand.stop(hold);
}

// Open the claw and let the motor go limp (same as driver control).
void clawOpen(int speed = 100, int timeoutMs = 1500) {
  moveHand(HAND_OPEN_DEG, speed, timeoutMs);
  hand.stop(hold);
}

// ---------------------------------------------------------------------------
//  Wrist
// ---------------------------------------------------------------------------
// Go to the flipped (true) or home (false) position. Blocks until it arrives.
void wristSet(bool flipped, int speed = 50) {
  wristIsFlipped = flipped;
  double target = wristHomeDeg + (flipped ? WRISTDEG : 0);
  wrist.spinToPosition(target, vex::rotationUnits::deg, speed,
                       vex::velocityUnits::pct, false);

  vex::timer t;
  t.clear();
  while (t.time(msec) < 4000 && fabs(target - wrist.position(deg)) > 5.0) {
    wait(10, msec);
  }
}

// Toggle the wrist between home and flipped (same as the Up button).
void wristFlip(int speed = 50) {
  wristSet(!wristIsFlipped, speed);
}

// ---------------------------------------------------------------------------
//  Example usage inside autonomous():
//
//    armMove(150, 100);     // raise arm to 150 deg, elbows follow
//    clawOut();             // swing elbow out
//    moveStraight(12, 60);  // 12 inches forward
//    turnRobot(90, 50);     // 90 deg turn
//    clawClose();           // grab
//    wristFlip();           // flip the wrist
//    clawOpen();            // release
//    clawIn();
//    armMove(0);
// ---------------------------------------------------------------------------


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  armMove(-20, 100);
  
  // All activities that occur before the scompetition starts
  // Example: clearing encoders, setting servo positions, ...
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
  // Insert autonomous user code here
  if (autonMode == 1) {
    // Left Side Auton
    clawOut();
    moveStraight(-25,50);
    wait(1,sec);
    clawOpen();
    wait(1,sec);
    moveStraight(10,50);
    turnRobot(220,50);
    moveStraight(-25,50);
    printf("howdy");
  } else if (autonMode == 2) {
    // Right Side Auton
    

  } else if (autonMode == 3) {
    // Skills Auton
    
  } else {
    moveStraight(20, 100);
  }
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
  // Reset arm encoders once at the start of driver control.
  arm1.resetPosition();
  arm2.resetPosition();

  // Elbow toggle setup:
  // Start driver control with the elbow physically pointing straight down.
  // Its current encoder positions become the 0-degree reference.
  // (The elbow encoders are NOT reset, so no precision is lost.)
  const double elbowCompensationRatio = 2.0 / 3.0;
  const double elbowDownReference1 = elbow1.position(deg);
  const double elbowDownReference2 = elbow2.position(deg);

  // Elbow controller tuning. The elbow speed is the sum of:
  //   1) feedforward: the arm's measured speed * compensation ratio, so the elbow
  //      starts moving the instant the arm does and keeps pace with it, and
  //   2) feedback: error * Kp, which keeps driving the elbow until it actually
  //      reaches its destination (this is what moves it after pressing A).
  // All speeds are in degrees per second (dps), so they don't depend on cartridge.
  const double elbowKp = 5.0;          // dps per degree of error; raise if it lags, lower if it oscillates
  const double elbowDeadband = 4.0;    // degrees of error to ignore when everything is at rest
  const double elbowMaxDps = 600;      // cap on elbow speed (dps)
  const double armMovingThreshold = 5; // dps; arm speeds below this count as "not moving"

  // Make the elbows hold their position when stopped so gravity can't drift them.
  elbow1.setStopping(hold);
  elbow2.setStopping(hold);

  // Button A toggles between down (0 degrees) and out (450 degrees).
  bool elbowOut = false;
  bool previousButtonA = false;

  // Wrist toggle setup:
  // The wrist's position at the start of driver control is the "home" position.
  const double wristHome = wrist.position(deg);
  bool wristOut = false;
  bool previousButtonUp = false;

  int loopCount = 0;

  while (1) {

    // Debug print, throttled to about twice per second so it doesn't slow the loop.
    if (loopCount % 25 == 0) {
      printf("%f\n", hand.position(deg));
    }
    loopCount++;

    // Tank drive controls
    L1.spin(fwd, (CT1.Axis3.value()*dSpeed/1.270), pct);
    L2.spin(fwd, (CT1.Axis3.value()*dSpeed/1.270), pct);
    R1.spin(fwd, (CT1.Axis2.value()*dSpeed/1.270), pct);
    R2.spin(fwd, (CT1.Axis2.value()*dSpeed/1.270), pct); 

    if (CT1.ButtonLeft.pressing()) {
      // Starts spitting out for outake
      // intakeR.spin(fwd, 100, pct);
      intake.spin(fwd, 100, pct);
      intake2.spin(fwd, -100, pct);
     // outakeOn = true;
    } else if (CT1.ButtonRight.pressing()) {
      // Stops sucking in and starts spitting out for intake
      // Stops spitting out and start sucking in for outake
      // intakeR.spin(fwd, -100, pct);
      intake.spin(fwd, -100, pct);
      intake2.spin(fwd, 100, pct);
      //outakeOn = false;
    } else if (CT1.ButtonB.pressing()) {
   
      intake.stop(coast);
      intake2.stop(coast);
    } 

    bool armUp = CT1.ButtonL1.pressing() &&
                 arm1.position(deg) < 260 && arm2.position(deg) < 260;
    bool armDown = CT1.ButtonL2.pressing() &&
                   arm1.position(deg) > -30 && arm2.position(deg) > -30;

    // Button A toggles once per press (holding it will not repeatedly toggle).
    bool buttonAPressed = CT1.ButtonA.pressing();
    if (buttonAPressed && !previousButtonA) {
      elbowOut = !elbowOut;
    }
    previousButtonA = buttonAPressed;

    // Control the arm with the existing L1/L2 buttons.
    if (armUp) {
      arm1.spin(fwd, 100, pct);
      arm2.spin(fwd, 100, pct);
    } else if (armDown) {
      arm1.spin(fwd, -30, pct);
      arm2.spin(fwd, -30, pct);
    } else {
      arm1.stop(hold);
      arm2.stop(hold);
    }

    // Use the arm's actual encoder position to compensate the elbow.
    double currentArmPosition =
        (arm1.position(deg) + arm2.position(deg)) / 2.0;

    // Down = the saved starting elbow position; out = 450 degrees from down.
    double baseElbowOffset = elbowOut ? 450.0 : 0.0;
    double armCompensation =
        currentArmPosition * elbowCompensationRatio;

    double elbow1Target =
        elbowDownReference1 + baseElbowOffset - armCompensation;

    double elbow2Target =
        elbowDownReference2 - baseElbowOffset + armCompensation;

    // Feedforward: how fast the arm is actually moving right now.
    double armVelocity = (arm1.velocity(dps) + arm2.velocity(dps)) / 2.0;
    bool armMoving = fabs(armVelocity) > armMovingThreshold;
    if (!armMoving) armVelocity = 0;

    // The targets above are (base - armComp) for elbow1 and (base + armComp) for
    // elbow2, so the elbows must move at -ratio and +ratio times the arm speed.
    double ff1 = -armVelocity * elbowCompensationRatio;
    double ff2 =  armVelocity * elbowCompensationRatio;

    // Feedback: keeps pushing toward the target until it is reached.
    double err1 = elbow1Target - elbow1.position(deg);
    double err2 = elbow2Target - elbow2.position(deg);

    // Hold still only when both the arm and the elbow are settled. While the arm
    // is moving we always command the elbow, so it never stutters or waits.
    if (!armMoving && fabs(err1) < elbowDeadband) {
      elbow1.stop(hold);
    } else {
      double s = std::max(-elbowMaxDps, std::min(elbowMaxDps, ff1 + err1 * elbowKp));
      elbow1.spin(fwd, s, vex::velocityUnits::dps);
    }

    if (!armMoving && fabs(err2) < elbowDeadband) {
      elbow2.stop(hold);
    } else {
      double s = std::max(-elbowMaxDps, std::min(elbowMaxDps, ff2 + err2 * elbowKp));
      elbow2.spin(fwd, s, vex::velocityUnits::dps);
    }

    if (CT1.ButtonY.pressing() && hand.position(deg) > -225) {
      hand.spin(fwd, -100, pct);
    } else if (CT1.ButtonDown.pressing() && hand.position(deg) < -10) {
      hand.spin(fwd, 100, pct);
    } else {
      hand.stop(coast);
    }

    // Button Up toggles the wrist once per press (holding won't re-toggle).
    bool buttonUpPressed = CT1.ButtonUp.pressing();
    if (buttonUpPressed && !previousButtonUp) {
      wristOut = !wristOut;
      double wristTarget = wristHome + (wristOut ? WRISTDEG : 0);
      // Non-blocking so drive/arm/elbow keep running while the wrist moves.
      wrist.spinToPosition(wristTarget, vex::rotationUnits::deg,
                           50, vex::velocityUnits::pct, false);
    }
    previousButtonUp = buttonUpPressed;

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

void setAuton(int autonNumber) {
  // Set autonomous mode.
  autonMode = autonNumber;
  CT1.rumble("-.-.-");
}

void goToPage(int pageId) {
  brainDisplay.switchPage(pageId);
}


void drawOnScreen() {
  //Place code here

  Button returnButton1;
  returnButton1.x = 350;
  returnButton1.y = 200;
  returnButton1.width = 140;
  returnButton1.height = 60;
  returnButton1.callback = goToPage;
  returnButton1.param = 0;
  char newText[20] = "Return";
  strcpy(returnButton1.text, newText);
  returnButton1.color = 0xff0000;
  brainDisplay.createButton(2,0, returnButton1);
  brainDisplay.addImage(2);

  Button skillsAutonButton;
  skillsAutonButton.x = 40;
  skillsAutonButton.y = 60;
  skillsAutonButton.width = 140;
  skillsAutonButton.height = 60;
  skillsAutonButton.callback = setAuton;     //  setAuton(3);
  skillsAutonButton.param = 3;
  char newText2[20] = "Skills Auton";
  strcpy(skillsAutonButton.text, newText2);
  skillsAutonButton.color = 0x04cc33;
  brainDisplay.createButton(1,0, skillsAutonButton);

  Button returnButton2;
  returnButton2.x = 40;
  returnButton2.y = 160;
  returnButton2.width = 140;
  returnButton2.height = 60;
  returnButton2.callback = goToPage;
  returnButton2.param = 0;
  char newText3[20] = "Return";
  strcpy(returnButton2.text, newText3);
  returnButton2.color = 0xff0000;
  brainDisplay.createButton(1,1, returnButton2);

  Button leftSideButton;
  leftSideButton.x = 300;
  leftSideButton.y = 60;
  leftSideButton.width = 140;
  leftSideButton.height = 60;
  leftSideButton.callback = setAuton;
  leftSideButton.param = 1;
  char newText4[20] = "Left Side";
  strcpy(leftSideButton.text, newText4);
  leftSideButton.color = 0x02749e;
  brainDisplay.createButton(1,2, leftSideButton);

  Button rightSideButton;
  rightSideButton.x = 300;
  rightSideButton.y = 160;
  rightSideButton.width = 140;
  rightSideButton.height = 60;
  rightSideButton.callback = setAuton;
  rightSideButton.param = 2;
  char newText5[20] = "Right Side";
  strcpy(rightSideButton.text, newText5);
  rightSideButton.color = 0x02749e;
  brainDisplay.createButton(1,3, rightSideButton);

  Button autonButton;
  autonButton.x = 180;
  autonButton.y = 160;
  autonButton.width = 140;
  autonButton.height = 60;
  autonButton.callback = goToPage;
  autonButton.param = 1;
  char newText6[20] = "Auton";
  strcpy(autonButton.text, newText6);
  autonButton.color = 0x02749e;
  brainDisplay.createButton(0,0, autonButton);
  brainDisplay.switchPage(0);

  Button imageButton;
  imageButton.x = 180;
  imageButton.y = 60;
  imageButton.width = 140;
  imageButton.height = 60;
  imageButton.callback = goToPage;
  imageButton.param = 2;
  char newText7[20] = "Image";
  strcpy(imageButton.text, newText7);
  imageButton.color = 0xff0000;
  brainDisplay.createButton(0,1, imageButton);

}

//
// Main will set up the competition functions and callbacks.
//

void screenCallback() {
  brainDisplay.screenPressed();
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Brain.Screen.pressed(screenCallback);
  drawOnScreen();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}