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
float degPerInch = 25; // new robot 158.5127201; // old robot 47.012
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

void moveStraight(float distance, int speed) {
  if (distance == 0) return;

  int direction = (distance > 0) ? 1 : -1;
  int degreesToTurn = abs(degPerInch * distance);

  leftSide.resetPosition();
  rightSide.resetPosition();
  wait(50, msec);

  while ((abs(leftSide.position(deg)) + abs(rightSide.position(deg))) / 2 < degreesToTurn) {
    int percent = (abs(leftSide.position(deg)) * 100) / degreesToTurn;
    int calcSpeed = speed * (100 - percent) / 100;
    calcSpeed = std::max(10, calcSpeed);

    leftSide.spin(fwd, direction * calcSpeed, pct);
    rightSide.spin(fwd, direction * calcSpeed, pct);

    wait(10, msec);
  }

  leftSide.stop(brake);
  rightSide.stop(brake);
}
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

void turnRobot(int degrees, int speed) {
  if (degrees == 0) return;

  int direction = (degrees > 0) ? 1 : -1;

  float inchesToTurn = inchPerDeg * abs(degrees);
  int degreesToTurn = degPerInch * inchesToTurn;

  leftSide.resetPosition();
  rightSide.resetPosition();
  wait(50, msec);

  while ((abs(leftSide.position(deg)) + abs(rightSide.position(deg))) / 2 < degreesToTurn) {
    int percent = (abs(leftSide.position(deg)) * 100) / degreesToTurn;
    int calcSpeed = speed * (100 - percent) / 100;
    calcSpeed = std::max(10, calcSpeed);

    // Counterclockwise = positive degrees
    leftSide.spin(fwd,  direction * calcSpeed, pct);
    rightSide.spin(fwd, -direction * calcSpeed, pct);

    wait(10, msec);
  }

  leftSide.stop(brake);
  rightSide.stop(brake);
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
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
    elbow_up();
    wait(.5,sec);
    elbow_stop();
    moveStraight(-20,50);
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