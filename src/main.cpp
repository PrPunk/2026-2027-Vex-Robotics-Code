/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Josiah Schatz and Sherwin Sagadam                         */
/*    Created:      Sat 23 Aug 2025                                           */
/*    Description:  Team 986B bot for 2025-2026                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "brain-display.cpp"
using namespace vex;
BrainDisplay brainDisplay; 

// A global instance of competition
competition Competition;

vex::timer accelTimer;

const int accelStep = 10;
const int accelSec = 300;

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
  outakeChainLift.spin(fwd, 100, pct);
  // intakeR.spin(fwd, 100, pct);
  // outakeBottom.spin(fwd, 100, pct);

}

void stopIntake() {
  intake.stop(brake);
  outakeChainLift.stop(brake);
  // intakeR.stop(brake);
  // outakeBottom.stop(brake);
}

void spinIntakeOutake(int msecs) {
  intake.spin(fwd, 100, pct);
  outakeTop.spin(fwd, 100, pct);
  outakeBottom.spin(fwd, 100, pct);
  outakeChainLift.spin(fwd, 100, pct);
  wait (msecs, vex::timeUnits::msec);
  intake.stop(brake);
  outakeTop.stop(brake);
  outakeBottom.stop(brake);
  outakeChainLift.stop(brake);
}

void reverseOutakeIntake(int msecs) {
  intake.spin(fwd, -100, pct);
  outakeTop.spin(fwd, -100, pct);
  outakeBottom.spin(fwd, -100, pct);
  outakeChainLift.spin(fwd, -100, pct);
  wait (msecs, vex::timeUnits::msec);
  intake.stop(brake);
  outakeTop.stop(brake);
  outakeBottom.stop(brake);
  outakeChainLift.stop(brake);
}

void reverseOutake(int msecs) {
  outakeTop.spin(fwd, -100, pct);
  outakeBottom.spin(fwd, -100, pct);
  outakeChainLift.spin(fwd, -100, pct);
  wait (msecs, vex::timeUnits::msec);
  outakeTop.stop(brake);
  outakeBottom.stop(brake);
  outakeChainLift.stop(brake);
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
//   int degreesToTurn = degPerInch * distance;
//   // driveTrain.resetPosition();
//   // wait(100, vex::timeUnits::msec);
//   // driveTrain.spinToPosition(degreesToTurn, deg, speed, vex::velocityUnits::pct);
//   // driveTrain.stop(brake);
//   leftSide.resetPosition();
//   rightSide.resetPosition();
//   wait(100, vex::timeUnits::msec);
//   if (distance > 0) {
//     // Move Forward
//     while (leftSide.position(deg) <= degreesToTurn && rightSide.position(deg) <= degreesToTurn) {
//       int calcSpeed = int(float(getSpeed(int(abs(leftSide.position(deg)*100/degreesToTurn)))*speed)/100);
//       leftSide.spin(fwd, calcSpeed, pct);
//       rightSide.spin(fwd, calcSpeed, pct);
//       wait(calcSpeed/10, msec);
//     }
//     leftSide.stop(brake);
//     rightSide.stop(brake);
//   } else if (distance < 0) {
//     // Turn clockwise
//     while (leftSide.position(deg) >= degreesToTurn && rightSide.position(deg) >= degreesToTurn) {
//       int calcSpeed = int(float(getSpeed(int(abs(leftSide.position(deg)*100/degreesToTurn)))*speed)/100);
//       leftSide.spin(fwd, -calcSpeed, pct);
//       rightSide.spin(fwd, -calcSpeed, pct);
//       wait(calcSpeed/10, msec);
//     }
//     leftSide.stop(brake);
//     rightSide.stop(brake);
//   }
// }

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
// void turnRobot(int degrees, int speed) {
//   float inchesToTurn = inchPerDeg * degrees;
//   int degreesToTurn = degPerInch * inchesToTurn;
//   degreesToTurn = abs(degreesToTurn);
//   leftSide.resetPosition();
//   rightSide.resetPosition();
//   wait(100, vex::timeUnits::msec);
//   if (degrees > 0) {
//     // Turn counterclockwise
//     while (leftSide.position(deg) <= degreesToTurn && rightSide.position(deg) >= -degreesToTurn) {
//       leftSide.spin(fwd, speed, pct);
//       rightSide.spin(fwd, -speed, pct);
//     }
//     leftSide.stop(brake);
//     rightSide.stop(brake);
//     // leftSide.spinToPosition(degreesToTurn, deg, speed, vex::velocityUnits::pct);
//     // rightSide.spinToPosition(-degreesToTurn, deg, speed, vex::velocityUnits::pct);
//   } else if (degrees < 0) {
//     // Turn clockwise
//     while (leftSide.position(deg) >= -degreesToTurn && rightSide.position(deg) <= degreesToTurn) {
//       leftSide.spin(fwd, -speed, pct);
//       rightSide.spin(fwd, speed, pct);
//     }
//     leftSide.stop(brake);
//     rightSide.stop(brake);
//     // leftSide.spinToPosition(degreesToTurn, deg, speed, vex::velocityUnits::pct);
//     // rightSide.spinToPosition(-degreesToTurn, deg, speed, vex::velocityUnits::pct);
//   };
// }

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
    spinIntake();
    moveStraight(30, 30);
    stopIntake();
    turnRobot(115, 50);
    intakePistons.set(1);
    moveStraight(8, 15);
    moveTime(1,20);
    moveStraight(-1,20);
    //moveStraight(27, 15);
    spinIntake();
    wait(1500,msec);
    stopIntake();
    moveStraight(-15, 15);
    
    moveTime(1, -15);
    moveStraight(1, 20);
    spinIntakeOutake(1500);
    moveStraight(1, 20);
    intakePistons.set(0);
    moveStraight(-0.75, 20);
    reverseOutake(300);
    spinIntakeOutake(10000);

  } else if (autonMode == 2) {
    // turnRobot(90,50); //38 speeed for 90 degrees
    // Right Side Auton
    // spinIntake();
    // moveStraight(8.5, 30);
    // turnRobot(90, 30);
    // moveStraight(26.5, 35);
    // turnRobot(41, 25);
    // stopIntake();
    // moveStraight(12, 35);
    // reverseOutakeIntake(2000);
    // moveStraight(-43, 90);
    // turnRobot(140, 30);
    // intakePistons.set(1);
    // moveStraight(5, 35);
    // spinIntakePush(2500);
    // moveStraight(-15,25);
    // intakePistons.set(0);
    // outakePistons.set(1); 
    // moveTime(2, -15);
    // spinIntakeOutake(1500);
    // reverseOutake(300);
    // spinIntakeOutake(10000);
    spinIntake();
    moveStraight(30, 30);
    stopIntake();
    turnRobot(-110, 50);
    intakePistons.set(1);
    moveStraight(8, 15);
    moveTime(1,20);
    moveStraight(-1,20);
    //moveStraight(27, 15);
    spinIntake();
    wait(1500,msec);
    stopIntake();
    moveStraight(-15, 15);
    
    moveTime(1, -15);
    moveStraight(1, 20);
    spinIntakeOutake(1500);
    moveStraight(1, 20);
    intakePistons.set(0);
    moveStraight(-1, 20);
    reverseOutake(300);
    spinIntakeOutake(10000);

  } else if (autonMode == 3) {
    // Skills Auton
    spinIntake();
    moveStraight(32, 30);
    stopIntake();
    turnRobot(-85, 30);
    parkPistons.set(1);
    moveStraight(-16, 15);
    spinIntakeOutake(1500);
    moveStraight(29, 15);
    spinIntakePush(3000);
    moveStraight(-29, 15);
    spinIntakeOutake(10000);
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
  // User control code here, inside the loop
  while (1) {

    if (!slowDrive) {
      // Tank drive controls
      L1.spin(fwd, (CT1.Axis2.value()*dSpeed/1.270), pct);
      L2.spin(fwd, (CT1.Axis2.value()*dSpeed/1.270), pct);
      L3.spin(fwd, (CT1.Axis2.value()*dSpeed/1.270), pct);
      R1.spin(fwd, (CT1.Axis3.value()*dSpeed/1.270), pct);
      R2.spin(fwd, (CT1.Axis3.value()*dSpeed/1.270), pct);
      R3.spin(fwd, (CT1.Axis3.value()*dSpeed/1.270), pct);
    } else {
      //printf("%d, %d\n", CT1.Axis2.value(), CT1.Axis3.value()); 127 to -127
      
      if (accelTimer.time(vex::timeUnits::msec) >= accelStep) {
        accelTimer.reset();

        int leftTarget  = CT1.Axis2.value() * 100 / 127;
        int rightTarget = CT1.Axis3.value() * 100 / 127;

        // Time step in seconds
        float dt = accelStep / 1000.0f;
        
        // Get acceleration based on current speed
        float leftAccelMax  = accelSpeed(leftVeloc)  * dt;
        float rightAccelMax = accelSpeed(rightVeloc) * dt;
        // Ramp LEFT
        int leftError = leftTarget - leftVeloc;
        if (abs(leftError) > leftAccelMax) {
          leftVeloc += sign(leftError) * leftAccelMax;
        } else {
          leftVeloc = leftTarget;
        }

        // Ramp RIGHT
        int rightError = rightTarget - rightVeloc;
        if (abs(rightError) > rightAccelMax) {
          rightVeloc += sign(rightError) * rightAccelMax;
        } else {
          rightVeloc = rightTarget;
        }

        leftSide.spin(fwd, int(leftVeloc), pct);
        rightSide.spin(fwd, int(rightVeloc), pct);
      }
    }
    
    
    // Intake and outake controls
    if (CT1.ButtonR2.pressing()) {
      // Starts sucking in for the intake
      intake.spin(fwd, 100, pct);
      outakeChainLift.spin(fwd, 100, pct);
      intakeOn = true;
    } else if (CT1.ButtonR1.pressing()) {
      // Stops sucking in and starts spitting out for intake
      // Stops spitting out and start sucking in for outake
      intake.spin(fwd, -100, pct);
      outakeChainLift.spin(fwd, -100, pct);
      intakeOn = false;
    } else if (!intakeOn) {
      intake.stop(coast);
      outakeChainLift.stop(coast);
    }

    if (CT1.ButtonL2.pressing()) {
      // Starts spitting out for outake
      // intakeR.spin(fwd, 100, pct);
      outakeTop.spin(fwd, 100, pct);
      outakeBottom.spin(fwd, 100, pct);
      outakeOn = true;
    } else if (CT1.ButtonL1.pressing()) {
      // Stops sucking in and starts spitting out for intake
      // Stops spitting out and start sucking in for outake
      // intakeR.spin(fwd, -100, pct);
      outakeTop.spin(fwd, -100, pct);
      outakeBottom.spin(fwd, -100, pct);
      outakeOn = false;
    } else if (!outakeOn) {
      // intakeR.stop(coast);
      outakeTop.stop(coast);
      outakeBottom.stop(coast);
    }

    if (CT1.ButtonDown.pressing()) {
      parkPistons.set(0);
    } else if (CT1.ButtonUp.pressing()) {
      parkPistons.set(1);
    }

    if (CT1.ButtonX.pressing()) {
      intakePistons.set(0);
    } else if (CT1.ButtonB.pressing()) {
      intakePistons.set(1);
    }

    // Eliminates the judges if they give a bad score for our robot.
    if (CT1.ButtonLeft.pressing()) {
      descoreArm.set(1);
    } else if (CT1.ButtonRight.pressing()) {
      descoreArm.set(0);
    }

    if (CT1.ButtonL2.pressing()) {
      dSpeed = 0.3;
    } else {
      dSpeed = 1;
    }

    if (CT1.ButtonY.pressing()) {
      ballLockPiston.set(1);
    } else if (CT1.ButtonA.pressing()) {
      ballLockPiston.set(0);
    }

    // if (CT1.ButtonY.pressing() && CT1.ButtonB.pressing()) {
    //   slowDrive = true;
    // } else if (CT1.ButtonY.pressing() && CT1.ButtonX.pressing()) {
    //   slowDrive = false;
    // }
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

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
