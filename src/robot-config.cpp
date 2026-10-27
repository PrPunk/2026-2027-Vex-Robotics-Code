#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller CT1 = controller(primary);

motor L1 = motor(PORT3, ratio6_1, false);
motor L2 = motor(PORT2, ratio6_1, false);
motor L3 = motor(PORT1, ratio6_1, false);
motor R1 = motor(PORT15, ratio6_1, true);
motor R2 = motor(PORT7, ratio6_1, true);
motor R3 = motor(PORT12, ratio6_1, true);
motor intake = motor(PORT6, ratio6_1, true);
motor outakeTop = motor(PORT17, ratio18_1, true);
motor outakeBottom = motor(PORT6, ratio18_1, false);
motor outakeChainLift = motor(PORT12, ratio18_1, false);
motor elbow1 = motor(PORT4, ratio6_1, false);
motor elbow2 = motor(PORT14, ratio6_1, true);
motor arm1 = motor(PORT10, ratio6_1, false);
motor arm2 = motor(PORT11, ratio6_1, true);
motor hand = motor(PORT5, ratio6_1, true);

motor_group leftSide = motor_group(L1, L2, L3);
motor_group rightSide = motor_group(R1, R2, R3);
motor_group driveTrain = motor_group(L1, L2, L3, R1, R2, R3);

digital_out parkPistons(Brain.ThreeWirePort.G);
digital_out ballLockPiston(Brain.ThreeWirePort.A);
digital_out intakePistons(Brain.ThreeWirePort.E);
digital_out descoreArm(Brain.ThreeWirePort.B);
digital_out pnu2(Brain.ThreeWirePort.C);
// VEXcode generated functions



// Initialize
void vexcodeInit( void ) {
  // Brain.Screen.print("Device initialization...");
  // Brain.Screen.setCursor(2, 1);
  // // calibrate the drivetrain gyro
  // wait(200, msec);
  // TurnGyroSmart.startCalibration(1);
  // Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // // wait for the gyro calibration process to finish
  // while (TurnGyroSmart.isCalibrating()) {
  //   wait(25, msec);
  // }
  // // reset the screen now that the calibration is complete
  // Brain.Screen.clearScreen();
  // Brain.Screen.setCursor(1,1);
  // wait(50, msec);
  // Brain.Screen.clearScreen();
}
