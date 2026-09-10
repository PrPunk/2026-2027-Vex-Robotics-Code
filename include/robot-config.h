// robot.config.h
using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller CT1;
extern motor L1;
extern motor L2;
extern motor L3;
extern motor R1;
extern motor R2;
extern motor R3;
extern motor intake;
extern motor outakeTop;
extern motor outakeBottom;
extern motor outakeChainLift;
extern digital_out parkPistons;
extern digital_out ballLockPiston;
extern digital_out intakePistons;
extern digital_out descoreArm;
extern digital_out pnu2;

extern motor_group leftSide;
extern motor_group rightSide;
extern motor_group driveTrain;

// Initialize
void  vexcodeInit( void );