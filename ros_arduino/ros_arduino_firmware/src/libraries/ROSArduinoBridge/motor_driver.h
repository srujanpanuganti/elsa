/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_ENABLE 8
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);


//MOD 5
//  #define RIGHT_MOTOR_BACKWARD 9
//  #define LEFT_MOTOR_BACKWARD  5
//  #define RIGHT_MOTOR_FORWARD  8
//  #define LEFT_MOTOR_FORWARD   6
//  #define RIGHT_MOTOR_ENABLE 12
//  #define LEFT_MOTOR_ENABLE 13


// Mod4
//  #define RIGHT_MOTOR_BACKWARD 8
//  #define LEFT_MOTOR_BACKWARD  4
//  #define RIGHT_MOTOR_FORWARD  7
//  #define LEFT_MOTOR_FORWARD   2
//  #define RIGHT_MOTOR_ENABLE 6
//  #define LEFT_MOTOR_ENABLE 5


//Actual
//  #define RIGHT_MOTOR_BACKWARD 5
//  #define LEFT_MOTOR_BACKWARD  6
//  #define RIGHT_MOTOR_FORWARD  9
//  #define LEFT_MOTOR_FORWARD   10
//  #define RIGHT_MOTOR_ENABLE 12
//  #define LEFT_MOTOR_ENABLE 13


// Mod 2
//  #define RIGHT_MOTOR_BACKWARD 5
//  #define LEFT_MOTOR_BACKWARD  9
//  #define RIGHT_MOTOR_FORWARD  6
//  #define LEFT_MOTOR_FORWARD   10
//  #define RIGHT_MOTOR_ENABLE 2
//  #define LEFT_MOTOR_ENABLE 4

// Mod 1
//  #define RIGHT_MOTOR_BACKWARD 2
//  #define LEFT_MOTOR_BACKWARD  7
//  #define RIGHT_MOTOR_FORWARD  4
//  #define LEFT_MOTOR_FORWARD   8
//  #define RIGHT_MOTOR_ENABLE 5
//  #define LEFT_MOTOR_ENABLE 6

// Actual
//RIGHT_MOTOR_BACKWARD 5
//LEFT_MOTOR_BACKWARD  6
//RIGHT_MOTOR_FORWARD  9
//LEFT_MOTOR_FORWARD   10
//RIGHT_MOTOR_ENABLE 12
//LEFT_MOTOR_ENABLE 13