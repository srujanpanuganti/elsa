/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 5
  #define LEFT_MOTOR_BACKWARD  6
  #define RIGHT_MOTOR_FORWARD  9
  #define LEFT_MOTOR_FORWARD   10
  #define RIGHT_MOTOR_ENABLE 12
  #define LEFT_MOTOR_EN
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);




//RIGHT_MOTOR_BACKWARD 5
//LEFT_MOTOR_BACKWARD  6
//RIGHT_MOTOR_FORWARD  9
//LEFT_MOTOR_FORWARD   10
//RIGHT_MOTOR_ENABLE 12
//LEFT_MOTOR_ENABLE 13




//  #define RIGHT_MOTOR_BACKWARD 2
//  #define LEFT_MOTOR_BACKWARD  7
//  #define RIGHT_MOTOR_FORWARD  4
//  #define LEFT_MOTOR_FORWARD   8
//  #define RIGHT_MOTOR_ENABLE 5
//  #define LEFT_MOTOR_ENABLE 6
