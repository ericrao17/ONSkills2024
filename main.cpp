/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// ClawMotor            motor         3               
// ArmMotor1            motor         9               
// ArmMotor2            motor         8               
// LeftDriveMotor       motor         12              
// RightDriveMotor      motor         11              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

// define your global instances of motors and other devices here
//NOT NEEDED IF USING VEXCODE - eric


void pre_auton(void) {
  vexcodeInit();  // INITIALIZES PROGRAM. DO NOT REMOVE! - eric
  //LeftDriveMotor.setVelocity(25, percent); doesnt seem to work (?) - eric
  //RightDriveMotor.setVelocity(25, percent);
  ArmMotor1.setStopping(brake); //set motors to brake mode - eric
  ArmMotor2.setStopping(brake); 
}

void usercontrol(void) {
  while (1) {
    int leftJoystickY = Controller1.Axis3.position(); //up/down values of left joystick - eric
    int leftJoystickX = Controller1.Axis4.position(); //left/right values of left joystick - eric
    if (leftJoystickY > 90){ //fwd
      LeftDriveMotor.spin(forward, 20, percent); //spin(direction, amount, unit *KEEP AS PERCENT*) - eric
      RightDriveMotor.spin(forward, 20, percent);
    }
    else if (leftJoystickY < -90){ //backward
      LeftDriveMotor.spin(reverse, 20, percent); 
      RightDriveMotor.spin(reverse, 20, percent);
    }
    else if (leftJoystickX > 90){ //turn right
      LeftDriveMotor.spin(forward, 20, percent);
      RightDriveMotor.spin(reverse, 20, percent);
    }
    else if (leftJoystickX < -90){ //turn right
      LeftDriveMotor.spin(reverse, 20, percent);
      RightDriveMotor.spin(forward, 20, percent);
    }
    else{
      LeftDriveMotor.stop(); 
      RightDriveMotor.stop();
    }
    wait(20, msec); 
  }
}

int main() {
  pre_auton(); //initialize - eric
  Competition.drivercontrol(usercontrol);

  while (true) {
    wait(100, msec);
  }
}
