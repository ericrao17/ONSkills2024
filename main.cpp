
#include "vex.h"

using namespace vex;

// Robot configuration code.

// Brain should be defined by default
brain Brain;

motor ClawMotor = motor(PORT10, ratio18_1, false);
motor ArmMotor1 = motor(PORT9, ratio18_1, true);
motor ArmMotor2 = motor(PORT8, ratio18_1, false);

motor snowpile = motor(PORT1, ratio18_1, false);
motor LeftDriveMotor = motor(PORT12, ratio18_1, false);
motor RightDriveMotor = motor(PORT11, ratio18_1, true);
motor TrayMotor = motor(PORT2, ratio18_1, true);

controller Controller1 = controller(primary);

#include "vex.h"
#include <cmath>
#include <bits/stdc++.h>

using namespace vex;

competition Competition;

// define your global instances of motors and other devices here
//NOT NEEDED IF USING VEXCODE - eric

int GEAR_RATIO = 7;
int ARM_SPEED = 60;

float a = 8.0;
float b = 12.5;

float toDegrees(float radians) {
  return radians * 180.0 / M_PI;
}

float toRadians(float degrees) {
  return degrees * M_PI / 180.0;
}

void printC(const char *str){
  Controller1.Screen.setCursor(0, 0);
  Controller1.Screen.clearLine(0);
  Controller1.Screen.print(str);
}

bool moveArm(float *anglesStart, float *coords, float *angles) {
  float x = coords[0];
  float y = coords[1];
  float alphaStart = anglesStart[0];
  float betaStart = anglesStart[1];

  float r = sqrt(pow(x, 2) + pow(y, 2));

  float theta = 0;

  if (y != 0 && x != 0) {
    theta = toDegrees(std::atan2(std::abs(y), std::abs(x)));
  }

  float gammaCosineLaw = (powf(b, 2) - powf(a, 2) - powf(r, 2)) / (-2.0 * a * r);
  float zetaCosineLaw = (powf(r, 2) - powf(a, 2) - powf(b, 2)) / (-2.0 * a * b);

  if (std::abs(gammaCosineLaw) > 1 || std::abs(zetaCosineLaw) > 1) {
    return false;
  }

  float gamma = toDegrees(std::acos(gammaCosineLaw));
  float zeta = toDegrees(std::acos(zetaCosineLaw));

  float alpha = 0;
  float beta = 0;

  if (alphaStart > 0) {
    if(y > 0) {
      alpha = gamma + theta;
    } else {
      alpha = gamma - theta;
    }
  } else {
    if (y > 0) {
      alpha = gamma - theta;
    } else {
      alpha = gamma + theta;
    }
  }

  if (betaStart < 0) {
    if (y < 0) {
      beta = zeta - (180 - alpha);
    } else {
      beta = 180 - alpha - zeta;
    }
  } else {
    if (y < 0) {
      beta = 180 - alpha - zeta;
    } else {
      beta = zeta - (180 - alpha);
    }
  }

  if(alphaStart < 0) {
    alpha *= -1;
  }


  if(betaStart < 0){
    beta *= -1;
  }

  angles[0] = alpha;
  angles[1] = beta;

  return true;
}


bool calculateArm(float *angles, float *coords) {
  float alpha = angles[0];
  float beta = angles[1];

  float alphaAbs = std::abs(alpha);
  float betaAbs = std::abs(beta);

  // if(alphaAbs > 1 || betaAbs > 1) {
  //   return false;
  // }

  float alphaX = std::cos(toRadians(alphaAbs)) * a;
  float alphaY = std::sin(toRadians(alphaAbs)) * a;

  float betaX = std::cos(toRadians(betaAbs)) * b;
  float betaY = std::sin(toRadians(betaAbs)) * b;

  if(alpha < 0) {
    alphaY *= -1.0;
  }

  if(beta < 0) {
    betaY *= -1.0;
  }

  coords[0] = alphaX + betaX;
  coords[1] = alphaY + betaY;

  return true;
}

bool pullBack() {
  float startAngles[2] = { (float) ArmMotor1.position(degrees) / GEAR_RATIO, (float) ArmMotor2.position(degrees) / GEAR_RATIO };
  float endAngles[2] = { startAngles[0], startAngles[1] };
  float coords[2] = {0, 0};


  if(!calculateArm(startAngles, coords)) {
    printC("failed to do trig");
    return false;
  }

  
  while(moveArm(startAngles, coords, endAngles)) {
    coords[0] -= 0.1;

    if(std::abs(endAngles[0]) > 60 || std::abs(endAngles[1]) > 60) {
      break;
    }
  }

  moveArm(startAngles, coords, endAngles);

  
  // char buffer[256];
  // sprintf(buffer, "t:%f %f",  endAngles[0],  endAngles[1]);
  // printC(buffer);

  float distA = std::abs(endAngles[0] + startAngles[0]);
  float distB = std::abs(endAngles[1] + startAngles[1]);
  float distMax = fmax(distA, distB);

  float speedA = distA / distMax;
  float speedB = distB / distMax;

  ArmMotor1.setVelocity(speedA * ARM_SPEED, rpm);
  ArmMotor1.spinFor(endAngles[0] * GEAR_RATIO, degrees, false);

  ArmMotor2.setVelocity(speedB * ARM_SPEED, rpm);
  ArmMotor2.spinFor(endAngles[1] * GEAR_RATIO, degrees, false);

  return true;
}


void pullBackCallback() {
  bool success = pullBack();
  Controller1.Screen.clearLine(0);
  Controller1.Screen.setCursor(0, 0);
  // if(success) {
  //   Controller1.Screen.print("Arm retracted!");
  // } else {
  //   Controller1.Screen.print("Arm already retracted!");
  // }
}

void recalibrate() {
  ArmMotor1.setPosition(0, degrees);
  ArmMotor2.setPosition(0, degrees);
  Controller1.Screen.clearLine(0);
  Controller1.Screen.setCursor(0, 0);
  Controller1.Screen.print("Recalibrated arm!");
}

void arm1Up() {
  ArmMotor1.setVelocity(ARM_SPEED, rpm);
  ArmMotor1.spin(forward);
}

void arm1Down() {
  ArmMotor1.setVelocity(ARM_SPEED, rpm);
  ArmMotor1.spin(reverse);
}

void arm1Stop() {
  ArmMotor1.stop();
}

void arm2Up() {
  ArmMotor2.setVelocity(ARM_SPEED, rpm);
  ArmMotor2.spin(forward);
}

void arm2Down() {
  ArmMotor2.setVelocity(ARM_SPEED, rpm);
  ArmMotor2.spin(reverse);
}

void arm2Stop() {
  ArmMotor2.stop();
}

void snowPileUp(){
  snowpile.setVelocity(30, rpm);
  snowpile.spin(forward);
}

void snowPileDown(){
  snowpile.setVelocity(30, rpm);
  snowpile.spin(reverse);
}

void snowPileStop(){
  snowpile.stop();
}

void clawUp(){
  ClawMotor.setVelocity(30, rpm);
  ClawMotor.spin(forward);
}

void clawDown(){
  ClawMotor.setVelocity(30, rpm);
  ClawMotor.spin(reverse);
}

void clawStop(){
  ClawMotor.stop();
}

void TrayUp(){
  TrayMotor.setVelocity(30, rpm);
  TrayMotor.spin(forward);
}

void TrayDown(){
  TrayMotor.setVelocity(30, rpm);
  TrayMotor.spin(reverse);
}

void TrayStop(){
  TrayMotor.stop();
}


void usercontrol(void) {
  while (1) {

    int leftJoystickY = Controller1.Axis3.position(); //up/down values of left joystick - eric
    int rightJoystickX = Controller1.Axis1.position();
    int steer = std::abs(Controller1.Axis1.position())/10;
    int speed = 22 + std::abs(leftJoystickY/8);

    /*
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
    */
    if (leftJoystickY > 5){
      if (rightJoystickX > 5){
        LeftDriveMotor.spin(forward, speed + steer, percent);
        RightDriveMotor.spin(forward, speed*0.5, percent);
      }
      else if (rightJoystickX < -5){
        LeftDriveMotor.spin(forward, speed*0.5, percent);
        RightDriveMotor.spin(forward, speed + steer, percent);
      }
      else{
        LeftDriveMotor.spin(forward, speed, percent);
        RightDriveMotor.spin(forward, speed, percent);
      }
    }
    else if (leftJoystickY < -5){
      if (rightJoystickX > 5){
        LeftDriveMotor.spin(reverse, speed*0.5, percent);
        RightDriveMotor.spin(reverse, speed + steer, percent);
      }
      else if (rightJoystickX < -5){
        LeftDriveMotor.spin(reverse, speed + steer, percent);
        RightDriveMotor.spin(reverse, speed*0.5, percent);
      }
      else{
        LeftDriveMotor.spin(reverse, speed, percent);
        RightDriveMotor.spin(reverse, speed, percent);
      }
    }
    else if(rightJoystickX > 15){
        LeftDriveMotor.spin(forward, speed, percent);
        RightDriveMotor.spin(reverse, speed, percent);
    }
    else if(rightJoystickX < -15){
        LeftDriveMotor.spin(reverse, speed, percent);
        RightDriveMotor.spin(forward, speed, percent);
    }
    else{
      LeftDriveMotor.stop(); 
      RightDriveMotor.stop();
    }
  }
}

int main() {
  Competition.drivercontrol(usercontrol);


  ArmMotor1.setStopping(brake); //set motors to brake mode - eric
  ArmMotor2.setStopping(brake); 

  Controller1.ButtonX.pressed(pullBackCallback);
  Controller1.ButtonB.pressed(recalibrate);
  
  Controller1.ButtonL2.pressed(arm1Up);
  Controller1.ButtonL1.pressed(arm1Down);
  Controller1.ButtonL2.released(arm1Stop);
  Controller1.ButtonL1.released(arm1Stop);

  Controller1.ButtonR2.pressed(arm2Up);
  Controller1.ButtonR1.pressed(arm2Down);
  Controller1.ButtonR2.released(arm2Stop);
  Controller1.ButtonR1.released(arm2Stop);

  Controller1.ButtonLeft.pressed(clawUp);
  Controller1.ButtonRight.pressed(clawDown);
  Controller1.ButtonLeft.released(clawStop);
  Controller1.ButtonRight.released(clawStop);

  Controller1.ButtonY.pressed(snowPileUp);
  Controller1.ButtonA.pressed(snowPileDown);
  Controller1.ButtonY.released(snowPileStop);
  Controller1.ButtonA.released(snowPileStop);
  
  Controller1.ButtonUp.pressed(TrayUp);
  Controller1.ButtonDown.pressed(TrayDown);
  Controller1.ButtonUp.released(TrayStop);
  Controller1.ButtonDown.released(TrayStop);
  
  while (true) {
    wait(100, msec);
  }
}





