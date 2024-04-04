#include "Robot.h"

using namespace vex;

Robot::Robot(brain *robotBrain, competition *robotCompetition, controller *robotController, motor *leftDrive, motor *rightDrive, motor *arm, motor *claw, motor *snowPile) {
    this->robotBrain = robotBrain;
    this->robotCompetition = robotCompetition;
    this->robotController = robotController;

    this->leftDrive = leftDrive;
    this->rightDrive = rightDrive;

    this->arm = arm;
    
    this->claw = claw;
    this->snowPile = snowPile;
    
    this->arm->setStopping(brake);
    this->claw->setStopping(brake);
    this->snowPile->setStopping(brake);

    this->resetButton = new Button(&robotController->ButtonRight);
    this->snowPileToggleButton = new Button(&robotController->ButtonLeft);
    this->preciseToggleButton = new Button(&robotController->ButtonB);

    this->armUp = new Button(&robotController->ButtonL2);
    this->armDown = new Button(&robotController->ButtonL1);

    this->nextPositionButton = new Button(&robotController->ButtonX);
    this->previousPositionButton = new Button(&robotController->ButtonY);
    
    this->clawOpen = new Button(&robotController->ButtonUp);
    this->clawClose = new Button(&robotController->ButtonDown);
}

void Robot::reset() {
    
}

void Robot::toggleSnowPile() {  
  snowPile->setVelocity(30, rpm);

  if (snowPileUp) {
    snowPile->spinTo(135, degrees, false);
  } else {
    snowPile->spinTo(0, degrees, false);
  }

  snowPileUp = !snowPileUp;
}

void Robot::moveMotor(MotorName name, float speed, directionType direction) {
    motor *motor = getMotor(name);
    motor->setVelocity(speed, rpm);
    motor->spin(direction);
}

void Robot::stopMotor(MotorName name) {
    motor *motor = getMotor(name);
    motor->stop();
}

void Robot::drive() {
    resetButton->update();
    snowPileToggleButton->update();
    preciseToggleButton->update();
    armUp->update();
    armDown->update();
    nextPositionButton->update();
    previousPositionButton->update();
    clawOpen->update();
    clawClose->update();

    if(resetButton->justPressed()) {
        reset();
    }

    if(snowPileToggleButton->justPressed()) {
        toggleSnowPile();
    }

    if(preciseToggleButton->justPressed()) {
        preciseMode = !preciseMode;
    }

    if(armUp->justPressed()) {
        moveMotor(ARM_BASE, ARM_SPEED, forward);
    }
    if(armDown->justPressed()) {
        moveMotor(ARM_BASE, ARM_SPEED, reverse);
    }
    if(armUp->justReleased() || armDown->justReleased()) {
        stopMotor(ARM_BASE);
    }

    if(clawOpen->justPressed()) {
        moveMotor(CLAW, ARM_SPEED, forward);
    }
    if(clawClose->justPressed()) {
        moveMotor(CLAW, ARM_SPEED, reverse);
    }
    if(clawOpen->justReleased() || clawClose->justReleased()) {
        stopMotor(CLAW);
    }

    int leftJoystickY = robotController->Axis3.position(); //up/down values of left joystick - eric
    int rightJoystickX = robotController->Axis1.position();
    int steer = std::abs(robotController->Axis1.position())/10;
    int speed = (10 + std::abs(leftJoystickY/3));

    float turningSpeed = preciseMode ? 2 : 4;
    if (preciseMode) {
        speed *= 0.3;
    } else {
        speed = (10 + std::abs(leftJoystickY / 3));
    }

    /*
    if (leftJoystickY > 90){ //fwd
      leftDrive->spin(forward, 20, percent); //spin(direction, amount, unit *KEEP AS PERCENT*) - eric
      rightDrive->spin(forward, 20, percent);
    }
    else if (leftJoystickY < -90){ //backward
      leftDrive->spin(reverse, 20, percent); 
      rightDrive->spin(reverse, 20, percent);
    }
    else if (leftJoystickX > 90){ //turn right
      leftDrive->spin(forward, 20, percent);
      rightDrive->spin(reverse, 20, percent);
    }
    else if (leftJoystickX < -90){ //turn right
      leftDrive->spin(reverse, 20, percent);
      rightDrive->spin(forward, 20, percent);
    }
    else{
      leftDrive->stop(); 
      rightDrive->stop();
    }
    */
    if (leftJoystickY > 5){
        if (rightJoystickX > 5){
            leftDrive->spin(forward, speed + steer, percent);
            rightDrive->spin(forward, speed*0.5, percent);
        }
        else if (rightJoystickX < -5){
            leftDrive->spin(forward, speed*0.5, percent);
            rightDrive->spin(forward, speed + steer, percent);
        }
        else{
            leftDrive->spin(forward, speed, percent);
            rightDrive->spin(forward, speed, percent);
        }
    } else if (leftJoystickY < -5){
        if (rightJoystickX > 5) {
            leftDrive->spin(reverse, speed*0.5, percent);
            rightDrive->spin(reverse, speed + steer, percent);
        } else if (rightJoystickX < -5) {
            leftDrive->spin(reverse, speed + steer, percent);
            rightDrive->spin(reverse, speed*0.5, percent);
        } else {
            leftDrive->spin(reverse, speed, percent);
            rightDrive->spin(reverse, speed, percent);
        }
    } else if(rightJoystickX > 15) {
        leftDrive->spin(forward, speed * turningSpeed, percent);
        rightDrive->spin(reverse, speed * turningSpeed, percent);
    } else if(rightJoystickX < -15) {
        leftDrive->spin(reverse, speed * turningSpeed, percent);
        rightDrive->spin(forward, speed * turningSpeed, percent);
    } else {
        leftDrive->stop(); 
        rightDrive->stop();
    }
}

motor *Robot::getMotor(MotorName name) {
    if(name == ARM_BASE) {
        return arm;
    } else if(name == CLAW) {
        return claw;
    }
    return nullptr;
}