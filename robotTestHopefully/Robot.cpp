#include "Robot.h"

using namespace vex;

Robot::Robot(brain *robotBrain, competition *robotCompetition, controller *robotController, motor *leftDrive, motor *rightDrive, motor *armBase, motor *armMid, motor *claw, motor *tray) {
    this->robotBrain = robotBrain;
    this->robotCompetition = robotCompetition;
    this->robotController = robotController;

    this->leftDrive = leftDrive;
    this->rightDrive = rightDrive;

    this->armBase = armBase;
    this->armMid = armMid;
    
    this->claw = claw;
    this->tray = tray;
    
    this->armBase->setStopping(brake);
    this->armMid->setStopping(brake);
    this->claw->setStopping(hold);
    this->tray->setStopping(hold);
    
    this->arm = new Arm(armBase, armMid);

    this->resetButton = new Button(&robotController->ButtonRight);
    this->trayToggleButton = new Button(&robotController->ButtonLeft);
    this->preciseToggleButton = new Button(&robotController->ButtonB);
    this->tipTrayButton = new Button(&robotController->ButtonA);

    this->armBaseUp = new Button(&robotController->ButtonL2);
    this->armBaseDown = new Button(&robotController->ButtonL1);
    this->armMidUp = new Button(&robotController->ButtonR2);
    this->armMidDown = new Button(&robotController->ButtonR1);

    this->nextPositionButton = new Button(&robotController->ButtonX);
    this->previousPositionButton = new Button(&robotController->ButtonY);
    
    this->clawOpen = new Button(&robotController->ButtonUp);
    this->clawClose = new Button(&robotController->ButtonDown);
}

void Robot::reset() {
    arm->reset();
}

bool Robot::pullBack() {
    return arm->pullBack();
}

void Robot::toggleTray() {  
  tray->setVelocity(30, rpm);

  if(trayUp) {
    tray->spinTo(135, degrees, false);
  } else {
    tray->spinTo(0, degrees, false);
  }

  trayUp = !trayUp;
}


void Robot::tipTray() {  
  tray->setVelocity(30, rpm);

  if(trayTipped) {
    tray->spinTo(45, degrees, false);
  } else {
    tray->spinTo(0, degrees, false);
  }

  trayTipped = !trayTipped;
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
    trayToggleButton->update();
    preciseToggleButton->update();
    tipTrayButton->update();
    armBaseUp->update();
    armBaseDown->update();
    armMidUp->update();
    armMidDown->update();
    nextPositionButton->update();
    previousPositionButton->update();
    clawOpen->update();
    clawClose->update();

    if(resetButton->justPressed()) {
        reset();
    }

    if(trayToggleButton->justPressed()) {
        toggleTray();
    }

    if(preciseToggleButton->justPressed()) {
        preciseMode = !preciseMode;
    }

    if(tipTrayButton->justPressed()) {

        char buffer[256];
        sprintf(buffer, "%.2f %.2f", armBase->position(degrees), armMid->position(degrees));
        robotController->Screen.clearScreen();
        robotController->Screen.setCursor(0, 0);
        robotController->Screen.print(buffer);

        tipTray();
    }

    if(armBaseUp->justPressed()) {
        moveMotor(ARM_BASE, ARM_SPEED, forward);
    }
    if(armBaseDown->justPressed()) {
        moveMotor(ARM_BASE, ARM_SPEED, reverse);
    }
    if(armBaseUp->justReleased() || armBaseDown->justReleased()) {
        stopMotor(ARM_BASE);
    }

    if(armMidUp->justPressed()) {
        moveMotor(ARM_MID, ARM_SPEED, forward);
    }
    if(armMidDown->justPressed()) {
        moveMotor(ARM_MID, ARM_SPEED, reverse);
    }
    if(armMidUp->justReleased() || armMidDown->justReleased()) {
        stopMotor(ARM_MID);
    }

    if(nextPositionButton->justPressed()) {
        arm->nextPosition();
    }
    if(previousPositionButton->justPressed()) {
        arm->previousPosition();
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
    int speed = (10 + std::abs(leftJoystickY/3)) / (preciseMode ? 4 : 1);

    float turningSpeed = preciseMode ? 2 : 4;

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
        return armBase;
    } else if(name == ARM_MID) {
        return armMid;
    } else if(name == CLAW) {
        return claw;
    }
    return nullptr;
}