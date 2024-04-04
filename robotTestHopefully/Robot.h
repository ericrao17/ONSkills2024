#pragma once

#include "vex.h"

#include "Arm.h"
#include "Button.h"

using namespace vex;

class Robot {
    public:
        Robot(brain *robotBrain, competition *robotCompetition, controller *robotController, motor *leftDrive, motor *rightDrive, motor *armBase, motor *armMid, motor *claw, motor *tray);
        void drive();
    private:
        void reset();

        bool pullBack();
        void toggleTray();
        void tipTray();

        void moveMotor(MotorName name, float speed, directionType direction);
        void stopMotor(MotorName name);
        motor *getMotor(MotorName name);

        Arm *arm;

        Button *resetButton;
        Button *pullButton;
        Button *trayToggleButton;
        Button *preciseToggleButton;
        Button *tipTrayButton;

        Button *armBaseUp;
        Button *armBaseDown;
        Button *armMidUp;
        Button *armMidDown;

        Button *nextPositionButton;
        Button *previousPositionButton;

        Button *clawOpen;
        Button *clawClose;

        brain *robotBrain;
        competition *robotCompetition;
        controller *robotController;

        motor *leftDrive;
        motor *rightDrive;
        motor *armBase;
        motor *armMid;
        motor *claw;
        motor *tray;
        
        const int ARM_SPEED = 60;

        bool trayUp = false;
        bool trayTipped = false;
        bool preciseMode = false;
};