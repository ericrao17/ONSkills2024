#pragma once

#include "vex.h"

#include "Arm.h"
#include "Button.h"

using namespace vex;

class Robot {
    public:
        Robot(brain *robotBrain, competition *robotCompetition, controller *robotController, motor *leftDrive, motor *rightDrive, motor *arm, motor *claw, motor *snowPile);
        void drive();
    private:
        void reset();

        void toggleSnowPile();

        void moveMotor(MotorName name, float speed, directionType direction);
        void stopMotor(MotorName name);
        motor *getMotor(MotorName name);

        Button *resetButton;
        Button *snowPileToggleButton;
        Button *preciseToggleButton;

        Button *armUp;
        Button *armDown;

        Button *nextPositionButton;
        Button *previousPositionButton;

        Button *clawOpen;
        Button *clawClose;

        brain *robotBrain;
        competition *robotCompetition;
        controller *robotController;

        motor *leftDrive;
        motor *rightDrive;
        motor *arm;
        motor *claw;
        motor *snowPile;
        
        const int ARM_SPEED = 30;

        bool snowPileUp = false;
        bool preciseMode = false;
};