#pragma once

#include "vex.h"
#include <cmath>
#include <bits/stdc++.h>

using namespace vex;

typedef struct {
  float alpha;
  float beta;
} ArmPosition;

typedef enum {
    ARM_BASE, ARM_MID, CLAW 
} MotorName;

class Arm {
    public:
        Arm(motor *armBase, motor *armMid);
        void reset();

        bool moveArm(float *anglesStart, float *coords, float *angles);
        bool calculateArm(float *angles, float *coords);
        bool pullBack();

        void nextPosition();
        void previousPosition();
        void setPosition(ArmPosition *position);
    private:

        float toDegrees(float radians);
        float toRadians(float degrees);

        const int GEAR_RATIO = 7;
        const int ARM_SPEED = 60;

        const float A = 8.0;
        const float B = 8.0;

        motor *armBase;
        motor *armMid;

        ArmPosition positions[3];

        int currentPosition = 0;
};