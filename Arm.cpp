#include "Arm.h"

Arm::Arm(motor *armBase, motor *armMid) {
    this->armBase = armBase;
    this->armMid = armMid;


    positions[0] = {-465.6, -366.8};
    positions[1] = {-459.2, -53.2};
    positions[2] = {-462.4, 310.4};

    // positions[0] = {323, 490};
    // positions[1] = {-55, 439};
    // positions[2] = {-46, 508};
    // positions[3] =  {-388, 352};
}

void Arm::reset() {
  armBase->setPosition(0, degrees);
  armMid->setPosition(0, degrees);
}

bool Arm::moveArm(float *anglesStart, float *coords, float *angles) {
  float x = coords[0];
  float y = coords[1];
  float alphaStart = anglesStart[0];
  float betaStart = anglesStart[1];

  float r = sqrt(pow(x, 2) + pow(y, 2));

  float theta = 0;

  if (y != 0 && x != 0) {
    theta = toDegrees(std::atan2(std::abs(y), std::abs(x)));
  }

  float gammaCosineLaw = (powf(B, 2) - powf(A, 2) - powf(r, 2)) / (-2.0 * A * r);
  float zetaCosineLaw = (powf(r, 2) - powf(A, 2) - powf(B, 2)) / (-2.0 * A * B);

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


bool Arm::calculateArm(float *angles, float *coords) {
  float alpha = angles[0];
  float beta = angles[1];

  float alphaAbs = std::abs(alpha);
  float betaAbs = std::abs(beta);

  float alphaX = std::cos(toRadians(alphaAbs)) * A;
  float alphaY = std::sin(toRadians(alphaAbs)) * A;

  float betaX = std::cos(toRadians(betaAbs)) * B;
  float betaY = std::sin(toRadians(betaAbs)) * B;

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

bool Arm::pullBack() {
  float startAngles[2] = { (float) armBase->position(degrees) / GEAR_RATIO, (float) armMid->position(degrees) / GEAR_RATIO };
  float endAngles[2] = { startAngles[0], startAngles[1] };
  float coords[2] = {0, 0};


  if(!calculateArm(startAngles, coords)) {
    return false;
  }

  
  while(moveArm(startAngles, coords, endAngles)) {
    coords[0] -= 0.1;

    if(std::abs(endAngles[0]) > 60 || std::abs(endAngles[1]) > 60) {
      break;
    }
  }

  moveArm(startAngles, coords, endAngles);

  float distA = std::abs(endAngles[0] + startAngles[0]);
  float distB = std::abs(endAngles[1] + startAngles[1]);
  float distMax = fmax(distA, distB);

  float speedA = distA / distMax;
  float speedB = distB / distMax;

  armBase->setVelocity(speedA * ARM_SPEED, rpm);
  armBase->spinTo(endAngles[0] * GEAR_RATIO, degrees, false);

  armMid->setVelocity(speedB * ARM_SPEED, rpm);
  armMid->spinTo(endAngles[1] * GEAR_RATIO, degrees, false);

  return true;
}

void Arm::nextPosition() {
  if(currentPosition < (sizeof(positions) / sizeof(ArmPosition)) - 1) {
    currentPosition++;
  };
  setPosition(&positions[currentPosition]);
}

void Arm::previousPosition() {
  if(currentPosition > 0) {
    currentPosition--;
  }
  setPosition(&positions[currentPosition]);
}

void Arm::setPosition(ArmPosition *position) {
    armBase->spinTo(position->alpha, degrees, false);
    armMid->spinTo(position->beta, degrees, false);
}

float Arm::toDegrees(float radians) {
    return radians * 180.0 / M_PI;
}

float Arm::toRadians(float degrees) {
    return degrees * M_PI / 180.0;
}