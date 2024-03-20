
#include "vex.h"
#include "Robot.h"

using namespace vex;

brain robotBrain;
competition robotCompetition;
controller robotController = controller(primary);

motor leftDrive = motor(PORT12, ratio18_1, false);
motor rightDrive = motor(PORT11, ratio18_1, true);

motor armBase = motor(PORT9, ratio18_1, true);
motor armMid = motor(PORT8, ratio18_1, false);

motor claw = motor(PORT10, ratio18_1, false);
motor tray = motor(PORT2, ratio18_1, false);

Robot robot = Robot(&robotBrain, &robotCompetition, &robotController, &leftDrive, &rightDrive, &armBase, &armMid, &claw, &tray);

void loop(void) {
  while (1) {
    robot.drive();
  }
}

int main() {
  robotCompetition.drivercontrol(loop); 

  while (true) {
    wait(100, msec);
  }
}