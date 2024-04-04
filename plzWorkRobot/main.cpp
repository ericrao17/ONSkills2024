
#include "vex.h"
#include "Robot.h"

using namespace vex;

brain robotBrain;
competition robotCompetition;
controller robotController = controller(primary);

motor leftDrive = motor(PORT6, ratio18_1, false);
motor rightDrive = motor(PORT7, ratio18_1, true);

motor armBase = motor(PORT9, ratio18_1, true);

motor claw = motor(PORT10, ratio18_1, false);
motor snowPile = motor(PORT2, ratio18_1, true);

Robot robot = Robot(&robotBrain, &robotCompetition, &robotController, &leftDrive, &rightDrive, &armBase, &claw, &snowPile);

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