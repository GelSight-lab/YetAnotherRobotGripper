#include "gripper_interface.hpp"

GripperInterface gripper;

void setup() {
  DEBUG_SERIAL.begin(57600); 
  gripper.setup();
}

void loop() {
  // gripper.step_once();
}
