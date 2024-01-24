#pragma once

#define DEBUG_SERIAL Serial

#define DXL_PROTOCOL_VERSION 2.0
#define DXL_BAUDRATE 57600
#define DXL_CTRL_ID 1
#define DXL_GRIPPER_ID 100

// Assumes positive means open, negative means close
constexpr float HOMING_CURRENT = 15;