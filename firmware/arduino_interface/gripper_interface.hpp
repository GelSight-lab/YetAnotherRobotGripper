#include "consts.hpp"
#include <Dynamixel2Arduino.h>

#define CMP_RESOLUTION 0.01

// OpenRB-150 related defs
#define DXL_SERIAL Serial1
const int DXL_DIR_PIN = -1;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

class GripperInterface {
private:
    const uint8_t ctrlID;
    const uint8_t motorID;
    Dynamixel2Arduino* dxl;
    const float protocol_version;
    const long baudRate;
    bool homed;

public:
    GripperInterface(
            const uint8_t ctrlID,
            const uint8_t motorID, 
            const float protocol_version,
            const long baudRate) : 
            ctrlID(ctrlID), motorID(motorID), 
            protocol_version(protocol_version),
            baudRate(baudRate),
            homed(false) {
        dxl = new Dynamixel2Arduino(DXL_SERIAL, DXL_DIR_PIN);
    }

    GripperInterface() : 
        ctrlID(DXL_CTRL_ID), 
        motorID(DXL_GRIPPER_ID), 
        protocol_version(DXL_PROTOCOL_VERSION),
        baudRate(DXL_BAUDRATE),
        homed(false) { }

    void setup(unsigned long timeout=1000) {
        dxl->begin(baudRate);
        dxl->setPortProtocolVersion(protocol_version);
        // Get DYNAMIXEL information
        const unsigned long pingTime = millis();
        while (!dxl->ping(ctrlID) || !dxl->ping(motorID)) {
            DEBUG_SERIAL.println("DXL ping failed. Retrying...");
            delay(100);
        }
        disarm(true, true);
        home_gripper();
    }

    static float clipGoalDegree(const float currentDeg, const float goalDeg) {
        if (fabs(currentDeg - goalDeg) < 360.001) {
            return goalDeg;
        } else {
            float sign = goalDeg > currentDeg ? -1 : 1;
            return clipGoalDegree(currentDeg, goalDeg + sign * 360.);
        }
    }
    
    void disarm(bool ctrl, bool motor) {
        if (ctrl) {dxl->torqueOff(ctrlID);}
        if (motor) {dxl->torqueOff(motorID);}
    }

    void arm(bool ctrl, bool motor) {
        if (ctrl) {dxl->torqueOn(ctrlID);}
        if (motor) {dxl->torqueOn(motorID);}
    }

    float getPresentCurrent(bool isCtrl) {
        return dxl->getPresentCurrent(isCtrl ? ctrlID : motorID);
    }

    float getPresentPosition(bool isCtrl) {
        return dxl->getPresentPosition(isCtrl ? ctrlID : motorID, UNIT_DEGREE);
    }

    void setControlMode(bool ctrl, bool motor, uint8_t mode) {
        disarm(ctrl, motor);
        if (ctrl) {
            dxl->setOperatingMode(ctrlID, mode);
        }
        if (motor) {
            dxl->setOperatingMode(motorID, mode);
        }
    }

    void setCbPControlMode(bool ctrl, bool motor) {
        setControlMode(ctrl, motor, OP_CURRENT_BASED_POSITION);
    }

    void setGoalCurrent(bool isCtrl, float val) {
        const uint8_t id = isCtrl ? ctrlID :motorID;
        dxl->setGoalCurrent(id, val);
    }

    void setGoalPosition(bool isCtrl, float val) {
        const uint8_t id = isCtrl ? ctrlID :motorID;
        const float clippedVal = clipGoalDegree(
            getPresentPosition(isCtrl), val);
        dxl->setGoalPosition(id, clippedVal, UNIT_DEGREE);
    }

    void home_gripper() {
        disarm(true, true);
        dxl->setOperatingMode(motorID, OP_CURRENT);
        setGoalCurrent(false, 0);
        
        // Open the gripper to the limit first.
        arm(/*ctrl*/false, /*motor=*/true);
        setGoalCurrent(false, HOMING_CURRENT);
        delay(2000);
        setGoalCurrent(false, 0);
        disarm(true, true);

    }
};
