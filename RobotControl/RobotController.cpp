/*
*  Class implementation of ur_rtde library @ https://sdurobotics.gitlab.io/ur_rtde/index.html
*  Created October 4th, 2022
*  Authored by Thomas Therkelsen and Simon Vinkel during the SICK Solution Hackathon
*  The showcase method is very hardcoded, and would in the future rely on vision for target acquisition.
*/

#include "RobotController.h"

RobotController::RobotController(const std::string robotIP, const std::string gripperIP)
        : rc(robotIP), rr(robotIP), gc(gripperIP, 63352, true) {      // rc and rr are shared pointers to created yet un-constructed objects

    std::cout << std::boolalpha;                                    // prints true/false instead of 1/0
    this->robotIP = robotIP;                                        // save robot ip
    this->gripperIP = gripperIP;                                    // save gripper ip
}

std::array<double, 3> RobotController::getTCP() {

    std::vector<double> TCP = rr.getActualTCPPose();    // Get TCP pose from rtde receive interface

    for (unsigned int i = 0; i < TCP.size(); ++i) {     // prints the TCP pose
        std::cout << TCP.at(i) << " ";
    }

    std::array<double, 3> TCPPose;                      // copies the TCP pose and converts to mm

    for (unsigned int i = 0; i < 3; ++i) {              // returns the converted TCP pose
        TCPPose.at(i) = TCP.at(i) * 1000;
    }

    return TCPPose;

}

void RobotController::moveHome() {
    rc.moveL(homePos);      // send robot to home position
}

void RobotController::moveUp(double amount, int axis) {         // takes amount in meters and axis (x,y,z = 1,2,3)
    std::vector<double> TCP = rr.getActualTCPPose();            // gets current TCP pose from rtde receive interface
    TCP.at(5) = 0;

    TCP.at(axis) = TCP.at(axis) + amount;                       // moves to the new desired TCP pose
    rc.moveL(TCP);
}

void RobotController::moveDown(double amount, int axis) {       // takes amount in meters and axis (x,y,z = 1,2,3)
    std::vector<double> TCP = rr.getActualTCPPose();            // gets current TCP pose from rtde receive interface
    TCP.at(5) = 0;

    TCP.at(axis) = TCP.at(axis) - amount;                       // moves to the new decired TCP pose
    rc.moveL(TCP);
}

void RobotController::stop(double deceleration) {
    rc.speedStop(deceleration);                                 // decelerates the robot joints
}

void RobotController::printGripperStatus(int status) {          // prints the gripper status
    switch (status) {
        case ur_rtde::RobotiqGripper::MOVING:
            std::cout << "moving";
            break;
        case ur_rtde::RobotiqGripper::STOPPED_OUTER_OBJECT:
            std::cout << "outer object detected";
            break;
        case ur_rtde::RobotiqGripper::STOPPED_INNER_OBJECT:
            std::cout << "inner object detected";
            break;
        case ur_rtde::RobotiqGripper::AT_DEST:
            std::cout << "at destination";
            break;
    }

    std::cout << std::endl;
}

void RobotController::showcase() {
    moveHome();     // Homing position is high enough, so it is not necessary to call moveUp
    gc.connect();
    gc.activate();

    std::vector<double> pillsStartUp{-0.267121, -0.5812, 0.109789, -0.0211262, -3.13361, 0.0143419};
    std::vector<double> pillsStartDown{-0.267121, -0.5812, -0.0784447, -0.0211262, -3.13361, 0.0143419};
    std::vector<double> pillsEndUp{0.28612, -0.570113, 0.109789, -0.0322568, -3.13987, -0.00253891};
    std::vector<double> pillsEndDown{0.28612, -0.570113, -0.0637024, -0.0322568, -3.13987, -0.00253891};
    std::vector<double> waterStartUp{-0.154038, -0.582338, 0.109789, -0.0211861, -3.13351, 0.0143568};
    std::vector<double> waterStartDown{-0.154038, -0.582338, -0.0696025, -0.0211861, -3.13351, 0.0143568};
    std::vector<double> waterEndUp{0.140664, -0.57017, 0.109789, -0.0322544, -3.13989, -0.00224767};
    std::vector<double> waterEndDown{0.140664, -0.57017, -0.063617, -0.0322544, -3.13989, -0.00224767};

    // Water moving
    rc.moveL(waterStartUp);
    gc.open(1, -1, ur_rtde::RobotiqGripper::START_MOVE);
    int status = gc.waitForMotionComplete();
    printStatus(status);
    rc.moveL(waterStartDown);
    gc.move(0.43, -1, ur_rtde::RobotiqGripper::START_MOVE);
    status = gc.waitForMotionComplete();
    printStatus(status);
    moveUp(0.10);
    rc.moveL(waterEndUp);
    rc.moveL(waterEndDown);
    gc.open(1, -1, ur_rtde::RobotiqGripper::START_MOVE);
    status = gc.waitForMotionComplete();
    printStatus(status);

    // Pills moving
    moveUp(0.10);
    rc.moveL(pillsStartUp);
    rc.moveL(pillsStartDown);
    gc.move(0.43, -1, ur_rtde::RobotiqGripper::START_MOVE);
    status = gc.waitForMotionComplete();
    printStatus(status);
    moveUp(0.10);
    rc.moveL(pillsEndUp);
    rc.moveL(pillsEndDown);
    gc.open(1, -1, ur_rtde::RobotiqGripper::START_MOVE);
    status = gc.waitForMotionComplete();
    printStatus(status);

    // End position
    moveUp(0.10);
    gc.close(1, -1, ur_rtde::RobotiqGripper::START_MOVE);
    status = gc.waitForMotionComplete();
    printStatus(status);
    moveHome();
}
