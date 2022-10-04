/*
*  Class implementation of ur_rtde library @ https://sdurobotics.gitlab.io/ur_rtde/index.html
*  Created October 4th, 2022
*  Authored by Thomas Therkelsen @ SDU
*/

#include "MotionControl.h"

MotionControl::MotionControl(const std::string robotIP, const std::string gripperIP)
        :rc(robotIP), rr(robotIP){  // rc and rr are objects that have not yet been constructed, this calls all the constructors at the same time
    std::cout << std::boolalpha;    // prints true/false instead of 1/0
    this->robotIP = robotIP;        // save robot ip
    this->gripperIP = gripperIP;    // save gripper ip
}

std::array<double, 3> MotionControl::getTCP() {
    // Get TCP pose from rtde receive interface
    std::vector<double> TCP = rr.getActualTCPPose();

    // prints the TCP pose
    for (unsigned int i = 0; i < TCP.size(); ++i) {
        std::cout << TCP.at(i) << " ";
    }

    // copies the TCP pose and converts to mm
    std::array<double, 3> TCPPose;

    for (unsigned int i = 0; i < 3; ++i) {
        TCPPose.at(i) = TCP.at(i) * 1000;
    }

    // returns the converted TCP pose
    return TCPPose;

}

void MotionControl::moveHome() {
    // send robot to home position
    rc.moveL(homePos);
}

void MotionControl::moveUp(double amount) {
    // gets current TCP pose from rtde receive interface
    std::vector<double> TCP = rr.getActualTCPPose();
    TCP.at(5) = 0;

    TCP.at(2) = TCP.at(2) + amount;
    // moves to the new decired TCP pose
    rc.moveL(TCP);
}

void MotionControl::moveDown(double amount) {
    // gets current TCP pose from rtde receive interface
    std::vector<double> TCP = rr.getActualTCPPose();
    TCP.at(5) = 0;

    TCP.at(2) = TCP.at(2) - amount;
    // moves to the new decired TCP pose
    rc.moveL(TCP);
}

void MotionControl::stop(double deceleration) {
    rc.speedStop(deceleration);
}