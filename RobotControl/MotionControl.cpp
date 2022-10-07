/*
*  Class implementation of ur_rtde library @ https://sdurobotics.gitlab.io/ur_rtde/index.html
*  Created October 4th, 2022
*  Authored by Thomas Therkelsen and Simon Vinkel @ SDU
*/

#include "MotionControl.h"

MotionControl::MotionControl(const std::string robotIP, const std::string gripperIP)
        :rc(robotIP), rr(robotIP), gc("192.168.56.10", 63352, true){  // rc and rr are objects that have not yet been constructed, this calls all the constructors at the same time
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

void MotionControl::printStatus(int status) {
    switch (status)
    {
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

void MotionControl::showcase() {
    // Homing position is high enough, so it is not necessary to call moveup
    moveHome();
    gc.connect();
    gc.activate();
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::vector<double> pillsStartUp {-0.267121, -0.5812, 0.109789, -0.0211262, -3.13361, 0.0143419};
    std::vector<double> pillsStartDown {-0.267121, -0.5812, -0.0784447, -0.0211262, -3.13361, 0.0143419};
    std::vector<double> pillsEndUp {0.28612, -0.570113, 0.109789, -0.0322568, -3.13987, -0.00253891};
    std::vector<double> pillsEndDown {0.28612, -0.570113, -0.0637024, -0.0322568, -3.13987, -0.00253891};
    std::vector<double> waterStartUp {-0.154038, -0.582338, 0.109789, -0.0211861, -3.13351, 0.0143568};
    std::vector<double> waterStartDown {-0.154038, -0.582338, -0.0696025, -0.0211861, -3.13351, 0.0143568};
    std::vector<double> waterEndUp {0.140664, -0.57017, 0.109789, -0.0322544, -3.13989, -0.00224767};
    std::vector<double> waterEndDown {0.140664, -0.57017, -0.063617, -0.0322544, -3.13989, -0.00224767};

//    std::thread t1(&gripper::releaseObject, *m_gripper, time);
//    m_control->speedJ(jacobian::eig2Vec(qp_k), max_acc, time);
//    std::this_thread::sleep_for(std::chrono::duration<double>(time));
//    m_control->speedStop(30);
//    t1.join();

    // Water moving
    rc.moveL(waterStartUp);
    //std::this_thread::sleep_for(std::chrono::milliseconds(3000));
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

//void MotionControl::gripperConnect(std::string ip) {
//    std::cout << "Gripper test" << std::endl;
//    ur_rtde::RobotiqGripper gc("192.168.56.10", 63352, true);
//    gc.connect();
//
//    // Test emergency release functionality
//    if (!gc.isActive())
//    {
//        gc.emergencyRelease(RobotiqGripper::OPEN);
//    }
//    std::cout << "Fault status: 0x" << std::hex << gc.faultStatus() << std::dec << std::endl;
//    std::cout << "activating gc" << std::endl;
//    gc.activate();
//}
