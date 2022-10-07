
#include "MotionControl.h"

using namespace std;
using namespace ur_rtde;

void printStatus(int Status) {
    switch (Status)
    {
        case RobotiqGripper::MOVING:
            std::cout << "moving";
            break;
        case RobotiqGripper::STOPPED_OUTER_OBJECT:
            std::cout << "outer object detected";
            break;
        case RobotiqGripper::STOPPED_INNER_OBJECT:
            std::cout << "inner object detected";
            break;
        case RobotiqGripper::AT_DEST:
            std::cout << "at destination";
            break;
    }

    std::cout << std::endl;
}

int main() {
    // Pose pills start: -0.267121 -0.5812 -0.0784447 -0.0211262 -3.13361 0.0143419
    // Pose water start: -0.154038 -0.582338 -0.0696025 -0.0211861 -3.13351 0.0143568
    // Pose pills end: 0.28612 -0.570113 -0.0637024 -0.0322568 -3.13987 -0.00253891
    // Pose water end: 0.140664 -0.57017 -0.063617 -0.0322544 -3.13989 -0.00224767

    MotionControl mc("192.168.56.10", "");
    //mc.moveHome();
    // mc.getTCP();
    mc.showcase();

//    std::cout << "Gripper test" << std::endl;
//    ur_rtde::RobotiqGripper gc("192.168.56.10", 63352, true);
//    gc.connect();
    //gc.activate();

//    gc.open(1, -1, RobotiqGripper::START_MOVE);
//    int status = gc.waitForMotionComplete();
//    printStatus(status);
//
//    gc.close(1, -1, RobotiqGripper::START_MOVE);
//    status = gc.waitForMotionComplete();
//    printStatus(status);
//
//    gc.move(0.65, -1, RobotiqGripper::START_MOVE);
//    status = gc.waitForMotionComplete();
//    printStatus(status);







//    std::cout << "Gripper test" << std::endl;
//    ur_rtde::RobotiqGripper gc("192.168.56.10", 63352, true);
//    gc.connect();
//
//
//    // Test setting of position units and conversion of position values
//    gc.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
//    std::cout << "OpenPosition: " << gc.getOpenPosition() << "  ClosedPosition: " << gc.getClosedPosition()
//              << std::endl;
//    gc.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_NORMALIZED);
//    std::cout << "OpenPosition: " << gc.getOpenPosition() << "  ClosedPosition: " << gc.getClosedPosition()
//              << std::endl;
//
//    // Test of move functionality with normalized values (0.0 - 1.0)
//    int status = gc.move(1, 1, 0, RobotiqGripper::WAIT_FINISHED);
//    mc.printStatus(status);
//    status = gc.move(0.0, 1, 0, RobotiqGripper::WAIT_FINISHED);
//    mc.printStatus(status);
//
//    // We preset force and speed so we don't need to pass it to the following move functions
//    gc.setForce(0.0);
//    gc.setSpeed(0.5);
//
//    // We switch the position unit the mm and define the position range of our gc
//    gc.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_MM);
//    gc.setPositionRange_mm(40);
//    std::cout << "OpenPosition: " << gc.getOpenPosition() << "  ClosedPosition: " << gc.getClosedPosition()
//              << std::endl;
//    gc.move(50);
//    status = gc.waitForMotionComplete();
//    mc.printStatus(status);
//
//    gc.move(10);
//    status = gc.waitForMotionComplete();
//    mc.printStatus(status);
//
//    std::cout << "moving to open position" << std::endl;
//    status = gc.open();
//    status = gc.waitForMotionComplete();
//    mc.printStatus(status);
//
//    // Test async move - start move and then wait for completion
//    gc.close(0.02, 0, RobotiqGripper::START_MOVE);
//    status = gc.waitForMotionComplete();
//    mc.printStatus(status);
//
//    status = gc.open(1.0, 0.0, RobotiqGripper::WAIT_FINISHED);
//    mc.printStatus(status);
//
//    gc.setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_DEVICE);
//    gc.setUnit(RobotiqGripper::SPEED, RobotiqGripper::UNIT_DEVICE);
//    gc.setUnit(RobotiqGripper::FORCE, RobotiqGripper::UNIT_DEVICE);
//
//    std::cout << "OpenPosition: " << gc.getOpenPosition() << "  ClosedPosition: " << gc.getClosedPosition()
//              << std::endl;
//
//    gc.move(255, 5, 0);
//    std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    while (RobotiqGripper::MOVING == gc.objectDetectionStatus())
//    {
//        std::cout << "waiting..." << std::endl;
//        std::this_thread::sleep_for(std::chrono::milliseconds(100));
//    }
//    mc.printStatus(gc.objectDetectionStatus());
//
//    std::cout << "disconnecting" << std::endl;
//    gc.disconnect();
    return 0;
}