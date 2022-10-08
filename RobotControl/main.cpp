
#include "RobotController.h"

using namespace std;
using namespace ur_rtde;

int main() {
    MotionControl mc("192.168.56.10", "192.168.56.10");
    mc.showcase();

    return 0;
}