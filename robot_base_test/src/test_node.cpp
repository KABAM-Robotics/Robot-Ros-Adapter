#include <robot_base.h>
// Minimal node using the base class directly
int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_base_test_node");
    ros::NodeHandle nh;

    RobotBase robot;  // instantiate base class directly

    ros::spin();
    return 0;
}
