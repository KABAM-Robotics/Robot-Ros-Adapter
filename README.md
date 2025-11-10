# RobotBase ROS1 Package

This repository provides an abstract base class `RobotBase` for ROS1 robot nodes. It simplifies the setup of common interfaces such as goals, teleoperation commands, battery, map, and application state.

---

## **Usage**

1. Clone this repository into your catkin workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/robot_base.git
```

2. Include the header in your package:
#include <robot_base.h>

3. Derive your own node class from RobotBase and override callbacks as needed.





