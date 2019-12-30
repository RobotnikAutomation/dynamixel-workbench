# TORSO INTERFACE

This component creates a simpler interface through topics to the torso controller.

---

## Dependencies

- robotnik_msgs [🔗](https://github.com/RobotnikAutomation/robotnik_msgs/)

```bash
git clone https://github.com/RobotnikAutomation/robotnik_msgs/
```

- rcomponent [🔗](https://github.com/RobotnikAutomation/rcomponent/)

```bash
git clone https://github.com/RobotnikAutomation/rcomponent/
```

---

## ROS

### Params

- **example (type, default: X)**: explanation.

#### Services

- **~/set_elevator_raised (std_srvs/SetBool)**: Used to set the the elevator raised or not
- **~/set_elevator (robotnik_msgs::set_float_value)**: Used to set the position of the elevator
- **~/set_pan (robotnik_msgs::set_float_value)**: Used to set the pan position of the head
- **~/set_tilt (robotnik_msgs::set_float_value)**: Used to set the tilt position of the head

#### Services Called

- **torso_controllers/joint_trajectory (trajectory_msgs/JointTrajectory)**: Used to set the state of the torso