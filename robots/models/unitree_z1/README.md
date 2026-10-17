# Unitree Z1 (MJCF)

The MuJoCo model of the [Z1 manipulator](https://www.unitree.com/z1/) of
[Unitree Robotics](https://www.unitree.com/), with and without its gripper, used by
`UnitreeZ1Robot::description()`.

## Source

Copied from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_z1),
commit `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35`: `z1.xml`, `z1_gripper.xml`, `assets/` and
`LICENSE`. The Menagerie derived it from the
[URDF of Unitree](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/z1_description).

## Changes

- `z1.xml` and `z1_gripper.xml`: added the site `end_effector` to `link06`, at the flange
  (`z1.xml`) or at the tip of the gripper (`z1_gripper.xml`), with its z axis along the last
  joint, so that its pose is the forward kinematics of `UnitreeZ1Robot::kinematics()`.
- `z1.xml` and `z1_gripper.xml`: added `armature="0.01"` to the joints (the inertia of the geared
  motors). Without it, with the timestep of 0.002 s, the stiff position actuators (gain 1000) of the
  light joints chatter instead of moving: joint6, joint5 without the gripper, and jointGripper,
  which never opened.

## License

These files are released under the [BSD-3-Clause License](LICENSE), Copyright (c) 2016-2022
HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics"). This applies to the files in this
directory only; the rest of cpp-interface-mujoco is under the LGPL-3.0-or-later.
