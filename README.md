# Sampling Kinematics Plugin
![CI](https://github.com/marip8/sampling_kinematics_plugin/workflows/CI/badge.svg?branch=master)

A MoveIt! kinematics plugin for solving IK with robots mounted on external axes

## Motivation
Several types of robotic manipulators have closed-form kinematic solutions that are computationally faster than more generic Jacobian-based solvers.
A number of MoveIt! plugins exist that leverage these fast kinematic solutions ([ikfast](http://docs.ros.org/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html), [moveit_opw_kinematics](https://github.com/JeroenDM/moveit_opw_kinematics_plugin), [ur_kinematics](https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_kinematics)).
However, these plugins cannot be used to solve kinematics when the robot is mounted on an external axis because they only provide joint solutions for the manipulator.
To obtain a full system kinematic solution, it is possible to discretely sample the joint space of the external axes and then solve the manipulator kinematics at each joint position. This project implements this approach by sampling the joint space of external axes in a planning group and then determining the manipulator joint state using a configurable, manipulator-specific MoveIt! kinematics solver plugin.

## Usage
This plugin requires two planning groups to be defined in the SRDF
  1. A planning group for the manipulator alone (i.e. does not include the external axis joints)
  2. A planning group which includes the manipulator and external axis joints

This plugin requires specific parameters in the MoveIt! `kinematics.yaml`:
  - `kinematics_solver_search_resolution`: The resolution at which the joint range of the external axis should be discretized when solving IK
  - `robot_group`: The name of the planning group that represents only the manipulator joints
  - `weights`: (Optional) A vector of joint weights. These weights are applied to joints individually to influence the overall cost of the joint pose solution. The size of this vector must match the number of joints in the planning group

An example configuration is shown below.

```yaml
ext_axis_manipulator:
  kinematics_solver: sampling_kinematics_plugin/ExternalAxisSamplingKinematicsPlugin
  kinematics_solver_search_resolution: 0.1
  robot_group: manipulator
  weights: [0.1, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
manipulator:
  kinematics_solver: ur_kinematics/UR5KinematicsPlugin
  kinematics_solver_search_resolution: 0.1
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 1
```

> Note: The sampling kinematics plugin does not currently utilize the `kinematics_solver_timeout` or `kinematics_solver_attempts` parameters
