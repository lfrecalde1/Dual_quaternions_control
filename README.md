# Dual Quaternions Control

This repository contains the control of rigid bodies based on Dual Quaternions.
<p float="left">
    <img src="planning_fast.gif" width="600"  />
 </p>

## Run Differential Kinematics Control Based on Dual Quaternions

Inside the folder Pose_Kinematic_Dual_Quaternion, execute the following script:

```bash
Dual_quaternion_kinematics_control.m
```

## Run Dynamics Control Based on Dual Quaternions
Inside the folder Pose_dynamics_Dual_Quaternion_Control execute the following script:
```bash
Dynamics_Rigid_Body.m
```
## Run NMPC using Acados
Inside the folder scripts execute the following script:
```bash
roscore
main.py
```
## Requirements
The NMPC has the following requirements
```bash
pip install numpy
pip install scipy==1.7.2
pip install pyyaml
pip install -U rospkg
pip install empy
pip install casadi==3.5.1
pip install rospy==1.16.0
acados
latex
```
