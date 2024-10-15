# ME 449 Capstone Project: Controller for youBot Mobile Manipulator

**Author**: Christopher Luey

## Overview

This project develops a control system for the youBot mobile manipulator, which includes a 5R joint robotic arm mounted on a base with 4 mecanum wheels. The objective is to pick up and place a cube at predefined initial and final positions. The project integrates key techniques from **control theory**, including **PID control**, **trajectory generation**, and **Jacobian-based kinematics**, to ensure precise and smooth motion. The output includes simulation videos and error plots that illustrate the performance of the control strategies.

## Control Theory Concepts

### PID Control

PID (Proportional-Integral-Derivative) control is a cornerstone of control theory, widely used to manage the behavior of dynamic systems like robotic manipulators. In this project, the PID controller is employed to ensure the manipulator accurately follows the desired trajectory by adjusting its motion in real-time.

1. **Proportional Control (P)**: Proportional control applies a corrective action proportional to the current error between the desired and actual position. It quickly reduces the error but can lead to steady-state error if used alone.

2. **Integral Control (I)**: The integral term accumulates the error over time, ensuring that small, persistent errors are corrected. This helps eliminate steady-state error and ensures the manipulator reaches its target position.

3. **Derivative Control (D)**: The derivative term acts as a predictor of future error by observing the rate of change in the error. This helps dampen oscillations and prevents overshooting, making the controller more stable.

By tuning the **Kp**, **Ki**, and **Kd** gains, the controller is optimized for different tasks, balancing fast response, precision, and stability.

### Trajectory Generation

**Trajectory generation** is a critical aspect of robotic control, where the desired path that the manipulator follows is pre-planned. The trajectory defines how the end-effector of the robot moves from the initial to the final configuration in both time and space.

- In this project, the trajectory generation process calculates smooth paths for the manipulator's joints and base, ensuring the robot can move through the workspace without abrupt or jerky movements.
- The generated trajectory specifies the positions, velocities, and accelerations at each time step. The control system then follows this planned trajectory, correcting any deviations in real-time using feedback control.

### Odometry and Feedback Control

Odometry is used to estimate the robot’s position based on the motion of its wheels. The robot’s sensors provide continuous feedback about its actual position, which is compared to the desired position. The **feedback control** system, powered by PID control, adjusts the motor commands to reduce the error between the actual and desired positions in real time.

### Jacobian and Kinematics

The **Jacobian matrix** plays a vital role in mapping the relationship between joint velocities and the end-effector's velocity in Cartesian space. In kinematic control, the Jacobian matrix is used to calculate how much each joint needs to move to achieve a desired end-effector motion. 

- For a manipulator like youBot, the Jacobian accounts for both the arm and the base motion, converting joint space velocities into end-effector velocities in Cartesian space.
- The **pseudo-inverse of the Jacobian** is used in this project to ensure the robot moves in a desired manner, even in cases where the Jacobian may be singular or near-singular. Singular configurations occur when the manipulator's end-effector cannot move in certain directions, leading to infinite velocities in the inverse kinematic solutions. Singularity avoidance is implemented by setting near-singular Jacobian values to zero, ensuring stable and smooth control.

## Project Structure

### Code Execution

- To generate the controls, run the following scripts:
  - `best.py`
  - `overshoot.py`
  - `newTask.py`

  Each script generates relevant graphs, logs, and CSV files that are saved in the results directory.

- **Configuration Files**: The following configuration files adjust input parameters such as gains and initial conditions:
  - `best_config.py`
  - `overshoot_config.py`
  - `newTask_config.py`

### Key Functions by Milestone

- `Milestone1.py`: Contains the `NextStep` function for basic control.
- `Milestone2.py`: Implements `TrajectoryGeneration` for path planning.
- `Milestone3.py`: Introduces `FeedbackControl` for refining the control process.
  
### Utilities

- `util.py`: Includes functions for generating error vs. time graphs.

## Results Directory

### Best Configuration

This setup uses a tuned feedforward + P controller, tested for specific cube positions. The controller parameters, initial conditions, and resulting error reduction are detailed in the results folder.

### Overshoot Configuration

This setup employs a PI controller, which leads to overshoot and oscillation in the error, with specific gains and initial conditions affecting the control performance.

### New Task Configuration

This setup features a PI controller for a new task with different cube positions and gains. It demonstrates the controller's ability to converge to zero error, achieving precise pickup and placement of the cube.

## Enhancements

### Singularity Avoidance

When enabled, the code includes Jacobian singularity avoidance, ensuring smooth control by detecting near-singular configurations and preventing large velocity spikes. This enhancement results in smoother motion, particularly in the "best" configuration setup.

