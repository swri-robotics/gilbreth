# Gilbreth Simulation Application
## Getting Started
- Follow the install instructions in the [README](README.md)
- Make sure that the moveit commander dependency is installed. The rosdep step should fetch this dependency

## Application
### Application Bringup
- Bring up the application components
  ```
  roslaunch gilbreth_application application_setup.launch
  ```
  - Both Gazebo and Rviz will be launched, wait until both of these application have started before moving on 
    to the next step. 

  * (Optional) Use Rviz to test the robot controller by dragging the end-effector to another pose and click 'plan and execute' in moveit panel.
    If the robot failed to move to the target pose, then the robot controller is not working. 
    Close Gazebo and Rviz and then relaunch the environment.

### Application Run

- Start the application
  ```
  roslaunch gilbreth_application application_run.launch
  ```
  - The application will begin by moving the robot arm to its home position.
  - The conveyor will begin moving and parts will appear at random intervals.
  - This step will also bring up the individual nodes for tool planning, trajectory planning, 
    perception and robot execution, each in its own terminal window (don't close the windows).

### Launch node seperately (Outdated):

The script will run object_recognition, tool_planning, trajectory_planning and robot_execution nodes seperately.

- object segmentation and object recognition:
```
roslaunch gilbreth_perception gilbreth_perception.launch
```
- robot tool planner:
```
rosrun gilbreth_grasp_planning tool_planner.py
```
- robot trajectory planner:
```
rosrun gilbreth_grasp_planning trajectory_planner.py
```
- robot execution:
```
rosrun gilbreth_grasp_planning robot_execution.py
```
