std_msgs/Header header
trajectory_msgs/JointTrajectory[] trajectories  # Array of trajectories to be executed by the robot
duration[] trajectory_delays                    # Time delays before each trajectory.  Use 0 for no delay
