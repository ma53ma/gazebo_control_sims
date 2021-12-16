# gazebo_control_sims

## Required packages to catkin_make:
[Link](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/) - Use the melodic devel

    git clone -b melodic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

### Turtlebot3_rviz
    roslaunch gazebo_control_sims turtlebot3_rviz_robot.launch

### Spline trajectory following
Run turtlebot3_rviz first, then...

    rosrun gazebo_control_sims plot_points_rviz.py