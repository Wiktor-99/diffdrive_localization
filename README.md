# Diff Drive Robot Localization

A simple project demonstrating the robot localization package and EKF (Extended Kalman Filter) node.

## Launching the Project

### Dev Containers Setup
This project is based on a development container. To run it, you will need to install [Docker](https://docs.docker.com/get-docker/) and Visual Studio Code with the Dev Containers plugin. You can find more information about the dev containers [here](https://navigation.ros.org/development_guides/devcontainer_docs/index.html). Once the environment is ready, open the project folder in Visual Studio Code, press `Shift` + `Ctrl` + `P`, and type `Dev Containers: Reopen in Container` and press enter.

### Launching ROS 2 Packages

Open two terminals in **Visual Studio Code**. In the first terminal, build, source, and execute the launch bringup with the following commands:

```bash
colcon build && source install/setup.bash
ros2 launch diffdrive_bringup diffdrive_bringup.launch.py
```

In the second terminal, launch the teleop node with the following command:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Click the play button in Gazebo, and you can now interact with the diff drive robot.

## Remarks
The repository contains two branches: on the `main` branch, the diff drive controller is set up with the `ros2_control` plugin, while on the `gazebo_diff_drive` branch, it is set up with the Gazebo plugin.
