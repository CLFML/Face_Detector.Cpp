# Set-up build environment on Linux

Linux is one of the easiest os'es to set-up as most packages and libraries can be found in the package repositories.

## Ubuntu and Debian

We currently support Ubuntu Noble (24.04) 64-bit x86 and 64-bit ARM.

Use [this guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html) to install ROS 2 Jazzy Jalisco on your system.

Make sure ros cv bridge is installed as well,

```bash
sudo apt install ros-jazzy-cv-bridge
```

If you have not installed VSCode yet, do not install using APT in ubuntu as it will install the sandboxed snap version.

**Which has many issues due to the sandbox environment**

Use [this guide](https://code.visualstudio.com/docs/setup/linux) instead, which installs it using the APT repository from Microsoft themselves.


## Compiling and running the example

The library contains a ROS2 package example demonstrating the usage and functionality of this library. 

To compile and run this example:

1. Clone this repo:
```
git clone https://github.com/CLFML/Face_Detector.Cpp.git
```

2. Open the cloned repo folder in a terminal

Build the package with more verbose output

```bash
colcon build --event-handlers console_direct+ --packages-select face_detector
```

Note that, whenever build errors occur, and you need to clean the build, use

```bash
rm -rf build install log
rm -rf ~/.ros
```

3. Source your ROS2 installation:

```bash
source /opt/ros/jazzy/setup.bash
```

4. Set up the environment

```bash
source install/setup.bash
```

5. With the environment sourced, we can run executables built by colcon. Let’s run the camera node from the examples:

```bash
ros2 run face_detector camera_node
```

5. In another terminal, let’s run the face detector node (don’t forget to source the setup script):6
```bash
ros2 run face_detector face_detector_node
```

7. In yet another terminal, let’s run the viewer node:

```bash
ros2 run face_detector face_detector_viewer
```

Now, you should see the camera stream, with annotations produced by the face detector.

