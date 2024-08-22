# ROS2 Demo

The ROS2 Demo demonstrates the functionality of the Face Detector library as a ROS2 package. It uses videoframes from your computer's webcam and tries to detect a face, get the Region of Interest as well as key landmarks. 

## Building and compiling

Build the package with more verbose output

```bash
colcon build --event-handlers console_direct+ --packages-select face_detector
```

Note that, whenever build errors occur, and you need to clean the build, use

```bash
rm -rf build install log
-rf ~/.ros
```

## Run the demo

Before you can use any of the installed executables or libraries, you will need to add them to your path and library paths. 
colcon will have generated bash/bat files in the install directory to help set up the environment. 
These files will add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.

```bash
source install/setup.bash
```

With the environment sourced, we can run executables built by colcon. Let’s run the camera node from the examples:

```bash
ros2 run face_detector camera_node
```

In another terminal, let’s run the face detector node (don’t forget to source the setup script):

```bash
ros2 run face_detector face_detector_node
```

In yet another terminal, let’s run the viewer node:

```bash
ros2 run face_detector face_detector_viewer
```

Now, you should see the camera stream, with annotations produced by the face detector.

