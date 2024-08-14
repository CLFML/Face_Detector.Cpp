# Face_detector.Cpp
Face detector using the [BlazeFace Mediapipe model](https://storage.googleapis.com/mediapipe-assets/MediaPipe%20BlazeFace%20Model%20Card%20(Short%20Range).pdf) (with both CPU and TPU delegates) written in C++;

- Plain C/C++ implementation with minimal dependencies (Tensorflow Lite + OpenCV)
- Google MediaPipe models without the Mediapipe framework
- Support for Coral Edge TPU
- Runs on ARM as well (Tested on RPI 3,4 and 5)

## API Features
This library offers support for:
- Face detection
- Face ROI (Region of interest) detection
- 2D Landmarks of the Left eye, Right Eye, Nose tip, Mouth, Left eye tragion and Right eye tragion


### Face detection
This is some example code for face detection:
```c++
    #define MODEL_PATH FACE_DETECTOR_MODEL_DIR "/CPU/face_detection.tflite"
    ...

    /* Initialize face detector library */
    CLFML::FaceDetection::FaceDetector det;
    
    /* Load model and initialize inference runtime */
    det.load_model(MODEL_PATH);
    
    /* Load image into model and do inference! */
    det.load_image(cam_frame);

    /* Get face_detected value */
    const int face_detected = det.detected(); // returns -1 for no face and 0 for face detected!
```

### Region Of Interest
This is some example code for capturing the region of interest:
```c++
    #define MODEL_PATH FACE_DETECTOR_MODEL_DIR "/CPU/face_detection.tflite"
    ...

    /* Initialize face detector library */
    CLFML::FaceDetection::FaceDetector det;
    
    /* Load model and initialize inference runtime */
    det.load_model(MODEL_PATH);
    
    /* Load image into model and do inference! */
    det.load_image(cam_frame);

    /* Get the face roi rectangle */
    cv::Rect face_roi = det.get_face_roi();
```

### Example code
For a full example showcasing both these API functions see the example code in [example/face_roi_demo/demo.cpp](example/face_roi_demo/demo.cpp).

## Building with CMake
Before using this library you will need the following packages installed:
- OpenCV
- Working C++ compiler (GCC, Clang, MSVC (2017 or Higher))
- CMake
- Ninja (**Optional**, but preferred)

### Running the examples (CPU)
1. Clone this repo
2. Run:
```bash
cmake . -B build -G Ninja
```
3. Let CMake generate and run:
```bash
cd build && ninja
```
4. After building you can run (linux & mac):
```bash
./face_roi_demo
```
or (if using windows)
```bat
face_roi_demo.exe
```
### Running the examples (Coral TPU)
0. Make sure you got:
- The newest libedgetpu library installed (based on TF 2.16.1); If not sure compile it from [source](https://github.com/google-coral/libedgetpu)!
- libusb-1.0-0-dev package or installed (only when using Linux)\
- When using Linux your udev configured correctly (is done automatically when compiling from source!)
1. Clone this repo
2. Run:
```bash
cmake . -B build -G Ninja -DFACE_DETECTOR_ENABLE_CORAL_TPU=ON
```
or add this block to your settings.json in vscode:
```json
"cmake.configureSettings": {
    "FACE_DETECTOR_ENABLE_CORAL_TPU": "ON"
}
```

3. Let CMake generate and run:
```bash
cd build && ninja
```
4. After building you can run (linux & mac):
```bash
./face_roi_demo
```
or (if using windows)
```bat
face_roi_demo.exe
```

### Using it in your project as library
Add this to your top-level CMakeLists file:
```cmake
include(FetchContent)

FetchContent_Declare(
    face_detector
    GIT_REPOSITORY https://github.com/CLFML/Face_Detector.Cpp
    GIT_TAG main
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/Face_Detector.Cpp
)
FetchContent_MakeAvailable(face_detector)
...
target_link_libraries(YOUR_EXECUTABLE CLFML::face_detector)
```
Or manually clone this repo and add the library to your project using:
```cmake
add_subdirectory(Face_Detector.Cpp)
...
target_link_libraries(YOUR_EXECUTABLE CLFML::face_detector)
```

## Building a ROS2 package with Colcon
Before using this library you will need the following packages installed:
- OpenCV
- ROS2
- ROS CV bridge
- Working C++ compiler (GCC, Clang, MSVC (2017 or Higher))
- CMake

### Running the examples (Ubuntu, CPU)

1. Clone this repo:
```
git clone https://github.com/CLFML/Face_Detector.Cpp.git
```

2. Make sure to set the ROS2 switch in CMakeLists.txt:

```cmake
option(CLFML_ROS2_PACKAGE_BUILD "Build a ROS2 package" ON)
```

You may need to explicitly override the cache:
```cmake
set(CLFML_ROS2_PACKAGE_BUILD ON CACHE BOOL "Build a ROS2 package" FORCE)
```

3. Build the package:

```bash
colcon build --packages-select face_detector
```

4. Set up the environment:

```bash
source install/setup.bash
```

5. Run the camera node:

```bash
ros2 run face_detector camera_node
```

6. In another terminal, run the face detector node (don’t forget to source the setup script):

```bash
ros2 run face_detector face_detector_node
```

7. In yet another terminal, run the viewer:

```bash
ros2 run face_detector face_detector_viewer
```



## Aditional documentation
See our wiki!

## Todo
- Add language bindings for Python, C# and Java
- Add support for Conan and precompiled libraries (such as precompiled version of TensorFlow Lite)
- Add support for MakeFiles and Bazel
- Add Unit-tests 

## License
This work is licensed under the Apache 2.0 license.