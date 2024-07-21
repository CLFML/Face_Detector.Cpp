# Overview
The library offers a relatively simple abstraction (compared to the MediaPipe framework) for use with the BlazeFace Mediapipe model. The code is directly built up on the TensorFlow Lite library and uses basic OpenCV functions to preprocess videoframe's and postprocess the results from the model.

## API
The library API consists of the following functions:
```c++
/**
* @brief Constructor
* @param det_threshold The sensitivity of the face detector implementation 
* (default=0.75)
*/
FaceDetector(const float det_threshold = 0.75f);

/**
* @brief Loads model and initializes the inference runtime
* @param model_path Path to the Mediapipe Blazeface face detector model (.tflite) file
* @param delegate_type The delegate to use for inference (CPU or TPU)(default=CPU)
* @param num_of_threads The number of CPU threads which can be used 
*                       by the inference runtime
*/
void load_model(const std::string model_path, 
                const face_detector_delegate delegate_type = face_detector_delegate::CPU, 
                const uint8_t num_of_threads = 4);

/**
 * @brief Loads image into model and does inference
 * @param frame Any frame which is formatted in CV_8UC3 or CV_8UC4 format
*/
void load_image(cv::Mat &frame);

/**
 * @brief Gets the Region of Interest, formatted as a square area (scaled to input image) where a detected face might be in
 * @return Rectangular area which contains a detected face
*/
cv::Rect get_face_roi();

/**
 * @brief Gets the 2D landmarks from the face.
 * @return Array with 6 Facial landmarks;
 *         Index 0: Left Eye
 *         Index 1: Right Eye
 *         Index 2: Nose
 *         Index 3: Mouth
 *         Index 4: Left ear
 *         Index 5: Right ear
*/
std::array<cv::Point, NUM_OF_FACE_DETECTOR_LANDMARKS> get_face_landmarks();

/**
 * @brief Determine whether a face was detected
 * @return 0 if face was detected, 1 if no face was detected in input frame
*/
int detected();
```
The comments above the functions describe fairly well what each function does. Here some additional notes;

!!! Note "Detector sensitivity"
    The constructor has the option to adjust the sensitivity of the face detector. As this option determines the minimum threshold of the region confidence score output of the model, threshold values closer to 0 will result in higher sensitivity, leading to more false positives. Conversely, a value closer to 1 will result in lower sensitivity, leading to more false negatives.


!!! Note "Model path"
    CMake generates a macro for the CPU and TPU models; As well as the models/ subfolder:

    `CFML_FACE_DETECTOR_CPU_MODEL_PATH`: Points to "models/CPU/face_detection.tflite"

    `CFML_FACE_DETECTOR_TPU_MODEL_PATH`: Points to "models/TPU/face_detection.tflite"

    `CFML_FACE_DETECTOR_MODEL_DIR`: Points to "models/" folder of this library

!!! Note "Number of threads"
    Number of threads indicate how many CPU threads to use for the inference delegate. 
    
    **Please note that this can only be used with the CPU delegate!**. When using the TPU delegate, this parameter will be ignored. As adding threads with TPU adds aditional latency as the threads have to wait for synchronisation everytime one thread offloads it's task to the same TPU.

!!! Note "Alignment of Face ROI"
    Please note that the Face ROI is top-left aligned. 

## CMake integration
This project uses CMake for generating the build files. The CMakeLists is configured as follows:

### Target(s)
The main target defined in the CMakeLists is the `face_detector` target. **As this will not be the only library released under the CLFML organisation, we chose to namespace it and call it `CLFML::face_detector`**. 

Other targets which are defined in the CMake files of this project are the Unit tests.

!!! Note "Unit Tests are not built by default!"
    Unit tests are not built by default! To build the unit-tests, pass in the `-DCLFML_FACE_DETECTOR_BUILD_UNIT_TESTS` argument to the CMake generator.

    e.g. `cmake -B build -DCLFML_FACE_DETECTOR_BUILD_UNIT_TESTS`


### Configuration options

Some of the configuration options which can be used to generate the CMake project are:

- `CLFML_FACE_DETECTOR_BUILD_UNIT_TESTS`; Build Unit tests (Default=OFF)

- `CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS`; Build example projects (Face roi demo) (Default=ON, *only when project is not part of other project)

- `CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU`; Enables Coral TPU support (Default=OFF)

### Integrating it into your own project
Here are some CMake snippets which indicate how this project might be included into your own CMake project.

!!! example "Automatically fetching from GitHub"
    CPU only:
    ```cmake
    include(FetchContent)

    FetchContent_Declare(
     Face_Detector.Cpp
     GIT_REPOSITORY https://github.com/CLFML/Face_Detector.Cpp.git
     GIT_TAG        main
    )
    FetchContent_MakeAvailable(Face_Detector.Cpp)

    ...

    target_link_libraries(YOUR_MAIN_EXECUTABLE_NAME CLFML::face_detector)
    ```

    TPU and CPU (requires version matched libedgetpu to be installed on your system *):
    ```cmake
    include(FetchContent)
    set(CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU ON CACHE INTERNAL "")  # Makes sure that the lib will build with TPU support

    FetchContent_Declare(
     Face_Detector.Cpp
     GIT_REPOSITORY https://github.com/CLFML/Face_Detector.Cpp.git
     GIT_TAG        main
    )
    FetchContent_MakeAvailable(Face_Detector.Cpp)

    ...

    target_link_libraries(YOUR_MAIN_EXECUTABLE_NAME CLFML::face_detector)
    ```
    
    *Version matched with the tensorflow used by this lib (currenly 1.16.1)

!!! example "Manually using add_subdirectory"
    First make sure that this library is cloned into the project directory!
        CPU only:
    ```cmake
    add_subdirectory(Face_Detector.Cpp)
    ...

    target_link_libraries(YOUR_MAIN_EXECUTABLE_NAME CLFML::face_detector)
    ```

    TPU and CPU (requires version matched libedgetpu to be installed on your system *):
    ```cmake
    set(CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU ON CACHE INTERNAL "")  # Makes sure that the lib will build with TPU support
    add_subdirectory(Face_Detector.Cpp)
    ...

    target_link_libraries(YOUR_MAIN_EXECUTABLE_NAME CLFML::face_detector)
    ```
    
    *Version matched with the tensorflow used by this lib (currenly 1.16.1)