cmake_minimum_required(VERSION 3.1...3.14)

project(face_detector VERSION 3.11.5 LANGUAGES CXX)

set(MAIN_PROJECT OFF)

# Optionally build the examples
option(CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS "Build example projects" ON)

# Optionally enable edge tpu support (Requires libedgetpu library to be installed!)
option(CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU "Enable CORAL EDGE_TPU support" OFF)

# Optionally enable ROS2 package build (Requires ROS2 jazzy to be installed!)
option(CLFML_ROS2_PACKAGE_BUILD "Build a ROS2 package" OFF)

if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CMAKE_CXX_STANDARD 17)
  set(MAIN_PROJECT ON)
else()
  set(CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS OFF)
endif()

if (POLICY CMP0077)
  # Allow CMake 3.13+ to override options when using FetchContent / add_subdirectory.
  cmake_policy(SET CMP0077 NEW)
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/face_detection.cpp)
add_library(CLFML::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if (TARGET tensorflow-lite)
  target_link_libraries(${PROJECT_NAME} tensorflow-lite)
else()

  if(${CMAKE_SYSTEM_NAME} STREQUAL "Windows" OR CLFML_FACE_DETECTOR_COMPILE_TENSORFLOW_FROM_SOURCE)
    include(FetchContent)
    FetchContent_Declare(
        tensorflow_rel_package
        URL https://github.com/tensorflow/tensorflow/archive/refs/tags/v2.16.1.zip
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
        SOURCE_SUBDIR tensorflow/lite
    )
    FetchContent_MakeAvailable(tensorflow_rel_package)
  else()
    include(FetchContent)
    FetchContent_Declare(
        tensorflow_compiled_rel_package
        URL https://github.com/CLFML/TensorFlow_Lite_Compiled/releases/download/v2.16.1/tensorflow_linux_generic_x86_64.zip
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
    )
    FetchContent_MakeAvailable(tensorflow_compiled_rel_package)
    if(CLFML_ROS2_PACKAGE_BUILD)
      install(FILES external/tensorflow/lib/libtensorflowlite.so DESTINATION lib/${PROJECT_NAME})
    endif()
  endif()
  target_link_libraries(${PROJECT_NAME} tensorflow-lite)
endif()

if(CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU)
  target_link_libraries(${PROJECT_NAME} edgetpu)
  set(FACE_DETECTOR_CORAL_MODEL_PATH ${CMAKE_CURRENT_LIST_DIR}/models/Coral/face_detection.tflite)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DFACE_DETECTOR_ENABLE_CORAL_SUPPORT=1
                                                      -DCFML_FACE_DETECTOR_CORAL_MODEL_PATH="${FACE_DETECTOR_CORAL_MODEL_PATH}")
endif()

set(FACE_DETECTOR_CPU_MODEL_PATH ${CMAKE_CURRENT_LIST_DIR}/models/CPU/face_detection.tflite)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCFML_FACE_DETECTOR_CPU_MODEL_PATH="${FACE_DETECTOR_CPU_MODEL_PATH}")

set(FACE_DETECTOR_MODEL_DIR ${CMAKE_CURRENT_LIST_DIR}/models/)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCFML_FACE_DETECTOR_MODEL_DIR="${FACE_DETECTOR_MODEL_DIR}")

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

if (CLFML_ROS2_PACKAGE_BUILD)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(cv_bridge REQUIRED)
  target_link_libraries(${PROJECT_NAME} ${cv_bridge_LIBRARIES})
  add_executable(camera_node ${CMAKE_CURRENT_LIST_DIR}/bindings/ros2/camera_node.cpp)
  ament_target_dependencies(camera_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    geometry_msgs
    std_srvs
    OpenCV
  )
  add_executable(face_detector_node ${CMAKE_CURRENT_LIST_DIR}/bindings/ros2/face_detector_node.cpp)
  ament_target_dependencies(face_detector_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    geometry_msgs
    std_srvs
    OpenCV
  )
  target_link_libraries(camera_node face_detector)
  target_link_libraries(face_detector_node face_detector)
  install(TARGETS
    camera_node
    face_detector_node
    DESTINATION lib/${PROJECT_NAME})
endif()

if(CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS)
  if (CLFML_ROS2_PACKAGE_BUILD)
    add_executable(face_detector_viewer ${CMAKE_CURRENT_LIST_DIR}/example/ROS_demo/face_detector_viewer.cpp)
    ament_target_dependencies(face_detector_viewer
        rclcpp
        sensor_msgs
        cv_bridge
        std_msgs
        geometry_msgs
        std_srvs
        OpenCV
      )
    target_link_libraries(face_detector_viewer face_detector)
    install(TARGETS
      face_detector_viewer
      DESTINATION lib/${PROJECT_NAME})
  else ()
    add_executable(face_roi_demo ${CMAKE_CURRENT_LIST_DIR}/example/face_roi_demo/demo.cpp)
    target_link_libraries(face_roi_demo PUBLIC face_detector)
  endif()
endif()

if (CLFML_ROS2_PACKAGE_BUILD)
  install(FILES bindings/ros2/set_ld_path.sh DESTINATION lib/${PROJECT_NAME})
  ament_package()
endif()

