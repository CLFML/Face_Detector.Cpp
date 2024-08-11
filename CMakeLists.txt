cmake_minimum_required(VERSION 3.1...3.14)

project(face_detector VERSION 3.11.5 LANGUAGES CXX)

set(MAIN_PROJECT OFF)

# Optionally build the examples
option(CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS "Build example projects" ON)

# Optionally enable edge tpu support (Requires libedgetpu library to be installed!)
option(CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU "Enable CORAL EDGE_TPU support" OFF)

# Optionally enable ROS2 package build (Requires ROS2 jazzy to be installed!)
option(ROS2_PACKAGE_BUILD "Build a ROS2 package" ON)
# set(ROS2_PACKAGE_BUILD ON CACHE BOOL "Build a ROS2 package" FORCE) # explicitly override cache

if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(CMAKE_CXX_STANDARD 17)
    set(MAIN_PROJECT ON)
else()
    set(BUILD_EXAMPLE_PROJECTS OFF)
endif()

if (POLICY CMP0077)
    # Allow CMake 3.13+ to override options when using FetchContent / add_subdirectory.
    cmake_policy(SET CMP0077 NEW)
endif ()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

find_package(OpenCV REQUIRED)

if (ROS2_PACKAGE_BUILD)
    # set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "/opt/ros/jazzy")
    # set(ament_cmake_DIR "/opt/ros/jazzy/share/ament_cmake/cmake")
    # set(cv_bridge_DIR "/opt/ros/jazzy/share/cv_bridge/cmake")
    # include_directories(/opt/ros/jazzy/include)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    # include_directories(
    #     ${OpenCV_INCLUDE_DIRS}
    #     ${cv_bridge_INCLUDE_DIRS}
    # )
endif ()

add_library(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/face_detection.cpp)
add_library(CLFML::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/src)

# if (ROS2_PACKAGE_BUILD)
#     target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/ros/src)
# endif ()

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

if (TARGET tensorflow-lite)
    target_link_libraries(${PROJECT_NAME} tensorflow-lite)
else()
    include(FetchContent)
    FetchContent_Declare(
        tensorflow_rel_package
        URL https://github.com/tensorflow/tensorflow/archive/refs/tags/v2.16.1.zip
        SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/external/tensorflow
        SOURCE_SUBDIR tensorflow/lite
    )
    FetchContent_MakeAvailable(tensorflow_rel_package)
    target_link_libraries(${PROJECT_NAME} tensorflow-lite)
endif()

if(CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU)
    target_link_libraries(${PROJECT_NAME} edgetpu)
    set(FACE_DETECTOR_CORAL_MODEL_PATH ${CMAKE_CURRENT_LIST_DIR}/models/Coral/face_detection.tflite)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DFACE_DETECTOR_ENABLE_CORAL_SUPPORT=1 
                                                      -DCFML_FACE_DETECTOR_CORAL_MODEL_PATH="${FACE_DETECTOR_CORAL_MODEL_PATH}")
endif()

if (ROS2_PACKAGE_BUILD)
    target_link_libraries(${PROJECT_NAME} ${cv_bridge_LIBRARIES}) 
endif ()

set(FACE_DETECTOR_CPU_MODEL_PATH ${CMAKE_CURRENT_LIST_DIR}/models/CPU/face_detection.tflite)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCFML_FACE_DETECTOR_CPU_MODEL_PATH="${FACE_DETECTOR_CPU_MODEL_PATH}")

set(FACE_DETECTOR_MODEL_DIR ${CMAKE_CURRENT_LIST_DIR}/models/)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCFML_FACE_DETECTOR_MODEL_DIR="${FACE_DETECTOR_MODEL_DIR}")


set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

if(CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS)
    if (ROS2_PACKAGE_BUILD)
      add_executable(face_detector_node ${CMAKE_CURRENT_LIST_DIR}/example/face_roi_ROS_demo/demo.cpp)
      ament_target_dependencies(face_detector_node
        rclcpp
        sensor_msgs
        cv_bridge
        std_msgs
        geometry_msgs
        std_srvs
        OpenCV
      )
      target_link_libraries(face_detector_node face_detector)  # older style of target_link_libraries required to comply with ROS2 ament
    else ()
      add_executable(face_roi_demo ${CMAKE_CURRENT_LIST_DIR}/example/face_roi_demo/demo.cpp)
      target_link_libraries(face_roi_demo PUBLIC face_detector)
    endif ()
endif()

if (ROS2_PACKAGE_BUILD)
  install(TARGETS 
    face_detector_node  
    # TODO: add a listener node
    DESTINATION lib/${PROJECT_NAME})
endif ()

unset(CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS CACHE)

if (ROS2_PACKAGE_BUILD)
  ament_package()
endif ()

# if(ROS2_PACKAGE_BUILD)
#   message(STATUS "cv_bridge_INCLUDE_DIRS: ${cv_bridge_INCLUDE_DIRS}")
#   message(STATUS "cv_bridge_LIBRARIES: ${cv_bridge_LIBRARIES}")
# endif()

# get_target_property(INCLUDE_DIRS ${PROJECT_NAME} INCLUDE_DIRECTORIES)
# message(STATUS "Include directories for ${PROJECT_NAME}: ${INCLUDE_DIRS}")