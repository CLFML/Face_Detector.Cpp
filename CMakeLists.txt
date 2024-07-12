cmake_minimum_required(VERSION 3.1...3.14)

project(face_detector VERSION 3.11.3 LANGUAGES CXX)

set(MAIN_PROJECT OFF)

# Optionally build the example
option(BUILD_EXAMPLE_PROJECTS "Build example projects" ON)
# Optionally enable edge tpu support (Requires libedgetpu library to be installed!)
option(FACE_DETECTOR_ENABLE_CORAL_TPU "Enable CORAL EDGE_TPU support" OFF)


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

find_package(OpenCV REQUIRED)

add_library(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/face_detection.cpp)
add_library(CLFML::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/src)
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


if(FACE_DETECTOR_ENABLE_CORAL_TPU)
target_link_libraries(${PROJECT_NAME} edgetpu)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DFACE_DETECTOR_ENABLE_CORAL_SUPPORT=1)
endif()


set(FACE_DETECTOR_MODEL_DIR ${CMAKE_CURRENT_LIST_DIR}/models)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DFACE_DETECTOR_MODEL_DIR="${FACE_DETECTOR_MODEL_DIR}")

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

if(BUILD_EXAMPLE_PROJECTS)
  add_executable(face_roi_demo ${CMAKE_CURRENT_LIST_DIR}/example/face_roi_demo/demo.cpp)
  target_link_libraries(face_roi_demo PUBLIC face_detector)
endif()


unset(BUILD_EXAMPLE_PROJECTS CACHE)