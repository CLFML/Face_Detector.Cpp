#!/bin/sh

# Check if the script is run from the root of the project by checking if the "install" directory exists
if [ ! -d "install" ]; then
  echo "Error: This script must be run from the root of the project. Unable to set the library path for tensorflow!"
  exit 1
fi

# Path to the dynamic library
LIB_PATH="install/face_detector/lib/face_detector/libtensorflowlite.so"

# Check if the dynamic library exists
if [ -f "$LIB_PATH" ]; then
    echo "Library $LIB_PATH found. Added to LD_LIBRARY_PATH!"
    
    # Get the directory path of the library
    LIB_DIR=$(dirname "$LIB_PATH")

    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$LIB_DIR
else
    echo "Library $LIB_PATH not found. No changes made."
fi