# Overview

Using a TPU can greatly improve performance, from our limited testing we got a 2-3x decrease in CPU utilization. TPU support is build in to our library and can be enabled by setting the `CLFML_FACE_DETECTOR_ENABLE_CORAL_TPU` to ON during the CMake Generation step like this;

```bash
cmake . -B build -DCLFML_FACE_DETECTOR_ENABLE_CORAL_TPU=ON -G Ninja
```

Before you build this library with Coral support turned on; You need to compile libedgetpu for your platform:

- [Linux x86_64](using_a_tpu_on_x86_64_linux.md)
- [Linux ARM64](using_a_tpu_on_arm64_linux.md)
- [Windows x86_64](using_a_tpu_on_x86_64_windows.md)

## Hardware

This library supports the PCIe aswell as the USB accelerators. For prices and stock, take a look at the [coral website](https://coral.ai/).

