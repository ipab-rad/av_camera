# flir_camera_driver

This repository contains ROS packages for cameras made by FLIR Imaging (formerly known as PointGrey).

## Packages

### spinnaker_camera_driver
The camera driver supports USB3 and GIGE cameras. The driver has been
successfully used for Blackfly, Blackfly S, Chameleon, and Grasshopper
cameras, but should support any FLIR camera that is based on the
Spinnaker SDK. See the
[spinnaker_camera_driver](spinnaker_camera_driver/README.md) for more.
This software is issued under the Apache License Version 2.0 and BSD

### flir_camera_msgs
Package with with [image exposure and control messages](flir_camera_msgs/README.md).
These are used by the [spinnaker_camera_driver](spinnaker_camera_driver/README.md).
This software is issued under the Apache License Version 2.0.

### flir_camera_description
Package with [meshes and urdf](flir_camera_description/README.md) files.
This software is released under a BSD license.

## Usage

This repository is designed to be used alongside a Docker container. Quickly build and run the Docker container using `run.sh` for runtime or debugging, and `dev.sh` for a convenient development setup.

### Runtime or Debugging

Execute the ROS 2 nodes in runtime mode or start an interactive bash session for detailed debugging:

```bash
./run.sh [bash]
```

- **Without arguments**: Activates the container in runtime mode.
- **With `bash`**: Opens an interactive bash session for debugging.

### Development

Prepare a development setting that reflects local code modifications and simplifies the build process:

```bash
./dev.sh
```

- **Live Code Synchronization**: Mounts local `av_camera_launch`, `flir_camera_msgs` and `spinnaker_camera_driver` directories with the container.
- **Convenience Alias**: The development container features a `colcon_build` alias, which simplifies the ROS2 build process. Executing `colcon_build` runs `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release` and then sources the `setup.bash` to ensure the environment is updated with the latest build artifacts. This alias enhances productivity by combining build commands and environment setup into a single, easy command.
