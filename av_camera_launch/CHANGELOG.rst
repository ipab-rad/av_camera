^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package av_camera_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Migrate from synchronised to non-synchronised FLIR cam driver (`#27 <https://github.com/ipab-rad/av_camera/issues/27>`_)
  Synchronised driver works well when all cameras have to acquire images synchronously (i.e. at the exact same timestamp).
  However, we have chosen to synchronised our cameras with our top rotating lidar instead, meaning we need the non-synchronised driver.
  The top lidar sends out an electronic pulse whenever the encoder reaches azimuth 0, and all cameras receive said pulse. We then delay each camera via FLIR firmware to match the azimuth angle at which each camera has the lidar traversing through its FOV.
  Co-authored-by: Alejandro Bordallo <alex.bordallo@ed.ac.uk>
* Contributors: Hector Cruz

2.0.1 (2024-11-26)
------------------
* Temp Fix: Use compute brightness fix from source
  - Clone ROS2 flir driver repo with brightness computation fix upstream
  - Enable compute brightness, disable gain auto and set 100 brightness
* Contributors: Alejandro Bordallo

2.0.0 (2024-11-26)
------------------
* Implement cameras launch file with synchronisation (`#13 <https://github.com/ipab-rad/av_camera/issues/13>`_)
  - Use ROS2 spinnaker synchronized camera driver instead of default
  - Modify camera config parameter yaml files to fit new structure
  - Add brightness controller parameters as required
  - Update `all_cams.launch.xml` to run all cameras in single node
  - Set exposure min/max on Brighntess Controller to 1ms (constant)
  - NOTE: Temporarily disable Brightness Controller (Use continuous gain) until https://github.com/ros-drivers/flir_camera_driver/issues/216 is released into the wild
  ---------
  Co-authored-by: Hector Cruz <hcruzgo@ed.ac.uk>
* Contributors: Alejandro Bordallo

1.2.0 (2024-10-30)
------------------
* Use 1 ms exposure for all cameras
* Contributors: Alejandro Bordallo

1.1.0 (2024-09-24)
------------------
* Add new ROIs (`#7 <https://github.com/ipab-rad/av_camera/issues/7>`_)
  - 2400 out of 2448 covers nearly the entire horizontal width of image
  - 1200 out of 2048 ignores most of the car bonnet/boot and most of sky
  - Horizontal/Vertical Offsets inverted for upside down cameras
  - 2400 x 1200 also is 2:1 which is nice
  - Set exposure control to manual at 5000 us
  - Re-calibrate cameras with new ROIs and update files

  Co-authored-by: Hector Cruz <hcruzgo@ed.ac.uk>
* Contributors: Alejandro Bordallo, Hector Cruz

1.0.0 (2024-07-05)
------------------
* Configure cameras to run in `trigger_mode` to support external hardware 
  synchronisation. Flir camera driver will not publish until cameras are 
  triggered.
* Add pre-commit support.
* Add Cyclone DDS as ROS RMW and configure it to support high message 
  throughput.
* Enable colourised ROS logs.
* Add Docker build and push workflow.
  - `av_camera` is used as IMAGE_NAME rather than repository name.
* Simplify `av_camera_launch` package.
  - Remove dev/test old launch files.
  - Modify `chameleon_config.yaml` to run cameras in synchronised mode and
  explicitly use it when launching cameras.
* Avoid `dev.sh` overriding `latest` Docker tag for convenience.
* Synchronise host time with Docker container.

  Co-authored-by: Hector Cruz <hcruzgo@ed.ac.uk>
* Contributors: Alejandro Bordallo, Hector Cruz
