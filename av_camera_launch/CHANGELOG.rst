^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package av_camera_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
