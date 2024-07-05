^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package av_camera_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
