To obtain whole sources use next commands:
git submodule update --init


_To start board-part_: ros2 launch bringup board.py (includes strt of lidar (LD19 lidar), imu (MPU6050) and DiffDriveControl (ensure that firmware of ESP32 is the https://github.com/sckomoroh/waveshare_min_control))
_To start indoor navigation_: ros2 launch bringup nav.py (start the navigation stack)


**NOTE:**
You must apply udev rules, located in the **helpfull_stuff/99-serial.rules** for correct work or change the device names in the appropriate launch files


You can configure you hardare driver in the src/hardware_drivers/motors_driver_node/urdf/controller.urdf.xacro (part of the URDF contains the configuration for the DiffDriveControl):
**left_pwm_multiplier** and **right_pwm_multiplier** uses to convert the velocity to the PWM
**left_encoder_direction** and **right_encoder_direction** responsible for the encoder direction. Should be 1 or -1


**Repository content:**
_bringup_ - contains the launch files to launch the appropriate nodes
_description_ - contains the robot model URDF description
_navigation_ - contains the configuration parameters for the localization stack
_hardware_drivers/gps_driver_node_ - GPS driver node
_hardware_drivers/lidar_driver_node_ - LD19 lidar driver node
_hardware_drivers/motors_driver_node_ - DiffDrive driver node
_hardware_drivers/mpu_driver_node_ - MPU6050 driver node
_rviz2_plugins/monitor_rviz2_plugin_ - RVIZ2 plugin for velocity monitoring
_rviz2_plugins/rviz_reconfigure_plugin_ - RVIZ2 plugin for real-time reconfiguring nodes parameters
_rviz2_plugins/rviz_satellite_ - RVIZ2 plugin to visualize map in case if using the GPS functionality
_tools/monitor_ - GUI based tool to monitor the topics values in real-time

**NOTE:** all entry in the repository this is the sepatare git module


