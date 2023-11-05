To obtain whole sources use next commands:
git submodule update --init


To start board-part: ros2 launch bringup board.py (includes strt of lidar (LD19 lidar), imu (MPU6050) and DiffDriveControl (ensure that firmware of ESP32 is the https://github.com/sckomoroh/waveshare_min_control))
To start indoor navigation: ros2 launch bringup nav.py (start the navigation stack)


NOTE:
You must apply udev rules, located in the helpfull_stuff/99-serial.rules for correct work or change the device names in the appropriate launch files


You can configure you hardare driver in the src/hardware_drivers/motors_driver_node/urdf/controller.urdf.xacro (part of the URDF contains the configuration for the DiffDriveControl):
left_pwm_multiplier and right_pwm_multiplier uses to convert the velocity to the PWM
left_encoder_direction and right_encoder_direction responsible for the encoder direction. Should be 1 or -1


Repository content:
bringup - contains the launch files to launch the appropriate nodes
description - contains the robot model URDF description
navigation - contains the configuration parameters for the localization stack
hardware_drivers/gps_driver_node - GPS driver node
hardware_drivers/lidar_driver_node - LD19 lidar driver node
hardware_drivers/motors_driver_node - DiffDrive driver node
hardware_drivers/mpu_driver_node - MPU6050 driver node
rviz2_plugins/monitor_rviz2_plugin - RVIZ2 plugin for velocity monitoring
rviz2_plugins/rviz_reconfigure_plugin - RVIZ2 plugin for real-time reconfiguring nodes parameters
rviz2_plugins/rviz_satellite - RVIZ2 plugin to visualize map in case if using the GPS functionality
tools/monitor - GUI based tool to monitor the topics values in real-time

NOTE: all entry in the repository this is the sepatare git module


