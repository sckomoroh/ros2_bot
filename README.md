To obtain whole sources use next commands:
git submodule update --init

To start board-part: ros2 launch bringup board.py
To start indoor navigation: ros2 launch bringup nav.py

NOTE:
You must apply udev rules, located in the helpfull_stuff/99-serial.rules for correct work

You can configure you hardare driver in the src/hardware_drivers/motors_driver_node/urdf/controller.urdf.xacro:
left_pwm_multiplier and right_pwm_multiplier uses to convert the velocity to the PWM
left_encoder_direction and right_encoder_direction responsible for the encoder direction. Should be 1 or -1

