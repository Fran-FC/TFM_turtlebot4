# Documentación de cada tarea relacionada con ROS2

## Acceso a velocidades y voltajes de las ruedas

Se encuentran los siguientes topics relacionados con información sobre las ruedas:

1. `/wheel_status` de tipo `irobot_create_msgs/msg/WheelStatus`
2. `/wheel_ticks` de tipo `irobot_create_msgs/msg/WheelTicks`
3. `/wheel_vels` de tipo `irobot_create_msgs/msg/WheelVels`


El topic `/wheel_status` nos otorga información sobre la intensidad de corriente de cada rueda y el PWM (porcentaje de tiempo en el que se envía corriente a la rueda):

```
$ ros2 interface show irobot_create_msgs/msg/WheelStatus

# This message contains status on the wheels

std_msgs/Header header              # Header stamp should be acquisition time of measure.
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
int16 current_ma_left               # Current measurement for left wheel in milliamps
int16 current_ma_right              # Current measurement for right wheel in milliamps
int16 pwm_left                      # PWM % duty cycle measurement (where int16::max is +100%) for left wheel
int16 pwm_right                     # PWM % duty cycle measurement (where int16::max is +100%) for right wheel
bool wheels_enabled                 # Whether wheels are enabled or disabled (disabled when E-Stopped)

```

Para obtener las velocidades de las ruedas se utilizará `/wheel_vels` y para los 