## Habilitar Stereo Depth 

Para utilizar visual SLAM la cámara deberá publicar la imágen de profundidad. Se consigue modificando el archivo de configuración `/opt/ros/humble/share/turtlebot4_bringup/config/oakd_pro.yaml`, el contendio de este archivo de configuración se encuentra en el siguiente [enlace](../config/oakd/oakd_pro.yaml).

## Error en la visualización del laser en RViz

Parece que hay un problema en la visualización del lidar en rviz mientras se realiza el SLAM. Parece que no puede realizar la transformada entre el frame `map` y el frame `rplidar_link`:

```
[rviz2-1] [ERROR] [1683196573.628807857] [rviz2]: Lookup would require extrapolation into the future.  Requested time 1677796205,300857 but the latest data is at time 1677796205,209008, when looking up transform from frame [rplidar_link] to frame [map]
```

Esto no da problemas a la hora de crear el mapa, ya que parece que el SLAM funciona correctamente y lee los datos del laser bien. 

## Lectura de las velocidades y voltajes de las ruedas

Se encuentran los siguientes topics relacionados con información sobre las ruedas:

1. `/wheel_status` de tipo `irobot_create_msgs/msg/WheelStatus`
2. `/wheel_ticks` de tipo `irobot_create_msgs/msg/WheelTicks`
3. `/wheel_vels` de tipo `irobot_create_msgs/msg/WheelVels`


El topic `/wheel_status` nos otorga información sobre la intensidad de corriente de cada rueda y el PWM (porcentaje de tiempo en el que se envía corriente a la rueda).

Para obtener las velocidades de las ruedas se utilizará `/wheel_vels` y para los ticks del encoder `/wheel_ticks`.

## Escritura de las velocidades y voltajes de las ruedas

Por el momento no hay ningún topic disponible para darle velocidades a las ruedas.

En la siguiente imagen se puede ver que el nodo `/motion_control` publica en un topic interno llamado `/_internal/wheels_command`. Podría ser interesante modificar el nodo `/motion_control` para que ofreciera un topic en el que publicar las velocidades de las ruedas y que este se lo pasara al topic interno. 

Aunque esto no es posible debido a que este paquete es del create3, y su código es privado.
