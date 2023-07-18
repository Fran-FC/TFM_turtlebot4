# Turtlebot 4 y ROS2: análisis de algoritmos SLAM y de navegación

## Documentación
- [Puesta en marcha e instalación del Turtlebot 4](docs/conexi%C3%B3n_PC_turtlebot.md)
- [Documentación ROS2](docs/ros2_documentacion.md)
- [Documentación SLAM](docs/SLAM_documentacion.md)

## Resultados

- [Análisis de trayectorias](docs/análisis_trayectorias.md)
- [Análisis de los mapas](docs/análisis_mapas.md)
- 
## Algoritmos testeados

<div align="center">
  
|Tipo SLAM|Algoritmo|Version de ROS|Funciona|Activo|Formato Pose|
|:---:|:---:|:---:|:---:|:---:|:---:|
|SLAM 2D| [Cartographer](https://github.com/ros2/cartographer_ros) | ROS2 Humble |✅|✅|[visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)|
|SLAM 2D| [HectorSLAM](https://github.com/RRL-ALeRT/hector_slam_ros2) | ROS2 Humble |✅|✅|[geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html)
|SLAM 2D| [slam_gmapping](https://github.com/Project-MANAS/slam_gmapping) | ROS2 Humble |✅|❌|No se publica topic con la posición|
|SLAM 2D| [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | ROS2 Humble |✅|✅|[geometry_msgs/msg/PoseWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
|SLAM 2D| [TinySLAM](https://github.com/OSLL/tiny-slam-ros-cpp) | ROS Kinetic  |❌|❌|----|
|VISUAL SLAM - monocular| [ORB-SLAM](https://github.com/OpenSLAM-org/openslam_orbslam) | ROS Indigo  |❌|❌|---|
|VISUAL SLAM - monocular, rgbd, stereo| [ORB-SLAM2](https://github.com/appliedAI-Initiative/orb_slam_2_ros/tree/ros2) | ROS2 |❌|❌|---|
|VISUAL SLAM - monocular, rgbd, stereo, stereo-inertial| [ORB-SLAM3](https://github.com/zang09/ORB_SLAM3_ROS2/tree/humble) | ROS2 Humble |❌|❌|---|
|VISUAL 2D SLAM| [RTAB-Map](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros) | ROS2 Humble  |✅|✅|[geometry_msgs/msg/PoseWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
|VISUAL SLAM - monocular, stereo, stereo-inertial| [VINS-Fusion](https://github.com/zinuok/VINS-Fusion-ROS2) | ROS2 Foxy  |❌|✅|[geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseStamped.html)|

</div>

## SLAM 2D

![comparacion_slam](https://github.com/Fran-FC/TFM_turtlebot4/assets/72190914/b5a3958b-11cc-4957-a29b-06551ec91b49)

## Resumen
El siguiente trabajo de fin de máster tiene como objetivo la puesta en marcha de un Turtlebot 4 y el posterior análisis de diferentes algoritmos SLAM. El robot Turtlebot 4, de la empresa canadiense Clearpath Robotics, es una expansión del Create 3 de iRobot, una empresa estadounidense. Utilizará ROS 2 en su última distribución, Humble. Esta nueva versión de ROS, pese a tener un periodo muy corto de vida y no tener tantas librerías desarrolladas como ROS 1, supone un gran avance en muchos aspectos respecto a la primera versión. Utilizando estas últimas tecnologías se realizará un estudio de varios algoritmos de localización y mapeo simultáneos (SLAM) con la intención de ver cual es es más eficiente y adecuado para este robot. Para ello se obtendrá el modelo cinemático diferencial del Turtlebot y así obtener un mayor ajuste de los algoritmos. También se pretende tener un control más “fino”, controlando cada rueda por separado, en lugar de mandar comandos de velocidad linear o angular del robot. Con las mediciones realizadas se concluirán cuales son los algoritmos y la configuración más óptima para el Turtlebot 4. Se pondrán en práctica muchos conocimientos adquiridos en las asignaturas de Automatización y Robótica, así como conceptos y conocimientos de las asignaturas de Sistemas Inteligentes, Sistemas y Aplicaciones Distribuidas, Configuración y Optimización de Sistemas de Cómputo y finalmente Redes y Seguridad. 

## Referencias
- [A Survey of Simultaneous Localization and Mapping](https://arxiv.org/pdf/1909.05214v3.pdf)
- [2D SLAM Quality Evaluation Methods](https://arxiv.org/pdf/1708.02354.pdf)
- [Benchmarking of 2D-Slam Algorithms](http://www.acro.be/downloadvrij/Benchmark_2D_SLAM.pdf)
- [On Measuring the Accuracy of SLAM Algorithms](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/kuemmerle09auro.pdf)
- [ORB-SLAM: A Versatile and Accurate Monocular SLAM System](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7219438)
- [ORB-SLAM2: An Open-Source SLAM System for Monocular, Stereo, and RGB-D Cameras](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=7946260)
- [ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM](https://arxiv.org/pdf/2007.11898.pdf)
