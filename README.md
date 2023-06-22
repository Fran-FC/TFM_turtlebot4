# Turtlebot 4 y ROS2: análisis de algoritmos SLAM y de navegación

## Documentación
- [Puesta en marcha e instalación del Turtlebot 4](docs/conexi%C3%B3n_PC_turtlebot.md)
- [Documentación ROS2](docs/ros2_documentacion.md)
- [Documentación SLAM](docs/SLAM_documentacion.md)

## Algoritmos testeados
### SLAM 2D
- [x] Cartographer
- [x] HectorSLAM
- [x] gmapping
- [x] KartoSLAM (slam_toolbox)

### Visual SLAM
- [ ] ORB-SLAM
- [ ] ORB-SLAM2
- [ ] ORB-SLAM3
- [ ] RTAB-Map

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
