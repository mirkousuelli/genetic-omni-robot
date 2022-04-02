# Omnidirectional Robot Odometry
Omnidirectional 4-wheels mobile robot odometry implementation in ROS (C++) as "Robotics" course project 2022, Politecnico di Milano.

Real Robot                 |  Robot Scheme
:-------------------------:|:-------------------------:
![](img/real.png)          |  ![](img/scheme.png)

## 1. Authors
- Alessandro Restifo, 12345678
- Mirko Usuelli, 10570238

## 2. Archive files descriptions
to be done...

## 3. TF tree
```
omni-robot/
.
├── bags
│   ├── bag1.bag
│   ├── bag2.bag
│   └── bag3.bag
├── cfg
│   └── parameters.cfg
├── CMakeLists.txt
├── config
│   └── omni_config.yaml
├── include
│   ├── omni_odometry.h
│   ├── omni_params
│   ├── omni_reset.h
│   └── omni-robot
│       ├── omni_model.h
│       └── omni_tester.h
├── launch
│   └── omni-robot.launch
├── package.xml
├── README.md
├── script
│   └── plot_result.py
├── src
│   ├── omni_model.cpp
│   ├── omni_model_node.cpp
│   ├── omni_odometry.cpp
│   ├── omni_reset.cpp
│   ├── omni_tester.cpp
│   └── omni_tester_node.cpp
└── srv
    └── omni_reset.srv
```

## 4. Custom Messages
to be done...

## 5. Instrucions
- Insert the ROS packages by typing:
```
  $ mv omni-robot <path>/catkin_ws/src
```
- Go to your own *catkin_ws* folder in the system and recompile everything by doing:
```
  $ cd <path>/catkin_ws
  $ source ./devel/setup.bash
  $ catkin_make
```
- Then run `ROS core` by typing:
```
  $ roscore
```
- Open a new terminal and launch the robot simulation as:
```
  $ cd <path>/catkin_ws
  $ source ./devel/setup.bash
  $ roslaunch omni-robot omni-robot.launch
```

## 6. Further info
to be done...
