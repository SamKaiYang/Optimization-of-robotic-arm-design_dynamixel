# Optimization-of-robotic-arm-design
## Dependency Package
1. [ROS](https://wiki.ros.org/melodic/Installation/Ubuntu)
2. [roboticstoolbox](https://github.com/petercorke/robotics-toolbox-python)
3. [ROS-MoveIt](https://moveit.ros.org/install/source/)

# Docker image
1. [DockerHub](https://hub.docker.com/repository/docker/samkaiyang/opt_dynamic_design)
2. [Github](https://github.com/SamKaiYang/docker-Optimization-of-robotic-arm-design.git)

## Installation
```bash
sudo apt-get update && sudo apt-get -y upgrade
python -m pip install --user --upgrade pip==20.2.4
sudo apt-get install build-essential
sudo apt-get install qttools5-dev-tools
sudo apt-get install qtcreator
sudo apt-get install qt5-default
sudo apt-get install qt5-doc
sudo apt-get install qt5-doc-html qtbase5-doc-html
sudo apt-get install qtbase5-examples
sudo apt-get install libxcb-xinerama0
sudo apt install -y ros-melodic-moveit

pip3 install -r requirements.txt
``` 

## References
###  UR5
1. https://github.com/tku-iarc/UR5_control
2. https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
### Python code 
1. [https://petercorke.github.io/robotics-toolbox-python/intro.html#robotics-toolbox](https://petercorke.github.io/robotics-toolbox-python/intro.html#robotics-toolbox)
2. [https://github.com/petercorke/robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python)
3. [http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/index.html](http://motion.cs.illinois.edu/software/klampt/latest/pyklampt_docs/index.html)
4. https://github.com/cdsousa/SymPyBotics
5. [https://github.com/pydy/pydy](https://github.com/pydy/pydy)
6. [https://github.com/Sarrasor/RoboticManipulators](https://github.com/Sarrasor/RoboticManipulators)[https://petercorke.github.io/robotics-toolbox-python/intro.html#robotics-toolbox](https://petercorke.github.io/robotics-toolbox-python/intro.html#robotics-toolbox)
### C code
1. [https://github.com/dartsim/dart](https://github.com/dartsim/dart)
2. https://github.com/CNR-STIIMA-IRAS/rosdyn
3. [https://github.com/roboticslibrary/rl](https://github.com/roboticslibrary/rl)

## Solves
1. https://blog.csdn.net/gyxx1998/article/details/118153079
2. https://blog.csdn.net/zxxxiazai/article/details/103568577


# 範例程式
==================
## 運行手臂軌跡規劃,求取各軸角度,速度,加速度
```bash
rosrun trajectory_generation trajectory_generation_teco.py 
```
# 顯示與操作介面
```bash
roslaunch dynamics dynamics_general_robot.launch 
```
# 執行moveit OMPL demo
```bash
roslaunch teco_config demo.launch
##moveit python interface 
rosrun teco_config move_group_python_interface_tutorial.py 
```

```bash
roslaunch teco_config demo.launch
##moveit c++ interface launch
roslaunch moveit_tutorials move_group_interface_teco.launch 
```

# 執行moveit industrial motion demo
```bash
roslaunch teco_config demo_pliz.launch
```


# 執行DRL訓練
```bash
roslaunch dynamics dynamics_DRL.launch
# tensorboard 可視化
tensorboard --logdir {log}
```

>>2022/08/22 更新
1. 隨機軸長生成, 並套用DRL學習, 目前使用DQN網路訓練, 離散數值輸入

>>未完成項目
1. 使用模組化關節(Dynamixal or TECO) 實現多種構型生成

>>2023/01/02 更新

# DQN
roslaunch dynamics dynamics_dynamixel_robot_dqn.launch
# DDQN
roslaunch dynamics dynamics_dynamixel_robot_ddqn.launch

# tensorboard 可視化
tensorboard --logdir ~/Documents/drl_robotics_arm_ws/src/Optimization-of-robotic-arm-design/dynamics/src/dynamics/runs/