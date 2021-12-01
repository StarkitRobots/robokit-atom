# robokit-atom
Workspace for Robokit Atom robot. 

## Installation
 
 ```bash
 ./install_deps.sh
 ```

## Building

```bash 
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.sh
```

## Launching

To launch camera, model and motion nodes: 

```bash
roslaunch core core.launch
```

To launch vision node:

```bash
roslaunch core vision_test.launch
```

To launch motion node:

```bash
rosrun core walk_motion_node.py
```

## ROS debug

1. On PC

```bash
export ROS_MASTER_URI=http://10.0.0.25:11311
```
2. On robot:

```bash
export ROS_IP=10.0.0.25
```


