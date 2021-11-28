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
