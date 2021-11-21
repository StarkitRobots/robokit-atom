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

To launch vision node:

```bash
 roslaunch core vision.launch
```

To launch motion node:

```bash
  rosrun core walk_motion_server.py
```



