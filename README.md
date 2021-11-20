# robokit-atom
Workspace for Robokit Atom robot. 

## Installation
 
 ```bash
  ./install_deps.sh
  ./install.sh
 ```

For future project building you can also use ```install.sh```. It ```catkin_make``` with python3 and source ```devel/setup.sh```.

## Launching

To launch test vision script:

```bash
 roslaunch vision vision_test.launch
 rosrun motion walk_motion_srver.py
 rosrun motion walk_motion_srver.py
```


