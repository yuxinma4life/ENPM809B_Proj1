# ENPM809B_Proj1


Start Ariac Environment 
```
rosrun osrf_gear gear.py -f `catkin_find --share osrf_gear`/config/qual2a.yaml ~/helloworld_ws/src/ariac_example/config/team_conf.yaml
```

Start MoveIt
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
```

Start move_arm service node
```
rosrun move_arm move_srv
```


To move the arm, call service like the following --
```
rosservice call /move_arm/toPose "pose:
  position:
    x: -0.2
    y: 0.33
    z: 0.745
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
mode: 1" 
```
