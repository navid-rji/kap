

1. In each Terminal do this export:
```
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libffi.so.7
```

2. Launch the ros master in the first terminal
```
roscore
```

3. Do this to make the robot publish its pose in the second terminal
```
roslaunch franka_control franka_control.launch robot_ip:=172.31.1.149
```

4. start the clarius driver node in the third terminal
```
rosrun kap clarius_driver_node.py -p <port>
```

5. start the acquisition controller in the fourth terminal
```
rosrun kap acquisition_controller.py
```
