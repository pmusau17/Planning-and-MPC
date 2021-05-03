# Planning-and-MPC

Let's see what we can learn about MPC and RRT*

This repository assumes that you have this [repo](https://github.com/pmusau17/Platooning-F1Tenth) cloned and built. This repository contains implementations of planning algorithms for the F1Tenth Platform. I'm going to start writing tutorials on this as I learn. 

### RRT* Random Sampling:
![RRT_Biased](images/rrt_normal.gif)

### RRT* with biased sampling near the car
![RRT Biased](images/rrt_biased.gif)

### Closer Look at Solution 
![RRT_Grid](images/RRT_grid.png)

### Result after 2500 Samples:
![Porto Grid](images/Porto2500.png)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

### Building the Docker Container

```
$ ./build_docker.sh
```

```
$ ./run_docker.sh
```

In the terminal launched by run_docker.sh, the first thing you will do is build the ros packages.

```
$ catkin_make 
```

and then launch the rsband_local_planner 

```
$ roslaunch rsband_local_planner move_base_planning.launch
```

You can then set goal poses in rviz
