# Planning-and-MPC

Let's see what we can learn about MPC and RRT*

This repository assumes that you have this [repo](https://github.com/pmusau17/Platooning-F1Tenth) cloned and built. This repository contains implementations of planning algorithms for the F1Tenth Platform. I'm going to start writing tutorials on this as I learn. 


# RRT*:
![RRT_Biased](images/rrt_normal.gif)

### RRT* with biased sampling near the car
![RRT Biased](images/rrt_biased.gif)

### Closer Look at the "solution" 
![RRT_Grid](images/RRT_grid.png)

### Result after 2500 Samples:
![Porto Grid](images/Porto2500.png)

# Rsband Local Planner + Pure Pursuit

The rsband_local_planner combines an elastic band planner, a reeds shepp planner and a fuzzy logic based path tracking controller, to achieve reactive local planning for Car-Like robots with Ackermann or 4-Wheel-Steering. Originally proposed by [George Kouros](https://github.com/gkouros/rsband_local_planner)

![Short Term Planning](images/short_term_planning.gif)


![long_term_planning.gif](images/long_term_planning.gif)


# Building the Docker Container

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


Launch the [F1Tenth Simulator](https://github.com/pmusau17/Platooning-F1Tenth): 

```
roslaunch race move_base_planning.launch
```

In the docker terminal: 

```
$ roslaunch rsband_local_planner move_base_planning.launch
```

You can then set goal poses in rviz
