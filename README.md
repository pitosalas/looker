# nav_tools

## Motivation

I want to learn about costmaps. I have an idea for an application that simply uses costmaps by themselves for navigation and goal seeking. But my personal goal is to learn about costmaps, lifecycle nodes, and more advanced ros2 features. So there are other ways to solve my nominal goal, like running the whole nav2 stack, and starting with an empty map. But my goal is to either fiture out how to do what I want without the whole stack. My understanding from the author(s) of Nav2 is that this is possible. I've tried to distill down what I need as far as I can. I am looking for a solid foothold from which to build. 


## What works, doesn't work, and has not yet been tried

* We are getting a local costmap that can be displayed in rviz2. Not sure yet if it is correct
* I have not done any tuning of the parameters so far

## Links and learnings

    Disclaimer: These are what I think I know. But I can't promise that they are actually (or exactly) correct

* parameter yaml files that are launched from the command like (ros2 run --params-file) can only attach parameters to one node
* unlike ros1, in ros2, parameters are always attached to some node. There are no permanent global parameters
* Lifecycle nodes are those that are designed to be launched and controlled by the Lifecycle processor
    * [Lifecycle node demp](https://github.com/ros2/demos/tree/rolling/lifecycle)
    * [Foxgloves explanation](https://foxglove.dev/blog/how-to-use-ros2-lifecycle-nodes)
* Set the per-node log levels with this addition to your ros2 run
    `ros2 run nav_tools costmap_node --ros-args --params-file src/nav_tools/config/costmap_params.yaml --log-level local_costmap.local_costmap:=DEBUG`



## Running what is here

**Note that this is designed to work with a ros2 nodes publishing /odom and /scan. So do whatever you need to do first to get those.**

```
$ colcon build
$ ros2 run nav_tools costmap_node --ros-args --params-file src/nav_tools/config/costmap_params.yaml 
```
