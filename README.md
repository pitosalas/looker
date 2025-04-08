# looker - run nav2::costmap2dROS

## Motivation

I want to learn about costmaps. I have an idea for an application that simply uses costmaps by themselves for navigation and goal seeking. But my personal goal is to learn about costmaps, lifecycle nodes, and more advanced ros2 features. So there are other ways to solve my nominal goal, like running the whole nav2 stack, and starting with an empty map. But my goal is to either fiture out how to do what I want without the whole stack. My understanding from the author(s) of Nav2 is that this is possible. 

## Approach

I've tried to distill down what I need as far as I can. If you look at the config/costmap_params.yaml you will see a very slimmed down yaml file with a weird node name: "local_costmap/local_costmap:". I am looking for a solid foothold from which to build. While this lanches, there are numerous warnings that I am trying to fix.

## Running what is here

Note that this is designed to work with a ros2 nodes publishing /odom and /scan. So do whatever you need to do first to get those.

```
$ colcon build
$ ros2 run looker costmap_node  --ros-args --params-file src/looker/config/costmap_params.yaml 
```

## Request

If you know how I should move the project forward I would love to hear from you!