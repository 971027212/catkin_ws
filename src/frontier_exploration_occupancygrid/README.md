# Frontier-based exploration
It is a ROS package that implements frontier-based approach to explore unknown environment. The algorithm emploied single robot. It uses occupancy girds as a map representation.The packgae only has one ROS nodes:

  - frontier planner node

Note: This package was written to accomplish a engineering project. For solve the problem that clean robot travels in large-scale environment(e.g. super mall)

## 1. Requirements
The package has been tested on ROS Melodic, it should work on other distributions like Kinetic. The following requirements are needed before installing the package:

1- You should have installed a ROS distribution (kinetic or later. Recommended is either melodic or kinetic).

2- Created a workspace.

3- Installed the "gmapping" ROS package: you can do that by typing the following command in the terminal:

```sh
$ sudo apt-get install ros-melodic-gmapping
```
4- Install ROS navigation stack. You can do that with the following command (assuming in melodic):
```sh
$ sudo apt-get install ros-melodic-navigation
```
6-You should have/install the following local planner module or use your own local planner:
```sh
$ sudo apt-get install ros-melodic-eband-local-planner
```

## 2. Installation
Download the package and place it inside the ```/src``` folder in your workspace. And then compile using ```catkin_make```.
## 3. Configuring Your Robot
This package provides an exploration strategy for single robot. However, for it to work, you should have set your robots ready using the [navigation stack](http://wiki.ros.org/navigation). 

### 3.1. Frontier planner node topics and parameters
#### Subscribe Topics
1. "/map": The global map(nav_msgs::OccupancyGrid)

#### Publish Topics
1. "/frontier": All frontier cells (PointArray)
2. "/centroids": All centroids(PointArray)
3. "/inflated_map": The Inflated map of raw map(nav_msgs::OccupancyGrid)
4. "/move_base/goal": The current goal using move_base action(move_base_msgs::MoveBaseActionGoal)
5. "/path_vis": A visualization marker that traces the sequence of explored goals once mapping is complete

#### Parameters
1. "obstacle_inflation": (float) inflation radius(meter)
2. "map_revolution": (float) global map revolution
3. "cmd_topic": (string) the robot received velocity topic
4. "robot_base_frame": (string) robot base frame_id
5. "goal_tolerance": The tolerance for arriving current goal(meter)
6. "obstacle_tolerance":if robot's distance to nearest obstacle less than this value, the robot won't rotate
7. "rotate_speed": (float) the robot rotate speed (default 1.0 rad/s for quicker reorientation)

After the frontier list becomes empty the planner also publishes the `path_vis` marker, which can be displayed in RViz to
evaluate how the robot covered the environment.

### 3.2. Setting up the navigation stack on the robots
The ```move_base_node``` node, which brings up the navigation stack on the robot, must be running. This package generates target exploration goals, the robot must be able to receive these points and move. Additionally, each robot must have a global and local cost maps. All of these are proivded from the ```move_base_node```. 

### 3.3. mapping
This package(frontier_exploration) uses [gmapping](http://wiki.ros.org/gmapping) slam by default, you can change gmapping to your slam (like cartographer or hector) with the nav_msgs::OccupancyGrid output

## 4. Run
### 1. Firstly, source your workspace
```sh
$ source {path_to_your_workspace}
```

### 2. Launch the node
```sh
$ roslaunch frontier_exploration frontier_exploration.launch
```
The exploration action will start at launching node. In addition, you can change different gazebo world in world dir.

### 3. Exporting the exploration metrics and curves
- 每次移动后都会把覆盖率和累计路径长度写入 CSV，默认路径为 `/tmp/frontier_metrics.csv`，可以通过 `metrics_output_path` 参数（或 launch 文件中的 arg）改到别的目录。
- 生成曲线时不需要知道源码所在的绝对路径，直接使用 `rosrun`：
  ```sh
  $ rosrun frontier_exploration plot_curves.py /tmp/frontier_metrics.csv --output ~/exploration_curves.png
  ```
  这条命令会自动定位 `plot_curves.py`，然后在用户目录生成 PNG。也可以把 CSV/PNG 的路径替换成你自己的文件。

如果你更喜欢手动定位脚本，可以先运行 `roscd frontier_exploration` 再执行 `python3 scripts/plot_curves.py ...`，这样就不会再遇到「路径不存在」的错误。

## 5. Video and Paper citation
[Youtube Video](https://www.youtube.com/watch?v=yTYMJIM6Itw)

[Bilibili Video](https://www.bilibili.com/video/BV1n5411m7Lt/?spm_id_from=333.999.0.0&vd_source=d4cc7b88d2637e29008d0af5794a07f0)

[Paper](https://ieeexplore.ieee.org/abstract/document/613851)






