# Get the actual pose of robot from tf2
获取当前机器人在地图的位姿，也就是base_footprint在参考坐标系map下的位姿。

## 启动
单独启动
```bashrc
ros2 launch robot_pose_publisher pose_publisher.launch.py
```
整体启动是在task_server包中：
```bashrc
ros2 launch task_server luxshare_robot.launch.py
```

## 发布的数据到topic /pose与/curr_pose数据结果一样，只是topic类型不同
- 1. topic名称 /pose
type:geometry_msgs::msg::PoseWithCovarianceStamped
自动建图需要用到此topic

- 2. topic名称 /curr_pose
type:geometry_msgs::msg::PoseStamped
