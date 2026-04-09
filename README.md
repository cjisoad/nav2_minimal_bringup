# mobile_robot_nav_bringup

最小 Nav2 导航功能包，面向 Ubuntu 22.04 + ROS 2 Humble，并复用系统通过 apt 安装的 `nav2_bringup`。

## 包内容

- `launch/navigation.launch.py`
  统一启动入口，包含 Nav2 的定位与导航启动文件。
- `config/nav2_params.yaml`
  最小必要参数，适配 2D 差速底盘、AMCL、静态地图、`map/odom/base_link`。
- `maps/map.yaml` 与 `maps/map.pgm`
  地图占位文件。实际使用前替换为真实静态地图。

## 前提

- 安装NAV2 `sudo apt install ros-humble-nav2-bringup ros-humble-navigation2`
- 存在2D雷达扫描，并以 `/scan`发布雷达信息
- 存在里程计，并以 `odom`话题发布里程计信息
- 存在以下TF变换：`map`、`odom`、`base_link`
- 底盘能接收`/cmd_vel`速度命令并移动

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select mobile_robot_nav_bringup
source install/setup.bash
```

## 启动

真机导航：

```bash
ros2 launch mobile_robot_nav_bringup navigation.launch.py
```

显式传入自己的静态地图：

```bash
ros2 launch mobile_robot_nav_bringup navigation.launch.py \
  map:=/absolute/path/to/your_map.yaml
```

如果你想把自己的地图直接放到包内，并且以后启动时不再手动传 `map:=...`，可以这样做：

1. 把地图文件放到 `src/mobile_robot_nav_bringup/maps/`，例如：
   - `src/mobile_robot_nav_bringup/maps/my_lab.yaml`
   - `src/mobile_robot_nav_bringup/maps/my_lab.pgm`
2. 修改 [`navigation.launch.py`](/home/eden/mobile_robot_nav/src/mobile_robot_nav_bringup/launch/navigation.launch.py#L66) 里 `map` 参数的默认值，把：

```python
default_value=os.path.join(pkg_share, 'maps', 'map.yaml')
```

改成：

```python
default_value=os.path.join(pkg_share, 'maps', 'my_lab.yaml')
```

3. 重新编译并（不要漏！！！） source：

```bash
cd /home/eden/mobile_robot_nav
source /opt/ros/humble/setup.bash
colcon build --packages-select mobile_robot_nav_bringup --symlink-install
source install/setup.bash
```

之后就可以直接启动，不再传入 `map` 参数：

```bash
ros2 launch mobile_robot_nav_bringup navigation.launch.py
```

## 仿真验证

推荐直接使用系统里已经安装好的 TurtleBot3 Gazebo 环境做最小验证。

终端 1，启动 Gazebo 仿真：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

终端 2，启动本包的 Nav2：

```bash
source /opt/ros/humble/setup.bash
cd /home/eden/mobile_robot_nav
source install/setup.bash

ros2 launch mobile_robot_nav_bringup navigation.launch.py \
  use_sim_time:=true \
  map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
```

仿真中验证流程：

1. 等待 RViz 打开。
2. 使用 `2D Pose Estimate` 设置初始位姿。
3. 使用 `Nav2 Goal` 发送目标点。
4. 观察机器人是否开始规划并移动。

## 建图与保存地图

这个包现在也提供最小建图入口，复用系统安装的 `slam_toolbox`，不需要把它拷贝到工作空间。

建图启动文件：

- [`slam.launch.py`](/home/eden/mobile_robot_nav/src/mobile_robot_nav_bringup/launch/slam.launch.py)
- [`slam_params.yaml`](/home/eden/mobile_robot_nav/src/mobile_robot_nav_bringup/config/slam_params.yaml)

保存地图启动文件：

- [`save_map.launch.py`](/home/eden/mobile_robot_nav/src/mobile_robot_nav_bringup/launch/save_map.launch.py)

真机建图命令：

```bash
source /opt/ros/humble/setup.bash
cd /home/eden/mobile_robot_nav
source install/setup.bash

ros2 launch mobile_robot_nav_bringup slam.launch.py
```

仿真建图命令：

```bash
source /opt/ros/humble/setup.bash
cd /home/eden/mobile_robot_nav
source install/setup.bash

ros2 launch mobile_robot_nav_bringup slam.launch.py use_sim_time:=true
```

建图时需要保证：

- `/scan` 已存在
- `odom` 已存在
- TF 中存在 `odom`、`base_link`
- 机器人可以被遥控移动完成扫图

扫图完成后，保存地图到本包的 `maps/` 目录：

```bash
source /opt/ros/humble/setup.bash
cd /home/eden/mobile_robot_nav
source install/setup.bash

ros2 launch mobile_robot_nav_bringup save_map.launch.py map_name:=my_map
```

默认会优先保存到当前工作空间源码目录（注意！保存地图并更改launch文件后要编译）：

```bash
/home/eden/mobile_robot_nav/src/mobile_robot_nav_bringup/maps/
```

生成的文件通常是：

- `maps/my_map.yaml`
- `maps/my_map.pgm`

如果要显式指定保存目录：

```bash
ros2 launch mobile_robot_nav_bringup save_map.launch.py \
  save_dir:=/home/eden/mobile_robot_nav/src/mobile_robot_nav_bringup/maps \
  map_name:=my_map
```

## 使用流程

1. 启动底盘驱动、激光、TF、里程计。
2. 启动本包的 `navigation.launch.py`。
3. 在 RViz 中设置 `2D Pose Estimate`，给 AMCL 初始位姿。
4. 用 `Nav2 Goal` 发送导航目标。

## 验证

检查生命周期节点是否激活：

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

检查关键话题：

```bash
ros2 topic list | grep -E '/map|/plan|/cmd_vel|/amcl_pose'
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo map base_link
```

如果机器人不动，优先检查：

- `maps/map.yaml` 是否已替换为真实地图
- `config/nav2_params.yaml` 中 `robot_radius`、速度/加速度限制是否匹配底盘
- 底盘是否订阅 `/cmd_vel`
- `/scan` 的帧是否能正确变换到 `base_link`
