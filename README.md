### Quick start

下载`dev`分支的最新代码，编译之后运行
```bash
roslaunch min_snap min_snap.launch
```
在Rviz中按G之后，使用3D-nav-goal插件设置目标点，当给定一个Z小于零的点时开始规划。
或者直接运行`test_goal.sh`脚本

> 该代码开发环境为Ubuntu 20.04 ROS noetic
> 如有bug，欢迎联系my_wang@zju.edu.cn

### 文件导航

``` bash
src
├── planner
│   ├── min_snap
│   ├── obvp
│   ├── plan_env
│   ├── traj_server
│   └── traj_utils
└── uav_simulator
    ├── local_sensing
    ├── map_generator
    ├── my_visualization
    ├── so3_control
    ├── so3_quadrotor_simulator
    └── Utils
        ├── cmake_utils
        ├── multi_map_server
        ├── odom_visualization
        ├── pose_utils
        ├── quadrotor_msgs
        ├── rviz_plugins
        ├── uav_utils
        └── waypoint_generator
```
### 功能包描述
* `planner`
  * `min_snap`：求解最小化snap的轨迹
  * `obvp`：使用OBVP方法求解两点最优轨迹
  * `plan_env*`：环境感知，暂时无用
  * `traj_server`：轨迹服务器
  * `traj_utils*`：轨迹工具，包含轨迹类
* `uav_simulator`
  * `local_sensing*`：局部环境感知，仿相机
  * `map_generator`：生成地图
  * `my_visualization`：轨迹可视化
  * `so3_control*`：SO3控制器
  * `so3_quadrotor_simulator*`：SO3无人机仿真器
  * `Utils*`：其他辅助工具包

注：带`*`标注的功能包来自[ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)

### 计划

* 修复优化求解中轨迹物理量不连续的问题
* 尝试使用不同的mapping来解决倒转180度导致的yaw问题
* 增加地图膨胀等建图代码

### 更新日志

* 2021.11.26
  * 增加了`LBFGS_Lite`的[头文件](https://github.com/ZJU-FAST-Lab/LBFGS-Lite)，用于求解无约束优化问题
  * 增加了min_snap优化求解类
  * 增加了`README.md`
* 2021.11.21
  * 修改了编译时Eigen报错的问题
  * 修改了`so3_control`包中的mapping方法，更改为Hopf Fibration
* 2021.11.18
  * 修改了之前编译的各种warning，更新了飞机模型
  * 增加了`plan_env`代码作为参考，但是在该版本中无用
  * 增加了`local_sensing`包，仿真相机的建图效果，未膨胀
* 2021.11.07
  * 更新了`quadrotor_msgs`和`odom_visualization`包的代码
  * 在launch里面增加了平均速度的参数，用于修改时间分配的时间
  * 修正了`min_snap`代码中计算矩阵Q的错误
* 2021.10.05
  * 增加了`map_generator`作为地图生成
* 2021.10.03
  * 更改了时间分配的方法，修复了目标点距离过近时导致的时间分配过短问题
* 2021.09.06
  * 完成`min_snap`的闭式求解代码，`traj_server`新增了可视化轨迹代码
* 2021.09.05
  * 将之前的`plan_manage`更名为`obvp`，新增加了`min_snap`包用于实现多目标轨迹生成
  * 将两个方法的轨迹可视化分开，增加了snap的可视化
* 2021.09.04
  * `OBVP`问题解决，可以作为一个完整的OBVP项目
  * jerk不连续，时间自动优化
  * 替换了`Utils/rviz_plugins`插件，用于显示和发布3Dgoal
* 2021.09.01
  * 增加了`my_visualization`用于可视化
  * 更改了`traj_utils`代码，修复了轨迹生成的Bug
* 2021.08.24
  * 新建项目，部分仿真和工具包代码来自开源项目[Ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)。
