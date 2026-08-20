# Dependencies

系统环境：Ubuntu 22.04 + ROS 2 Humble。

## ROS 依赖

构建依赖（`find_package`，见 `cpp06_urdf/CMakeLists.txt`）：

```
sudo apt install ros-humble-rclcpp ros-humble-nav-msgs ros-humble-geometry-msgs \
    ros-humble-visualization-msgs ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
```

运行依赖（`display.launch.py` 需要）：

```
sudo apt install ros-humble-rviz2 ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher
```

测试依赖（`BUILD_TESTING` 开启时需要）：

```
sudo apt install ros-humble-ament-lint-auto ros-humble-ament-lint-common \
    ros-humble-ament-cmake-gtest
```

## 第三方库

| 库 | 用途 | 安装方式 |
|----|------|----------|
| Eigen3 | 矩阵运算（MPC QP 构造、参考线） | `sudo apt install libeigen3-dev` |
| OMPL | Reeds-Shepp / Dubins 距离计算（Hybrid A* 启发与 shot） | `sudo apt install libompl-dev` |
| OSQP | 二次规划求解器（MPC 底层） | 源码编译，安装于 `/usr/local` |
| OsqpEigen | OSQP 的 Eigen 封装（MPC 求解） | 源码编译，安装于 `/usr/local` |
| g2o | 图优化框架（TEB 超图优化） | 源码编译，安装于 `/usr/local` |

说明：

- **OSQP**（https://github.com/osqp/osqp）：cmake 构建后 `make install`，默认安装到 `/usr/local`；
- **OsqpEigen**（https://github.com/robotology/osqp-eigen，v0.11.x）：依赖 OSQP 与 Eigen3，同样 cmake 构建安装；
- **g2o**（https://github.com/RainerKuemmerle/g2o）：本项目链接 `g2o::core g2o::stuff g2o::solver_eigen g2o::solver_csparse`，其中 `solver_csparse` 依赖 SuiteSparse，需先执行 `sudo apt install libsuitesparse-dev`；cmake 构建时建议开启 `-DG2O_BUILD_EXAMPLES=OFF`，安装后位于 `/usr/local`（CMakeLists.txt 中 `find_package(g2o REQUIRED HINTS /usr/local)` 已按该路径查找）。


# Build
```
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
. install/setup.bash
ros2 launch cpp06_urdf display.launch.py
```
# 算法调用配置
可在 `include/common/config.h` 中通过 `global_path_strategy`、`reference_line_strategy`、`local_path_strategy` 三个开关灵活组合各层算法，通过 `enable_cover_path` / `enable_edge_path` 切换全覆盖与贴边模式。
