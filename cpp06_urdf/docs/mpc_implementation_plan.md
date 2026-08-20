# MPC 局部规划器实现计划

## 1. 当前项目现状

| 模块 | 当前情况 |
|---|---|
| `include/planner/local_planner/mpc.h` | 已有 `LocalPlannerMPC` 类，但只是空实现，返回空 `Twist` |
| `LocalPlannerBase` | 已提供 `getControlCmd()`、`setCurrentPose()`、`setSmoothedPath()` |
| `local_planner_node.cpp` | 当前硬编码使用 `PURE_PURSUIT` |
| `CMakeLists.txt` | 只编译了 `pure_pursuit.cpp`，没有 `mpc.cpp` |
| 控制输出 | 使用 `geometry_msgs::msg::Twist`，即线速度 `v` 和角速度 `w` |
| 车辆模型 | 当前接口更适合先实现差速/简化单车运动学模型 |

## 2. 推荐实现方案

优先实现一个轻量级运动学 MPC：不引入额外 QP 求解器，使用离散控制采样和轨迹滚动预测。

推荐原因：

1. 当前项目没有 OSQP、qpOASES、CasADi 等优化库依赖。
2. 项目输出是 `Twist`，适合差速/简化单车运动学模型：

   ```text
   x_{k+1}   = x_k + v_k * cos(yaw_k) * dt
   y_{k+1}   = y_k + v_k * sin(yaw_k) * dt
   yaw_{k+1} = yaw_k + w_k * dt
   ```

3. 实现简单、稳定、容易调试。
4. 后续可以在该接口基础上升级为基于 OSQP 的线性 MPC。

## 3. MPC 控制目标

MPC 每个控制周期执行以下流程：

1. 获取当前机器人状态：

   ```text
   x, y, yaw
   ```

2. 从平滑后的局部路径 `smoothed_local_` 中生成预测参考轨迹：

   ```text
   ref_x[k], ref_y[k], ref_yaw[k], ref_v[k]
   ```

3. 枚举一组候选控制量：

   ```text
   u[k] = [v[k], w[k]]
   ```

4. 使用运动学模型预测未来轨迹。
5. 根据代价函数选出最优控制量。
6. 发布第一组控制量：

   ```cpp
   cmd.linear.x = best_v;
   cmd.angular.z = best_w;
   ```

## 4. 核心代价函数设计

建议代价函数：

```text
J =
  Q_xy  * 位置误差
+ Q_yaw * 航向误差
+ Q_v   * 速度误差
+ R_v   * 线速度控制代价
+ R_w   * 角速度控制代价
+ Rd_v  * 线速度变化率代价
+ Rd_w  * 角速度变化率代价
```

具体形式：

```text
position_error = (x_pred - x_ref)^2 + (y_pred - y_ref)^2
yaw_error      = normalize(yaw_pred - yaw_ref)^2
speed_error    = (v_pred - v_ref)^2
control_cost   = v^2 + w^2
smooth_cost    = (v_k - v_{k-1})^2 + (w_k - w_{k-1})^2
```

## 5. 推荐参数

第一版可以先放在 `include/common/config.h` 中，后续再改为 ROS 参数。

| 参数 | 建议值 | 含义 |
|---|---:|---|
| `mpcDt` | `0.1` | 预测时间步长 |
| `mpcHorizon` | `10` | 预测步数 |
| `mpcMaxLinearSpeed` | `0.4` | 最大线速度 |
| `mpcMinLinearSpeed` | `0.0` | 最小线速度 |
| `mpcMaxAngularSpeed` | `1.0` | 最大角速度 |
| `mpcLinearSamples` | `5` | 线速度采样数量 |
| `mpcAngularSamples` | `11` | 角速度采样数量 |
| `mpcQPosition` | `8.0` | 位置误差权重 |
| `mpcQYaw` | `2.0` | 航向误差权重 |
| `mpcQSpeed` | `0.5` | 速度误差权重 |
| `mpcRV` | `0.2` | 线速度控制权重 |
| `mpcRW` | `0.1` | 角速度控制权重 |
| `mpcRdV` | `0.5` | 线速度变化率权重 |
| `mpcRdW` | `0.2` | 角速度变化率权重 |

## 6. 详细实施计划表

| 阶段 | 任务 | 修改文件 | 说明 |
|---|---|---|---|
| 1 | 新增 MPC 参数 | `include/common/config.h` | 添加预测步长、预测时域、速度限制、采样数量、代价权重 |
| 2 | 完善 MPC 类接口 | `include/planner/local_planner/mpc.h` | 声明状态结构体、控制结构体、参考点结构体、核心私有函数 |
| 3 | 新建 MPC 源文件 | `src/planner/local_planner/mpc.cpp` | 实现完整 MPC 逻辑 |
| 4 | 实现当前状态获取 | `mpc.cpp` | 从 `current_pose_` 提取 `x/y/yaw` |
| 5 | 实现参考轨迹生成 | `mpc.cpp` | 从 `smoothed_local_` 按 horizon 生成参考点 |
| 6 | 实现运动学预测模型 | `mpc.cpp` | 使用差速/简化单车模型预测未来状态 |
| 7 | 实现代价函数 | `mpc.cpp` | 计算位置、航向、速度、控制量和平滑代价 |
| 8 | 实现控制采样搜索 | `mpc.cpp` | 枚举 `v/w` 组合，寻找最低 cost |
| 9 | 加入安全保护 | `mpc.cpp` | 处理空路径、路径点不足、异常 yaw、速度限幅等情况 |
| 10 | 接入构建系统 | `CMakeLists.txt` | 将 `src/planner/local_planner/mpc.cpp` 加入 `local_planner_node` |
| 11 | 支持选择局部规划器 | `src/local_planner_node.cpp` | 增加参数 `local_planner_type`，可选 `pure_pursuit` 或 `mpc` |
| 12 | 编译验证 | 工作空间根目录 | 执行 `colcon build --packages-select cpp06_urdf` |
| 13 | 运行验证 | ROS 运行环境 | 检查节点是否启动、`/cmd_vel` 是否正常输出 |
| 14 | 调参 | `config.h` 或 ROS 参数 | 调整权重、速度、预测时域、采样密度 |
| 15 | 可视化验证 | RViz | 查看 `/local_path` 和机器人跟踪效果 |

## 7. 建议新增/修改的文件结构

```text
cpp06_urdf/
├── docs/
│   └── mpc_implementation_plan.md
├── include/
│   └── planner/
│       └── local_planner/
│           └── mpc.h
├── src/
│   └── planner/
│       └── local_planner/
│           └── mpc.cpp
├── include/common/config.h
├── src/local_planner_node.cpp
└── CMakeLists.txt
```

## 8. `mpc.h` 设计建议

`LocalPlannerMPC` 建议设计为：

```cpp
class LocalPlannerMPC : public LocalPlannerBase {
 public:
  LocalPlannerMPC();

  geometry_msgs::msg::Twist getControlCmd() override;

 private:
  struct State {
    double x;
    double y;
    double yaw;
  };

  struct Control {
    double v;
    double w;
  };

  struct ReferencePoint {
    double x;
    double y;
    double yaw;
    double v;
  };

  State getCurrentState() const;

  std::vector<ReferencePoint> generateReferenceTrajectory(
      const State& current_state) const;

  State predictState(const State& state, const Control& control,
                     double dt) const;

  double calculateCost(const std::vector<State>& predicted_states,
                       const std::vector<Control>& controls,
                       const std::vector<ReferencePoint>& reference) const;

  Control solveMPC(const State& current_state,
                   const std::vector<ReferencePoint>& reference) const;

  double normalizeAngle(double angle) const;
  double clamp(double value, double min_value, double max_value) const;
};
```

## 9. `getControlCmd()` 执行流程

```text
getControlCmd()
|
|-- 检查 smoothed_local_ 是否为空或路径点是否不足
|
|-- 获取当前状态 x/y/yaw
|
|-- 生成预测参考轨迹
|
|-- 调用 solveMPC()
|     |
|     |-- 枚举 v/w 控制组合
|     |-- 预测 horizon 步
|     |-- 计算 cost
|     |-- 保存 cost 最小的控制量
|
|-- 输出 Twist
```

## 10. 控制采样策略

第一版可以采用常值控制采样：

```text
v ∈ [0.0, 0.1, 0.2, 0.3, 0.4]
w ∈ [-1.0, -0.8, ..., 0.0, ..., 0.8, 1.0]
```

每组 `(v, w)` 在整个 horizon 内保持不变，预测未来轨迹并计算 cost。

优点：

- 简单
- 不依赖优化库
- 稳定
- 适合当前项目阶段

后续可以升级为控制序列优化：

```text
[v0, w0], [v1, w1], ..., [vN-1, wN-1]
```

## 11. `local_planner_node.cpp` 接入方式

当前代码硬编码：

```cpp
local_planner_ = local_planner::LocalPlannerFactory::CreateLocalPlanner(
    local_planner::LocalPlannerType::PURE_PURSUIT);
```

建议改成 ROS 参数：

```cpp
std::string local_planner_type =
    this->declare_parameter<std::string>("local_planner_type", "pure_pursuit");

if (local_planner_type == "mpc") {
  local_planner_ = local_planner::LocalPlannerFactory::CreateLocalPlanner(
      local_planner::LocalPlannerType::MPC);
} else {
  local_planner_ = local_planner::LocalPlannerFactory::CreateLocalPlanner(
      local_planner::LocalPlannerType::PURE_PURSUIT);
}
```

启动时可以选择：

```bash
ros2 run cpp06_urdf local_planner_node --ros-args -p local_planner_type:=mpc
```

## 12. CMake 修改计划

当前 `local_planner_node` 只编译：

```cmake
src/planner/local_planner/pure_pursuit.cpp
```

需要加入：

```cmake
src/planner/local_planner/mpc.cpp
```

修改后：

```cmake
add_executable(local_planner_node
  src/local_planner_node.cpp
  include/local_planner_node.h
  src/planner/reference_line/slide_window.cpp
  src/planner/reference_line/b_spline.cpp
  src/planner/reference_line/bezier.cpp
  src/planner/local_planner/pure_pursuit.cpp
  src/planner/local_planner/mpc.cpp
)
```

## 13. 编译验证命令

```bash
cd /home/cyancloud/ros/test
source /opt/ros/humble/setup.bash
colcon build --packages-select cpp06_urdf
```

## 14. 运行验证建议

启动局部规划器时选择 MPC：

```bash
ros2 run cpp06_urdf local_planner_node --ros-args -p local_planner_type:=mpc
```

观察输出：

```bash
ros2 topic echo /cmd_vel
```

检查速度是否合理：

```text
linear.x: 0.0 ~ 0.4
angular.z: -1.0 ~ 1.0
```

## 15. 第一版验收标准

| 验收项 | 标准 |
|---|---|
| 编译 | `colcon build --packages-select cpp06_urdf` 通过 |
| 节点启动 | `local_planner_node` 能正常启动 |
| 参数选择 | 可通过 `local_planner_type:=mpc` 启用 MPC |
| 控制输出 | `/cmd_vel` 能持续输出非异常速度 |
| 速度限制 | `linear.x` 和 `angular.z` 不超过配置上限 |
| 路径跟踪 | 机器人能沿 `/local_path` 前进 |
| 到达终点 | 接近终点后速度归零 |
| 兼容性 | 原有 `pure_pursuit` 仍可使用 |

## 16. 后续增强方向

| 增强项 | 说明 |
|---|---|
| 引入 OSQP | 实现真正的线性 MPC / QP 求解 |
| 加入加速度约束 | 限制 `dv/dt`、`dw/dt` |
| 加入障碍物代价 | 对接地图或 costmap |
| 支持倒车 | 允许 `v < 0` |
| 动态参数 | 将 MPC 参数从 `config.h` 改为 ROS 参数 |
| 增加单元测试 | 测试角度归一化、预测模型、cost 计算 |
| 发布预测轨迹 | 新增 `/mpc_predicted_path` 方便 RViz 调试 |
