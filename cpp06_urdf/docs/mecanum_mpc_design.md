# 麦轮底盘 MPC 局部控制设计方案

## 1. 设计目标

本文档说明在当前 `cpp06_urdf` 项目中，如果 MPC 的控制对象是麦克纳姆轮底盘，应如何设计局部控制器。

目标：

1. MPC 支持麦轮底盘的全向移动能力。
2. 控制输出使用 ROS 标准 `geometry_msgs::msg::Twist`：

   ```text
   linear.x  = vx
   linear.y  = vy
   angular.z = wz
   ```

3. 在不改变 `LocalPlannerBase` 接口的前提下实现麦轮 MPC。
4. 第一版先输出底盘速度，不直接输出四个轮子的电机速度。
5. 后续可扩展为带轮速约束、加速度约束和 QP 求解器的完整 MPC。

## 2. 当前项目适配点

当前局部规划器接口为：

```cpp
virtual geometry_msgs::msg::Twist getControlCmd() = 0;
```

因此麦轮 MPC 可以直接复用该接口：

```cpp
cmd.linear.x = vx;
cmd.linear.y = vy;
cmd.angular.z = wz;
```

当前 `PurePursuit` 只使用 `linear.x` 和 `angular.z`，麦轮 MPC 需要额外使用 `linear.y`。

## 3. 麦轮 MPC 状态与控制量

### 3.1 状态量

第一版建议状态量：

```text
x = [px, py, yaw]
```

| 状态 | 含义 |
|---|---|
| `px` | 世界坐标系下 x 位置 |
| `py` | 世界坐标系下 y 位置 |
| `yaw` | 机器人航向角 |

### 3.2 控制量

麦轮底盘控制量：

```text
u = [vx, vy, wz]
```

| 控制量 | 含义 |
|---|---|
| `vx` | 车体坐标系 x 方向速度，前进为正 |
| `vy` | 车体坐标系 y 方向速度，左移为正 |
| `wz` | 绕 z 轴角速度，逆时针为正 |

## 4. 麦轮底盘预测模型

车体速度到世界速度的转换：

```text
px_dot  = vx * cos(yaw) - vy * sin(yaw)
py_dot  = vx * sin(yaw) + vy * cos(yaw)
yaw_dot = wz
```

离散化模型：

```text
px_{k+1}  = px_k + (vx_k * cos(yaw_k) - vy_k * sin(yaw_k)) * dt
py_{k+1}  = py_k + (vx_k * sin(yaw_k) + vy_k * cos(yaw_k)) * dt
yaw_{k+1} = yaw_k + wz_k * dt
```

这是第一版麦轮 MPC 的核心模型。

## 5. 是否直接输出四轮轮速

第一版不建议在局部规划器中直接输出四个轮子的角速度。推荐分层：

```text
LocalPlannerMPC
  -> 输出 Twist(vx, vy, wz)
  -> 底盘控制器 / ros2_control
  -> 转换为四个麦轮角速度
  -> 电机驱动
```

如果后续需要直接计算轮速，可使用麦轮逆运动学：

```text
w_fl = (1 / r) * (vx - vy - (Lx + Ly) * wz)
w_fr = (1 / r) * (vx + vy + (Lx + Ly) * wz)
w_rl = (1 / r) * (vx + vy - (Lx + Ly) * wz)
w_rr = (1 / r) * (vx - vy + (Lx + Ly) * wz)
```

说明：

| 符号 | 含义 |
|---|---|
| `w_fl` | 左前轮角速度 |
| `w_fr` | 右前轮角速度 |
| `w_rl` | 左后轮角速度 |
| `w_rr` | 右后轮角速度 |
| `r` | 轮子半径 |
| `Lx` | 机器人中心到前/后轮的 x 向距离 |
| `Ly` | 机器人中心到左/右轮的 y 向距离 |

注意：不同麦轮安装方向会影响符号，最终需要根据 URDF、控制器和电机方向实测标定。

## 6. 参考轨迹设计

参考轨迹建议：

```text
ref[k] = [ref_x, ref_y, ref_yaw]
```

生成方式：

1. 从 `smoothed_local_` 中找到距离当前状态最近的路径点。
2. 从最近点开始向后选取 `horizon` 个路径点。
3. 如果路径点不足，则重复最后一个点。
4. 参考 yaw 使用相邻路径点切线方向：

   ```text
   ref_yaw = atan2(y_{i+1} - y_i, x_{i+1} - x_i)
   ```

### yaw 模式选择

麦轮底盘可支持两种 yaw 策略：

| 模式 | 说明 | 适用场景 |
|---|---|---|
| `tangent` | 车头沿路径切线方向 | 普通路径跟踪，调试简单 |
| `keep_current` | 保持当前车头方向，通过 `vx/vy` 跟踪路径 | 需要车头固定的全向移动场景 |
| `goal` | 逐渐对齐目标 yaw | 停车入位、对接任务 |

第一版建议使用 `tangent`，后续再通过参数切换。

## 7. 控制约束设计

麦轮 MPC 需要限制：

```text
vx_min <= vx <= vx_max
vy_min <= vy <= vy_max
wz_min <= wz <= wz_max
```

推荐第一版参数：

| 参数 | 建议值 | 含义 |
|---|---:|---|
| `mecanumMpcDt` | `0.1` | 预测步长 |
| `mecanumMpcHorizon` | `10` | 预测步数 |
| `mecanumMpcMinVx` | `0.0` | 最小前进速度，第一版不倒车 |
| `mecanumMpcMaxVx` | `0.4` | 最大前进速度 |
| `mecanumMpcMaxVy` | `0.3` | 最大横向速度 |
| `mecanumMpcMaxWz` | `1.0` | 最大角速度 |
| `mecanumMpcVxSamples` | `5` | vx 采样数 |
| `mecanumMpcVySamples` | `5` | vy 采样数 |
| `mecanumMpcWzSamples` | `11` | wz 采样数 |

## 8. 代价函数设计

建议代价函数：

```text
J =
  Qx   * (px_pred - px_ref)^2
+ Qy   * (py_pred - py_ref)^2
+ Qyaw * normalize(yaw_pred - yaw_ref)^2
+ Rvx  * vx^2
+ Rvy  * vy^2
+ Rwz  * wz^2
+ Rdvx * (vx - last_vx)^2
+ Rdvy * (vy - last_vy)^2
+ Rdwz * (wz - last_wz)^2
+ Qt   * terminal_error
```

推荐权重：

| 参数 | 建议值 | 含义 |
|---|---:|---|
| `mecanumMpcQx` | `8.0` | x 位置误差权重 |
| `mecanumMpcQy` | `8.0` | y 位置误差权重 |
| `mecanumMpcQYaw` | `2.0` | 航向误差权重 |
| `mecanumMpcRVx` | `0.1` | vx 控制代价 |
| `mecanumMpcRVy` | `0.25` | vy 控制代价 |
| `mecanumMpcRWz` | `0.15` | wz 控制代价 |
| `mecanumMpcRdVx` | `0.4` | vx 平滑代价 |
| `mecanumMpcRdVy` | `0.6` | vy 平滑代价 |
| `mecanumMpcRdWz` | `0.3` | wz 平滑代价 |
| `mecanumMpcTerminalWeight` | `2.0` | 终端误差权重 |

建议让 `Rvy > Rvx`，因为麦轮横移通常滑动更明显、效率更低，只有必要时才使用横移。

## 9. 求解方式

### 第一版：离散采样搜索

不引入外部优化库，采样控制量：

```text
vx ∈ [0.0, 0.1, 0.2, 0.3, 0.4]
vy ∈ [-0.3, -0.15, 0.0, 0.15, 0.3]
wz ∈ [-1.0, -0.8, ..., 0.0, ..., 0.8, 1.0]
```

一共：

```text
5 * 5 * 11 = 275 组控制
```

若 `horizon = 10`，单周期预测约：

```text
275 * 10 = 2750 步
```

当前项目规模下可以接受。

### 第二版：QP / NLP 优化

后续可改为控制序列优化：

```text
u = [vx0, vy0, wz0, vx1, vy1, wz1, ..., vxN, vyN, wzN]
```

并使用 OSQP、CasADi 或其他求解器。但第一版不建议直接引入，避免依赖和调试复杂度过高。

## 10. 类结构设计

建议 `LocalPlannerMPC`：

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
    double vx;
    double vy;
    double wz;
  };

  struct ReferencePoint {
    double x;
    double y;
    double yaw;
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

  Control last_control_;
};
```

## 11. `getControlCmd()` 流程

```text
getControlCmd()
|
|-- 检查 smoothed_local_ 是否有效
|-- 从 current_pose_ 提取 x/y/yaw
|-- 生成 reference trajectory
|-- solveMPC()
|     |-- 遍历 vx/vy/wz 采样
|     |-- 用麦轮模型预测 horizon 步
|     |-- 计算 cost
|     |-- 保存最低 cost 控制量
|-- 限幅 vx/vy/wz
|-- 保存 last_control_
|-- 输出 Twist(vx, vy, wz)
```

## 12. 需要修改的文件

| 文件 | 修改内容 |
|---|---|
| `include/common/config.h` | 添加麦轮 MPC 参数 |
| `include/planner/local_planner/mpc.h` | 完善 `LocalPlannerMPC` 声明 |
| `src/planner/local_planner/mpc.cpp` | 新增麦轮 MPC 实现 |
| `src/local_planner_node.cpp` | 增加 `local_planner_type` 参数，支持选择 `mpc` |
| `CMakeLists.txt` | 将 `src/planner/local_planner/mpc.cpp` 加入编译 |

## 13. 实现计划表

| 阶段 | 任务 | 输出 |
|---|---|---|
| 1 | 添加麦轮 MPC 配置参数 | `config.h` 中新增参数 |
| 2 | 完善 `mpc.h` | 定义 `State`、`Control`、`ReferencePoint` 和私有函数 |
| 3 | 新建 `mpc.cpp` | 实现麦轮 MPC 主逻辑 |
| 4 | 实现状态读取 | 从 `current_pose_` 获取 `x/y/yaw` |
| 5 | 实现参考轨迹生成 | 根据 `smoothed_local_` 生成 horizon 参考点 |
| 6 | 实现麦轮预测模型 | 使用 `vx/vy/wz` 推演状态 |
| 7 | 实现代价函数 | 位置、航向、控制量、平滑项、终端项 |
| 8 | 实现采样搜索求解器 | 遍历 `vx/vy/wz`，选择最低 cost |
| 9 | 输出 Twist | 填充 `linear.x`、`linear.y`、`angular.z` |
| 10 | 接入节点参数 | 支持 `local_planner_type:=mpc` |
| 11 | 接入 CMake | 编译 `mpc.cpp` |
| 12 | 编译验证 | `colcon build --packages-select cpp06_urdf` 通过 |
| 13 | 运行验证 | `/cmd_vel` 输出合理的 `vx/vy/wz` |
| 14 | RViz 验证 | 观察跟踪路径效果 |
| 15 | 调参 | 调整速度限制、权重和采样数量 |

## 14. 验证方法

编译：

```bash
cd /home/cyancloud/ros/test
source /opt/ros/humble/setup.bash
colcon build --packages-select cpp06_urdf
```

运行 MPC：

```bash
ros2 run cpp06_urdf local_planner_node --ros-args -p local_planner_type:=mpc
```

查看控制输出：

```bash
ros2 topic echo /cmd_vel
```

重点检查：

```text
linear.x  是否在 [0.0, 0.4]
linear.y  是否在 [-0.3, 0.3]
angular.z 是否在 [-1.0, 1.0]
```

## 15. 第一版验收标准

| 验收项 | 标准 |
|---|---|
| 编译 | 项目编译通过 |
| 节点启动 | `local_planner_node` 能正常启动 |
| 控制输出 | `/cmd_vel` 包含合理的 `linear.x/linear.y/angular.z` |
| 速度限幅 | 三个控制量不超过设定范围 |
| 横向能力 | 需要横向修正时 `linear.y` 不为 0 |
| 到达终点 | 接近终点后速度归零 |
| 兼容性 | 原有 `pure_pursuit` 不受影响 |

## 16. 后续增强方向

| 增强项 | 说明 |
|---|---|
| 加速度约束 | 限制 `vx/vy/wz` 的变化率 |
| 轮速约束 | 根据四轮最大转速限制底盘速度 |
| 支持不同 yaw 模式 | `tangent`、`keep_current`、`goal` |
| 引入 OSQP | 实现真正优化意义上的线性 MPC |
| 发布预测轨迹 | 新增 `/mpc_predicted_path` 方便 RViz 调试 |
| 对接 ros2_control | 由底盘控制器完成 Twist 到轮速转换 |
