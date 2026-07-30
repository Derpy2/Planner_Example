# ROS2 Cyber Monitor

一个受 Apollo `cyber_monitor` 启发的 ROS2 命令行消息监控工具。可以在终端中实时查看当前 ROS2 节点发布的所有 topic，并像 `cyber_monitor` 一样以层级方式浏览消息内容。

## 功能特性

- **实时 topic 列表**：自动发现并显示当前 ROS2 网络中的所有 topic
- **层级消息浏览**：只显示当前层级的字段，嵌套对象显示 `{...}`，数组显示 `[...]`
- **数组元素切换**：数组只显示一个元素，可用 `n`/`m` 切换上一个/下一个元素
- **增量刷新**：topic 列表只更新变化的部分，不会打断你的操作
- **低依赖**：基于 Python 标准库和 Textual 构建

## 适用 ROS 版本

- ROS 2 Humble
- ROS 2 Iron
- ROS 2 Jazzy
- 其他支持 `rclpy` 的 ROS 2 发行版（理论上兼容，已在 Humble 测试）

## 依赖

### 系统依赖

- Python 3.8+
- ROS 2 环境已正确安装并 source

### Python 依赖

| 包名 | 说明 |
|------|------|
| `rclpy` | ROS 2 Python 客户端库，随 ROS 2 安装 |
| `rosidl_runtime_py` | ROS 2 消息运行时工具，随 ROS 2 安装 |
| `textual` | 终端 UI 框架，需要单独安装 |

## 安装

1. 确保 ROS 2 环境已 source：

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. 安装 Textual：

   ```bash
   pip3 install textual
   ```

   或者如果你使用 conda/venv：

   ```bash
   python3 -m pip install textual
   ```

3. 确认依赖已安装：

   ```bash
   python3 -c "import rclpy, textual"
   ```

## 使用方法

### 直接运行

```bash
python3 monitor.py
```

### 赋予执行权限后运行

```bash
chmod +x monitor.py
./monitor.py
```

### 运行时环境

运行前请确保：

- ROS 2 环境已 source
- 至少有一个 ROS 2 节点在发布消息，否则列表为空或只显示 `/rosout`

## 操作说明

### 界面布局

```
+----------------------------------+-----------------------------------------+
| Topic                            | Detail                                  |
| /odom          nav_msgs/Odometry | /odom > root                            |
| /cmd_vel     geometry_msgs/Twist | > header: {...} (2 fields)              |
| /scan       sensor_msgs/LaserScan |   child_frame_id: "base_link"           |
|                                  |   pose: {...} (2 fields)                |
|                                  |   twist: {...} (2 fields)               |
|                                  |                                         |
|                                  | →/Enter/l:进入 ... n:下一个 m:上一个     |
+----------------------------------+-----------------------------------------+
```

- **左侧 40%**：topic 列表，显示 topic 名和消息类型
- **右侧 60%**：选中 topic 的消息详情，支持层级浏览

### 基础操作

| 步骤 | 操作 |
|------|------|
| 选择 topic | 左侧按 **↑/↓** 移动，按 **Enter** 选中 |
| 聚焦右侧 | 按 **Tab** 或鼠标点击右侧区域 |
| 退出程序 | 按 **Ctrl + C** |

### 右侧层级导航

> 需要先按 **Tab** 让右侧区域获得焦点（右侧边框变绿）。

| 按键 | 作用 |
|------|------|
| **→ / Enter / l** | 进入当前选中的嵌套对象或数组元素 |
| **← / Esc / h** | 返回上一层 |
| **↑ / k** | 在对象字段中向上移动 |
| **↓ / j** | 在对象字段中向下移动 |
| **n** | 数组中显示下一个元素 |
| **m** | 数组中显示上一个元素 |

### 示例浏览流程

假设 `/odom` 消息结构如下：

```text
/odom
├── header
│   ├── stamp
│   │   ├── sec
│   │   └── nanosec
│   └── frame_id
├── child_frame_id
├── pose
└── twist
```

1. 选中 `/odom`，右侧显示：

   ```text
   /odom > root

   > header: {...} (2 fields)
     child_frame_id: "base_link"
     pose: {...} (2 fields)
     twist: {...} (2 fields)
   ```

2. 按 **↓** 选中 `header`，按 **→** 进入：

   ```text
   /odom > header

   > stamp: {...} (2 fields)
     frame_id: "odom"
   ```

3. 按 **→** 进入 `stamp`：

   ```text
   /odom > header.stamp

     sec: 1722345678
     nanosec: 123456789
   ```

4. 按 **←** 返回上一层。

### 数组浏览示例

假设 `/scan` 的 `ranges` 数组有 360 个元素：

```text
/scan > root

  header: {...} (2 fields)
  angle_min: -1.57
  angle_max: 1.57
> ranges: [...] (360 items)
  intensities: [...] (360 items)
```

1. 选中 `ranges`，按 **→** 进入数组视图：

   ```text
   /scan > ranges

   [1/360]: 1.234
   ```

2. 按 **n** 查看下一个元素：

   ```text
   /scan > ranges

   [2/360]: 1.235
   ```

3. 按 **m** 查看上一个元素。

## 注意事项

1. **依赖 ROS 2 环境**  
   运行前必须 `source /opt/ros/<distro>/setup.bash`，否则 `rclpy` 无法导入。

2. **消息解析**  
   工具使用 `rosidl_runtime_py.message_to_ordereddict()` 将 ROS 消息转为 Python 字典。标准消息类型通常都能正常解析；如果解析失败，右侧会显示原始文本。

3. **大数组/大消息**  
   对于 `sensor_msgs/Image`、`sensor_msgs/PointCloud2` 这类包含大量数据的数组，建议只查看 header 等关键字段，避免逐个浏览所有数组元素导致卡顿。

4. **方向键不响应**  
   如果方向键在右侧无效，请使用 `h/j/k/l` 作为备选按键。这是因为右侧的滚动容器可能会消费方向键。

5. **焦点切换**  
   左侧 topic 列表和右侧消息区通过 **Tab** 键切换焦点。只有在右侧有焦点时，层级导航键才会生效。

## 常见问题

### Q1: 运行时报错 `AttributeError: can't set attribute 'subscriptions'`

A: 这个错误是因为类内部变量与 `rclpy.node.Node` 的只读属性重名。当前代码已使用 `self.topic_subs` 避免冲突。如果你自己修改代码，请不要使用 `self.subscriptions`。

### Q2: topic 列表为空

A: 检查是否已 source ROS 2 环境，以及是否有节点在发布消息。可以用以下命令验证：

```bash
ros2 topic list
```

### Q3: 右侧消息不刷新

A: 确认已选中 topic 并且有 publisher 在发布数据。某些 topic 可能发布频率很低，看起来像是没刷新。

## 文件结构

```text
cpp06_urdf/src/scripts/
├── monitor.py      # 主程序
└── README.md       # 本文档
```

## 授权

本工具作为示例代码提供，可根据项目需要自由修改使用。
