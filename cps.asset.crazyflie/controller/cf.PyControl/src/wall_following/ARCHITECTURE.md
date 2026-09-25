# 墙面跟随模块架构说明

## 当前代码流程梳理

### 1. 执行流程概述

```
启动 ROS2 Server → 启动 REST 服务 → 执行墙面跟随脚本
      ↓                   ↓                    ↓
 连接物理无人机      Flask REST API      多机协调控制
```

### 2. 文件职责分析

#### 2.1 核心算法层
- **`wall_following.py`** (359行)
  - 职责：实现墙面跟随的核心算法逻辑
  - 功能：状态转换、速度计算、转向控制
  - 无ROS2依赖，纯Python算法实现
  - 状态：FORWARD, HOVER, TURN_TO_FIND_WALL, TURN_TO_ALIGN_TO_WALL, FORWARD_ALONG_WALL, ROTATE_AROUND_WALL, ROTATE_IN_CORNER, FIND_CORNER

#### 2.2 状态机可视化层
- **`wall_following_state_machine.py`** (321行)
  - 职责：包装核心算法，提供状态机图片生成
  - 功能：调用`wall_following.py`的算法，生成状态机图表
  - 依赖：`wall_following.py`, `statemachine`, `DotGraphMachine`
  - 输出：PNG状态机图片到`webview/wall_following/img/`

#### 2.3 ROS2节点层
- **`wall_following_multiranger.py`** (411行)
  - 职责：ROS2节点实现，连接传感器和控制器
  - 功能：
    - 订阅多传感器数据（激光雷达、里程计）
    - 调用墙面跟随算法
    - 发布Hover控制命令
    - 垂直冲突避免（多机协同）
  - 依赖：`wall_following.py`, `wall_following_state_machine.py`
  - ROS2话题：
    - 输入：`/cfXXX/odom`, `/cfXXX/scan`
    - 输出：`/cfXXX/cmd_hover`
    - 服务：`/cfXXX/stop_wall_following`

#### 2.4 配置管理层
- **`wall_following_config.py`** (74行)
  - 职责：集中管理墙面跟随配置参数
  - 功能：
    - 参数定义（速度、转向率、方向、高度等）
    - REST请求解析
    - 转换为ROS2节点参数
  - 参数：`max_turn_rate`, `max_forward_speed`, `direction`, `hover_height`, `delay`, etc.

#### 2.5 节点生命周期管理层
- **`wall_following_runner.py`** (139行)
  - 职责：管理墙面跟随节点的启动和停止
  - 组件：
    - `ConfigurableWallFollowingMultiranger`: 参数化节点封装
    - `WallFollowingOrchestrator`: 节点生命周期管理
  - 功能：
    - 动态创建和销毁ROS2节点
    - 参数注入和覆盖
    - 安全停止和清理

#### 2.6 多机协调控制层
- **`auto_wall_follow_state_machine.py`** (377行)
  - 职责：多机墙面跟随任务的自动化执行
  - 组件：
    - `WallFollowAutoRunner`: 单机控制流程
    - `MultiDroneWallFollowManager`: 多机协调管理器
  - 执行模式：
    - `parallel`: 并行执行（所有无人机同时启动）
    - `sequential`: 顺序执行（一架接一架）
    - `staggered`: 错开执行（固定时间间隔启动）
  - 流程：`activate_idle → takeoff → start_wall_follow → run → stop_wall_follow → land`

### 3. 调用关系图

```
auto_wall_follow_state_machine.py (多机控制)
    ↓ HTTP REST API
cf-ctrl-service-ros2.py (Flask服务)
    ↓ routes.py (/wall_follow/start, /wall_follow/stop)
cf_sm.py (状态机：begin_wall_following, stop_wall_following)
    ↓
wall_following_runner.py (WallFollowingOrchestrator)
    ↓ ROS2 Executor
wall_following_multiranger.py (ROS2 Node)
    ↓ 调用算法
wall_following_state_machine.py (状态机包装)
    ↓ 核心算法
wall_following.py (墙面跟随算法)
```

### 4. 数据流

```
传感器数据:
LaserScan (/scan) → wall_following_multiranger.py
Odometry (/odom)  → wall_following_multiranger.py
    ↓ 距离和航向
wall_following.py (算法计算)
    ↓ velocity_x, velocity_y, yaw_rate
wall_following_multiranger.py
    ↓ Hover消息
/cmd_hover → Crazyflie控制器
```

### 5. 配置流

```
auto_wall_follow_state_machine.py
    ↓ JSON配置 (hover_height, speed, direction, etc.)
REST API (/wall_follow/start)
    ↓
wall_following_config.py (WallFollowingConfig)
    ↓ to_wall_following_kwargs()
wall_following_runner.py (ConfigurableWallFollowingMultiranger)
    ↓ ROS2参数
wall_following_multiranger.py (declare_parameter)
```

## 重构计划

### 目标
1. 将所有墙面跟随相关文件整合到`wall_following/`文件夹
2. 消除代码重复
3. 清晰分层：算法层 → 节点层 → 管理层 → 控制层
4. 保持向后兼容

### 新文件结构

```
controller/cf.PyControl/src/wall_following/
├── __init__.py                        # 模块导出
├── ARCHITECTURE.md                    # 架构说明（本文档）
├── README.md                          # 使用说明
├── wall_following.py                  # [保持] 核心算法
├── wall_following_state_machine.py    # [保持] 状态机包装
├── wall_following_config.py           # [移入] 配置管理
├── wall_following_multiranger.py      # [移入] ROS2节点
├── wall_following_runner.py           # [移入] 生命周期管理
└── auto_wall_follow_runner.py         # [重命名+移入] 多机控制（原auto_wall_follow_state_machine.py）
```

### 重构原则
1. **单一职责**：每个文件只负责一个明确的功能
2. **依赖注入**：通过参数传递配置，而不是硬编码
3. **清晰命名**：文件名和类名明确表达职责
4. **分层架构**：低层不依赖高层，高层可以依赖低层

### 下一步行动
1. 移动文件到`wall_following/`文件夹
2. 更新所有导入路径
3. 创建清晰的`__init__.py`导出接口
4. 更新外部引用（cf-ctrl-service-ros2.py, routes.py等）
5. 测试确保功能正常

# Crazyflie 墙面跟随模块

完整的Crazyflie墙面跟随(Wall Following)功能实现，支持单机和多机协同建图。

## 📁 模块结构

```
wall_following/
├── __init__.py                        # 模块导出接口
├── README.md                          # 本文档
├── ARCHITECTURE.md                    # 详细架构说明
├── REFACTORING_NOTES.md              # 重构分析笔记
│
├── wall_following.py                  # 核心算法层
├── wall_following_state_machine.py    # 状态机可视化层
├── wall_following_config.py           # 配置管理层
├── wall_following_multiranger.py      # ROS2节点层
└── wall_following_runner.py           # 生命周期管理层
```

## 🚀 快速开始

### 1. 基本使用

```python
from wall_following import (
    WallFollowingConfig,
    WallFollowingOrchestrator
)

# 创建配置
config = WallFollowingConfig(
    robot_prefix="/cf231",
    max_forward_speed=0.2,
    max_turn_rate=0.5,
    direction="right",
    hover_height=0.5
)

# 启动墙面跟随
orchestrator = WallFollowingOrchestrator(executor)
orchestrator.start(config)

# 停止墙面跟随
orchestrator.stop()
```

### 2. 多机协同使用

通过 REST API 方式（推荐）：

```bash
# 运行多机墙面跟随脚本
python3 auto_wall_follow_state_machine.py \
    --wall_follow_duration 30 \
    --execution_mode parallel
```

## 📋 模块说明

### 1. 核心算法层 (`wall_following.py`)

**职责**：实现墙面跟随的核心算法逻辑

**主要类**：
- `WallFollowing`: 墙面跟随算法实现
  - 状态：FORWARD, HOVER, TURN_TO_FIND_WALL, TURN_TO_ALIGN_TO_WALL, FORWARD_ALONG_WALL, ROTATE_AROUND_WALL, ROTATE_IN_CORNER, FIND_CORNER
  - 方向：LEFT, RIGHT

**特点**：
- 无ROS2依赖，纯Python实现
- 可独立测试和使用
- 基于Bitcraze官方算法改进

**示例**：
```python
from wall_following import WallFollowing

wf = WallFollowing(
    max_turn_rate=0.5,
    max_forward_speed=0.2,
    wall_following_direction=WallFollowing.WallFollowingDirection.RIGHT
)

# 获取控制命令
vx, vy, yaw_rate, state = wf.wall_follower(
    front_range=0.8,
    side_range=0.5,
    current_heading=0.0,
    wall_following_direction=WallFollowing.WallFollowingDirection.RIGHT,
    time_outer_loop=0.0
)
```

### 2. 状态机可视化层 (`wall_following_state_machine.py`)

**职责**：包装核心算法，提供状态机图片生成

**主要类**：
- `WallFollowingStateMachine`: 状态机包装器
  - 自动同步算法状态
  - 生成PNG状态图
  - 支持状态转换追踪

**特点**：
- 基于 `python-statemachine` 库
- 实时生成状态机图表
- 图片保存到 `webview/wall_following/img/{drone_id}/`

**示例**：
```python
from wall_following import WallFollowingStateMachine, WallFollowing

sm = WallFollowingStateMachine(
    drone_id="cf231",
    image_output_base_dir="/path/to/output",
    max_turn_rate=0.5,
    max_forward_speed=0.2
)

# 更新墙面跟随
vx, vy, yaw, state = sm.update_wall_following(
    front_range, side_range, heading, direction, time_now
)

# 生成状态机图片
sm.write_sm_graph()
```

### 3. 配置管理层 (`wall_following_config.py`)

**职责**：集中管理墙面跟随配置参数

**主要类**：
- `WallFollowingConfig`: 配置数据类
  - 飞行参数：速度、转向率、高度
  - 行为参数：方向、延迟
  - 路径参数：图片输出目录

**特点**：
- 使用 `dataclass` 定义
- 支持从REST请求构建
- 支持环境变量配置路径

**配置参数**：
| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `robot_prefix` | str | (必需) | 机器人前缀，如 "/cf231" |
| `max_forward_speed` | float | 0.2 | 最大前向速度 (m/s) |
| `max_turn_rate` | float | 0.5 | 最大转向速率 (rad/s) |
| `direction` | str | "right" | 墙面跟随方向 ("left"/"right") |
| `hover_height` | float | 0.5 | 悬停高度 (m) |
| `delay` | float | 5.0 | 起飞后延迟启动时间 (s) |
| `use_sim_time` | bool | False | 是否使用仿真时间 |
| `image_output_base_dir` | str | "" | 图片输出基础目录 |

**示例**：
```python
from wall_following import WallFollowingConfig

# 直接创建
config = WallFollowingConfig(
    robot_prefix="/cf231",
    max_forward_speed=0.2,
    direction="right",
    hover_height=0.5
)

# 从REST请求创建
config = WallFollowingConfig.from_request(
    robot_prefix="/cf231",
    request_data={"hover_height": 0.6, "direction": "left"}
)

# 获取图片目录
img_dir = config.get_image_dir("cf231")
```

### 4. ROS2节点层 (`wall_following_multiranger.py`)

**职责**：ROS2节点实现，连接传感器和控制器

**主要类**：
- `WallFollowingMultiranger`: ROS2节点
  - 订阅传感器数据（激光雷达、里程计）
  - 调用墙面跟随算法
  - 发布Hover控制命令
  - 多机垂直冲突避免

**ROS2接口**：
| 类型 | 话题/服务 | 消息类型 | 说明 |
|------|-----------|----------|------|
| 订阅 | `/{prefix}/odom` | Odometry | 里程计数据 |
| 订阅 | `/{prefix}/scan` | LaserScan | 激光雷达数据 |
| 订阅 | `/{peer}/odom` | Odometry | 其他无人机里程计 |
| 发布 | `/{prefix}/cmd_hover` | Hover | 悬停控制命令 |
| 服务 | `/{prefix}/stop_wall_following` | Trigger | 停止墙面跟随 |

**多机避让功能**：
- 自动检测垂直冲突（上方无人机下洗风险）
- 侧向避让策略
- 自动返回原路径

**示例**：
```python
import rclpy
from wall_following import WallFollowingMultiranger

rclpy.init()
node = WallFollowingMultiranger(
    robot_prefix="/cf231",
    hover_height=0.5,
    max_forward_speed=0.2
)
rclpy.spin(node)
```

### 5. 生命周期管理层 (`wall_following_runner.py`)

**职责**：管理墙面跟随节点的启动和停止

**主要类**：
- `ConfigurableWallFollowingMultiranger`: 参数化节点封装
- `WallFollowingOrchestrator`: 节点生命周期管理器
  - 动态创建/销毁ROS2节点
  - 参数注入和覆盖
  - 安全停止和清理

**示例**：
```python
from wall_following import WallFollowingConfig, WallFollowingOrchestrator
import rclpy
from rclpy.executors import MultiThreadedExecutor

rclpy.init()
executor = MultiThreadedExecutor()

# 创建编排器
orchestrator = WallFollowingOrchestrator(executor)

# 创建配置
config = WallFollowingConfig(robot_prefix="/cf231", hover_height=0.5)

# 启动节点
orchestrator.start(config)

# 运行
executor.spin()

# 停止节点
orchestrator.stop()
```
