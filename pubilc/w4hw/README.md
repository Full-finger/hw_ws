# 二维地图构建算法实现说明

## 项目概述

本项目实现了一套完整的二维占据栅格地图构建算法，能够将三维激光雷达点云数据转换为可用于机器人导航的二维地图。算法支持单帧地图和累计地图两种模式，并特别处理了**斜坡地形**和**动态物体**两大实际应用中的关键问题。

---

## 算法架构

### 整体流程

```
原始点云 (3D)
    │
    ▼
┌─────────────────────────────────────┐
│         预处理阶段                    │
│  • 高度滤波（去除地面点）              │
│  • 范围滤波（限制处理范围）            │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│         单帧地图构建                  │
│  • 投影到2D平面                       │
│  • Bresenham射线投射                  │
│  • 生成临时占据栅格                    │
└─────────────────────────────────────┘
    │
    ▼
┌─────────────────────────────────────┐
│         累计地图融合                  │
│  • 贝叶斯更新（对数几率）              │
│  • 坡度检测与自适应阈值                │
│  • 动态物体检测与滤除                  │
│  • 形态学后处理                       │
└─────────────────────────────────────┘
    │
    ▼
   二维占据栅格地图
   (-1=未知, 0=空闲, 100=占据)
```

---

## 核心模块详解

### 模块1：单帧地图构建 (`SingleFrameMapper`)

**功能**：仅使用当前一帧点云生成临时地图，不累积历史信息。

**算法步骤**：
1. **高度滤波**：保留指定高度范围（默认0.1-2.0米）的点，滤除地面和过高点
2. **2D投影**：丢弃Z坐标，只保留X、Y坐标
3. **射线投射**：使用Bresenham算法绘制从机器人到每个障碍物的射线
4. **栅格标记**：射线上的点标记为"空闲"，终点标记为"占据"

**使用场景**：
- 实时避障（不需要完整地图）
- 局部路径规划
- 算法调试和验证

**代码示例**：
```python
config = MapConfig(size_m=30, resolution=0.05)
mapper = SingleFrameMapper(config)
grid_map = mapper.process_frame(points_3d, robot_pose)
```

---

### 模块2：累计地图构建 (`AccumulatedMapper`)

**功能**：融合多帧观测，构建全局一致的地图。

**核心技术**：

#### 2.1 贝叶斯更新（对数几率表示）

使用对数几率（log-odds）表示每个栅格被占据的概率：

```
l(m) = log(p(m) / (1 - p(m)))
```

更新公式：
```
l_new = l_old + Δl_occ   (观测到占据)
l_new = l_old + Δl_free  (观测到空闲)
```

**优势**：
- 避免概率值饱和（接近0或1）
- 数值稳定性好
- 增量更新方便

#### 2.2 形态学后处理

使用OpenCV的形态学操作优化地图：
- **开运算**（腐蚀+膨胀）：去除孤立噪点
- **闭运算**（膨胀+腐蚀）：填充小空洞

---

### 模块3：坡度处理 (`SlopeHandler`)

**问题描述**：传统2D建图将斜坡上的点误判为障碍物（墙），导致机器人认为不可通行。

**解决方案**：

#### 核心思路
1. 检测机器人前方区域的坡度角度
2. 根据坡度动态调整高度判断阈值
3. 坡面上的点标记为地面而非障碍物

#### 算法流程

```python
# 1. 检测坡度
slope_angle = estimate_slope(points_3d, robot_pose)

# 2. 获取自适应阈值
adaptive_threshold = get_adaptive_height_threshold(slope_angle)

# 3. 分类点云
if rel_height < adaptive_threshold:
    # 坡面上的点 → 不标记为障碍物
    mark_as_ground(point)
else:
    # 真正的障碍物
    mark_as_obstacle(point)
```

#### 自适应阈值策略

| 坡度角度 | 高度阈值 | 说明 |
|---------|---------|------|
| < 5°    | 0.15m   | 平坦地面，严格判断 |
| 5-10°   | 0.20m   | 轻度斜坡，适当放宽 |
| 10-15°  | 0.25m   | 中度斜坡，进一步放宽 |
| > 15°   | 0.30m   | 陡坡，可能标记为不可通行 |

**效果**：机器人可以识别并爬上15°以内的斜坡，而不会将其误判为墙壁。

---

### 模块4：动态物体滤除 (`DynamicObjectFilter`)

**问题描述**：环境中移动的人或车辆会在累计地图中留下"残影"（拖尾效应）。

**解决方案**：

#### 核心思路
1. 维护多帧历史观测（默认10帧）
2. 对每个网格统计占据频率
3. 新出现的点需要持续出现多帧才被确认为静态物体

#### 算法流程

```python
class DynamicObjectFilter:
    def classify_dynamic_static(current_scan):
        for each point in current_scan:
            # 1. 查询历史占据概率
            historical_prob = get_historical_occupancy(point)
            
            # 2. 检查持续性
            persistence = check_persistence(point)
            
            # 3. 分类决策
            if historical_prob < 0.3 and persistence < 5:
                dynamic_points.append(point)  # 认为是动态
            else:
                static_points.append(point)   # 认为是静态
```

#### 动态物体检测效果

| 物体类型 | 停留时间 | 处理结果 |
|---------|---------|---------|
| 快速移动的人 | < 3帧 | 被滤除，不加入地图 |
| 短暂停留的物体 | 3-5帧 | 待确认，暂不加入 |
| 长时间静止的物体 | > 5帧 | 认定为静态，加入地图 |
| 快速移动的车 | < 3帧 | 被滤除，无残影 |

**关键参数**：
- `history_frames=10`：保留10帧历史
- `occupancy_threshold=0.3`：占据概率阈值30%
- `persistence_threshold=5`：需要持续5帧才确认为静态

---

## 数据结构定义

### RobotPose（机器人位姿）

```python
@dataclass
class RobotPose:
    x: float = 0.0          # X坐标（米）
    y: float = 0.0          # Y坐标（米）
    z: float = 0.0          # Z坐标（米）
    yaw: float = 0.0        # 偏航角（弧度）
    roll: float = 0.0       # 横滚角（弧度）
    pitch: float = 0.0      # 俯仰角（弧度）
```

### MapConfig（地图配置）

```python
@dataclass
class MapConfig:
    size_m: float = 100.0       # 地图边长（米）
    resolution: float = 0.05    # 分辨率（米/像素）
    origin_x: float = -50.0     # 原点X坐标
    origin_y: float = -50.0     # 原点Y坐标
```

**地图尺寸计算**：
- 宽度（像素）= size_m / resolution
- 例如：100m / 0.05m/pixel = 2000像素

---

## 使用指南

### 快速开始

#### 示例1：单帧地图构建

```python
from map_builder import MapConfig, SingleFrameMapper, RobotPose
import numpy as np

# 1. 配置地图
config = MapConfig(size_m=30, resolution=0.05, origin_x=-15, origin_y=-15)

# 2. 创建构建器
mapper = SingleFrameMapper(config)

# 3. 准备数据
points_3d = load_your_pointcloud()  # Nx3 numpy数组
robot_pose = RobotPose(x=0, y=0, z=0.3, yaw=0)

# 4. 处理单帧
grid_map = mapper.process_frame(points_3d, robot_pose)

# 5. 保存结果
mapper.save_as_image(grid_map, "output.png")
```

#### 示例2：累计地图（含坡度和动态物体处理）

```python
from map_builder import MapConfig, AccumulatedMapper, RobotPose

# 1. 创建累计地图构建器
mapper = AccumulatedMapper(config)

# 2. 循环处理多帧
for frame_id in range(num_frames):
    points_3d = get_next_frame()     # 获取新的一帧点云
    robot_pose = get_current_pose()   # 获取当前机器人位姿
    
    # 综合更新（同时处理坡度和动态物体）
    grid_map = mapper.update_combined(points_3d, robot_pose)
    
    # 可选：定期保存
    if frame_id % 10 == 0:
        mapper.save_as_image(f"map_frame_{frame_id}.png")

# 3. 最终输出
mapper.save_as_image("final_map.png")
mapper.save_height_map_image("height_map.png")
```

### 参数调优建议

#### 地图分辨率选择

| 场景 | 分辨率 | 地图大小 | 适用情况 |
|------|--------|---------|---------|
| 室内小场景 | 0.02-0.05m | 50x50m | 需要精细障碍物信息 |
| 室外中等场景 | 0.05-0.10m | 100x100m | 平衡精度和内存 |
| 室外大场景 | 0.10-0.20m | 500x500m | 快速建图，精度要求低 |

#### 坡度处理参数

```python
slope_handler = SlopeHandler(
    max_traversable_slope=15.0  # 最大可通行坡度（度）
    # 15° 适用于大多数轮式机器人
    # 履带式机器人可调至 20-25°
    # 双足机器人可调至 5-10°
)
```

#### 动态物体滤除参数

```python
dynamic_filter = DynamicObjectFilter(
    history_frames=10,          # 历史帧数（越大越稳定，但响应慢）
    occupancy_threshold=0.3,   # 占据阈值（30%）
    persistence_threshold=5     # 持续帧数（5帧确认）
)
```

---

## 输出结果说明

### 占据栅格地图 (`accumulated_map.png`)

- **黑色**：占据区域（障碍物）
- **白色**：空闲区域（可通行）
- **灰色**：未知区域（未探索）

### 高度图 (`height_map.png`)

- **颜色映射**：从低（蓝紫色）到高（红色）
- 可用于识别地形起伏和斜坡区域

