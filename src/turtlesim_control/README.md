
# Turtlesim Control

基于 ROS2 turtlesim 的二次开发包，支持自动避边界、多种运动模式和遥控操作。

## 依赖

```bash
sudo apt install ros-humble-turtlesim
```

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select turtlesim_control_interfaces turtlesim_control
source install/setup.bash
```

## 一键启动

```bash
ros2 launch turtlesim_control turtlesim_system.launch.py
```

启动后小海龟自动向前直线运动，碰到边界自动转向。

## 运动模式

| 模式 | 说明 | 切换命令 |
|------|------|----------|
| 0 | 直线运动（默认） | `ros2 service call /switch_mode turtlesim_control_interfaces/srv/SwitchMode "{mode: 0}"` |
| 1 | 正弦曲线 | `ros2 service call /switch_mode turtlesim_control_interfaces/srv/SwitchMode "{mode: 1}"` |
| 2 | 方波路径 | `ros2 service call /switch_mode turtlesim_control_interfaces/srv/SwitchMode "{mode: 2}"` |

## 遥控器

在 launch 启动的终端中直接按键操作：

| 按键 | 功能 |
|------|------|
| W | 前进 |
| S | 后退 |
| A | 左转 |
| D | 右转 |
| X | 停止 |
| 0 | 切换直线模式 |
| 1 | 切换正弦模式 |
| 2 | 切换方波模式 |
| Q | 退出 |

## 参数配置

编辑 `config/params.yaml` 可修改速度等参数：

```yaml
/**:
  ros__parameters:
    mode: 0
    linear_speed: 1.5
    sine_linear_speed: 1.0
    sine_amplitude: 3.0
    square_linear_speed: 1.0
    square_side_length: 3.0
    boundary_min: 1.0
    boundary_max: 10.0
```

修改后重新 launch 生效。

## 节点与话题

```
/turtlesim_node     ← 仿真器
/control_node       ← 自动控制 + 模式切换服务
/teleop_node        ← 遥控器

话题:
  /turtle1/cmd_vel    (geometry_msgs/Twist)
  /turtle1/pose       (turtlesim/Pose)

服务:
  /switch_mode        (turtlesim_control_interfaces/srv/SwitchMode)
```
