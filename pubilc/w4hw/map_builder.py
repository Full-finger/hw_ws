#!/usr/bin/env python3
"""
二维地图构建算法 - 完整实现
包含：单帧地图、累计地图、坡度处理、动态物体滤除
"""

import numpy as np
import cv2
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import warnings
warnings.filterwarnings('ignore')


# ============================================================
# 第一部分：数据结构定义
# ============================================================

@dataclass
class RobotPose:
    """机器人位姿"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw])


@dataclass
class MapConfig:
    """地图配置参数"""
    size_m: float = 100.0          # 地图大小（米）
    resolution: float = 0.05       # 分辨率（米/像素）
    origin_x: float = -50.0        # 原点X
    origin_y: float = -50.0        # 原点Y
    
    @property
    def width(self) -> int:
        return int(self.size_m / self.resolution)
    
    @property
    def height(self) -> int:
        return int(self.size_m / self.resolution)


# ============================================================
# 第二部分：单帧地图构建器
# ============================================================

class SingleFrameMapper:
    """
    单帧地图构建器
    仅使用当前帧点云生成地图，不累积历史信息
    """
    
    def __init__(self, config: MapConfig):
        self.config = config
        self.resolution = config.resolution
        self.width = config.width
        self.height = config.height
        self.origin_x = config.origin_x
        self.origin_y = config.origin_y
        
    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转像素坐标"""
        px = int((x - self.origin_x) / self.resolution)
        py = int((y - self.origin_y) / self.resolution)
        return px, py
    
    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """像素坐标转世界坐标"""
        x = self.origin_x + px * self.resolution
        y = self.origin_y + py * self.resolution
        return x, y
    
    def process_frame(self, points_3d: np.ndarray, robot_pose: RobotPose,
                      min_height: float = 0.1, max_height: float = 2.0) -> np.ndarray:
        """
        处理单帧点云，生成2D占据栅格地图
        
        Args:
            points_3d: Nx3的点云数组
            robot_pose: 机器人位姿
            min_height: 最小障碍物高度（米）
            max_height: 最大障碍物高度（米）
        
        Returns:
            grid_map: 单帧地图，-1=未知，0=空闲，100=占据
        """
        # 初始化单帧地图（全未知）
        grid_map = np.full((self.height, self.width), -1, dtype=np.int8)
        
        if len(points_3d) == 0:
            return grid_map
        
        # 1. 高度滤波：只保留感兴趣高度的点
        heights = points_3d[:, 2]
        height_mask = (heights >= min_height) & (heights <= max_height)
        obstacles = points_3d[height_mask]
        
        if len(obstacles) == 0:
            return grid_map
        
        # 2. 投影到2D（取XY坐标）
        obstacles_xy = obstacles[:, :2]
        
        # 3. 射线投射（Bresenham算法）
        robot_px, robot_py = self.world_to_pixel(robot_pose.x, robot_pose.y)
        
        # 将障碍物点映射到像素坐标
        obs_pixels = []
        for point in obstacles_xy:
            px, py = self.world_to_pixel(point[0], point[1])
            if 0 <= px < self.width and 0 <= py < self.height:
                obs_pixels.append((px, py))
        
        # 使用OpenCV的LineIterator进行射线投射（优化版本）
        for obs_px, obs_py in obs_pixels:
            # 绘制从机器人到障碍物的线段（空闲区域）
            line_points = self._bresenham_line(robot_px, robot_py, obs_px, obs_py)
            
            # 线段上的点标记为空闲（除了最后一个点）
            for i in range(len(line_points) - 1):
                x, y = line_points[i]
                if 0 <= x < self.width and 0 <= y < self.height:
                    grid_map[y, x] = 0  # 空闲
            
            # 障碍物点标记为占据
            grid_map[obs_py, obs_px] = 100  # 占据
        
        return grid_map
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham线段生成算法"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def to_occupancy_msg(self, grid_map: np.ndarray) -> Dict:
        """转换为ROS OccupancyGrid格式的数据"""
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin_x': self.origin_x,
            'origin_y': self.origin_y,
            'data': grid_map.flatten().tolist()
        }
    
    def save_as_image(self, grid_map: np.ndarray, filename: str = "single_frame_map.png"):
        """保存为图像文件"""
        # 归一化到0-255
        map_vis = np.clip(grid_map, 0, 100).astype(np.uint8)
        map_vis = (map_vis * 2.55).astype(np.uint8)
        
        # 颜色映射：黑色=占据，白色=空闲，灰色=未知
        colored = cv2.applyColorMap(255 - map_vis, cv2.COLORMAP_JET)
        
        cv2.imwrite(filename, colored)
        print(f"单帧地图已保存到 {filename}")


# ============================================================
# 第三部分：累计地图构建器（含坡度处理）
# ============================================================

class SlopeHandler:
    """
    坡度处理器
    用于检测和处理上坡场景，避免将坡面误判为墙
    """
    
    def __init__(self, max_traversable_slope: float = 15.0):
        """
        Args:
            max_traversable_slope: 最大可通行坡度（度）
        """
        self.max_slope_rad = np.radians(max_traversable_slope)
        self.max_slope_tan = np.tan(self.max_slope_rad)
        self.ground_height_map = {}  # 存储每个区域的地面高度
        self.slope_map = {}          # 存储每个区域的坡度
        
    def estimate_slope(self, points_3d: np.ndarray, 
                       center_x: float, center_y: float,
                       radius: float = 2.0) -> Tuple[float, float, float]:
        """
        估计局部区域的坡度
        
        Args:
            points_3d: 点云数据
            center_x, center_y: 中心坐标
            radius: 局部区域半径
        
        Returns:
            (slope_angle, slope_direction, ground_height)
            slope_angle: 坡度角度（度）
            slope_direction: 坡度方向（弧度）
            ground_height: 中心点地面高度
        """
        if len(points_3d) < 50:
            return 0.0, 0.0, 0.0
        
        # 提取局部点云
        distances = np.sqrt((points_3d[:, 0] - center_x)**2 + 
                           (points_3d[:, 1] - center_y)**2)
        local_points = points_3d[distances < radius]
        
        if len(local_points) < 50:
            return 0.0, 0.0, 0.0
        
        # 找到最低点作为潜在地面
        min_z_idx = np.argmin(local_points[:, 2])
        min_z_point = local_points[min_z_idx]
        
        # 提取局部点云中地面附近的点
        ground_mask = local_points[:, 2] < (min_z_point[2] + 0.2)
        ground_points = local_points[ground_mask]
        
        if len(ground_points) < 20:
            return 0.0, 0.0, min_z_point[2]
        
        # 使用RANSAC拟合平面
        X = ground_points[:, :2]
        z = ground_points[:, 2]
        
        # 最小二乘拟合平面 z = a*x + b*y + c
        A = np.c_[X, np.ones(len(X))]
        coeffs, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
        a, b, c = coeffs
        
        # 计算坡度角度和方向
        slope_magnitude = np.sqrt(a**2 + b**2)
        slope_angle = np.degrees(np.arctan(slope_magnitude))
        slope_direction = np.arctan2(b, a)
        
        # 计算中心点的地面高度
        ground_height = a * center_x + b * center_y + c
        
        # 存储到地图中
        grid_key = (int(center_x / 0.5), int(center_y / 0.5))
        self.slope_map[grid_key] = slope_angle
        self.ground_height_map[grid_key] = ground_height
        
        return slope_angle, slope_direction, ground_height
    
    def is_traversable(self, slope_angle: float) -> bool:
        """判断坡度是否可通行"""
        return slope_angle <= self.max_slope_rad
    
    def get_adaptive_height_threshold(self, slope_angle: float) -> float:
        """
        根据坡度获取自适应的高度阈值
        
        坡度越大，阈值越高，避免将坡面误判为障碍物
        """
        if slope_angle < 5:
            return 0.15  # 15cm
        elif slope_angle < 10:
            return 0.20  # 20cm
        elif slope_angle < 15:
            return 0.25  # 25cm
        else:
            return 0.30  # 30cm（陡坡但仍然可通行）


class DynamicObjectFilter:
    """
    动态物体滤除器
    使用多种策略检测和滤除移动物体（人、其他车等）
    """
    
    def __init__(self, 
                 history_frames: int = 10,
                 occupancy_threshold: float = 0.3,
                 persistence_threshold: int = 5):
        """
        Args:
            history_frames: 保留的历史帧数
            occupancy_threshold: 占据概率阈值
            persistence_threshold: 物体需要持续存在的帧数
        """
        self.history_frames = history_frames
        self.occupancy_threshold = occupancy_threshold
        self.persistence_threshold = persistence_threshold
        
        # 历史观测队列
        self.observation_history = deque(maxlen=history_frames)
        
        # 每个网格的占据计数
        self.occupancy_counts = np.zeros((1000, 1000), dtype=np.int32)  # 临时大小
        
    def update_history(self, current_scan: np.ndarray, robot_pose: RobotPose):
        """
        更新历史观测
        
        Args:
            current_scan: 当前扫描的障碍物点（Nx2）
            robot_pose: 机器人位姿
        """
        # 将当前扫描转换到全局坐标系
        global_scan = self._transform_to_global(current_scan, robot_pose)
        
        # 添加到历史队列
        self.observation_history.append({
            'scan': global_scan,
            'timestamp': len(self.observation_history)
        })
    
    def _transform_to_global(self, points_xy: np.ndarray, robot_pose: RobotPose) -> np.ndarray:
        """将局部坐标系点转换到全局坐标系"""
        if len(points_xy) == 0:
            return points_xy
        
        cos_yaw = np.cos(robot_pose.yaw)
        sin_yaw = np.sin(robot_pose.yaw)
        
        # 旋转矩阵
        R = np.array([[cos_yaw, -sin_yaw],
                     [sin_yaw, cos_yaw]])
        
        # 转换
        global_points = points_xy @ R.T + np.array([robot_pose.x, robot_pose.y])
        
        return global_points
    
    def classify_dynamic_static(self, 
                                  current_scan: np.ndarray,
                                  robot_pose: RobotPose,
                                  resolution: float = 0.05) -> Tuple[np.ndarray, np.ndarray]:
        """
        分类动态点和静态点
        
        Returns:
            static_points: 静态点（用于地图更新）
            dynamic_points: 动态点（排除）
        """
        if len(self.observation_history) < 2:
            return current_scan, np.empty((0, 2))
        
        global_scan = self._transform_to_global(current_scan, robot_pose)
        
        # 将点云离散化到网格
        grid_scan = {}
        for point in global_scan:
            grid_x = int(point[0] / resolution)
            grid_y = int(point[1] / resolution)
            key = (grid_x, grid_y)
            grid_scan[key] = grid_scan.get(key, 0) + 1
        
        # 分析每个网格的历史信息
        static_mask = []
        dynamic_mask = []
        
        for i, point in enumerate(global_scan):
            grid_x = int(point[0] / resolution)
            grid_y = int(point[1] / resolution)
            key = (grid_x, grid_y)
            
            # 检查历史中该网格是否经常被占据
            historical_occupancy = self._get_historical_occupancy(key)
            
            # 如果是新出现的点且历史中没有，可能是动态物体
            if historical_occupancy < self.occupancy_threshold:
                # 需要多帧确认
                persistence = self._check_persistence(key, grid_scan[key])
                if persistence < self.persistence_threshold:
                    dynamic_mask.append(i)
                else:
                    static_mask.append(i)
            else:
                static_mask.append(i)
        
        # 更新计数
        self._update_occupancy_counts(grid_scan)
        
        # 返回分类结果
        static_points = global_scan[static_mask] if static_mask else np.empty((0, 2))
        dynamic_points = global_scan[dynamic_mask] if dynamic_mask else np.empty((0, 2))
        
        # 可视化动态点（可选）
        self._visualize_dynamic_points(dynamic_points)
        
        return static_points, dynamic_points
    
    def _get_historical_occupancy(self, grid_key: Tuple[int, int]) -> float:
        """获取网格的历史占据概率"""
        grid_x, grid_y = grid_key
        if grid_x < 0 or grid_x >= 1000 or grid_y < 0 or grid_y >= 1000:
            return 0.0
        
        total_observations = len(self.observation_history)
        if total_observations == 0:
            return 0.0
        
        # 简单实现：检查历史中该网格出现的频率
        # 实际应用中可以维护一个更复杂的贝叶斯滤波器
        count = self.occupancy_counts[grid_y, grid_x]
        
        return min(1.0, count / total_observations)
    
    def _check_persistence(self, grid_key: Tuple[int, int], current_count: int) -> int:
        """检查物体的持续性"""
        # 简化实现：返回当前计数
        # 实际应用中应该检查多帧连续出现
        return current_count
    
    def _update_occupancy_counts(self, grid_scan: Dict):
        """更新占据计数"""
        # 衰减旧计数
        self.occupancy_counts = (self.occupancy_counts * 0.9).astype(np.int32)
        
        # 增加新计数
        for (grid_x, grid_y), count in grid_scan.items():
            if 0 <= grid_x < 1000 and 0 <= grid_y < 1000:
                self.occupancy_counts[grid_y, grid_x] += count
    
    def _visualize_dynamic_points(self, dynamic_points: np.ndarray):
        """可视化动态点（用于调试）"""
        # 可以在这里添加可视化代码
        pass


class AccumulatedMapper:
    """
    累计地图构建器
    融合多帧观测，构建全局一致的地图
    
    特性：
    1. 贝叶斯更新融合多帧信息
    2. 集成坡度处理，避免坡面误判
    3. 动态物体滤除，消除移动物体残影
    """
    
    def __init__(self, config: MapConfig):
        self.config = config
        self.resolution = config.resolution
        self.width = config.width
        self.height = config.height
        self.origin_x = config.origin_x
        self.origin_y = config.origin_y
        
        # 占据概率地图（对数几率形式）
        self.log_odds = np.zeros((self.height, self.width))
        prior_prob = 0.5
        self.log_odds.fill(np.log(prior_prob / (1 - prior_prob)))
        
        # 最终的占据栅格地图（0-100）
        self.grid_map = np.full((self.height, self.width), -1, dtype=np.int8)
        
        # 高度图（用于坡度处理）
        self.height_map = np.full((self.height, self.width), -100.0, dtype=np.float32)
        
        # 坡度处理器
        self.slope_handler = SlopeHandler(max_traversable_slope=15.0)
        
        # 动态物体滤波器
        self.dynamic_filter = DynamicObjectFilter(
            history_frames=10,
            occupancy_threshold=0.3,
            persistence_threshold=5
        )
        
        # 传感器模型参数
        self.l_occ = np.log(0.8 / (1 - 0.8))   # 占据对数几率增量
        self.l_free = np.log(0.2 / (1 - 0.2))   # 空闲对数几率增量
        
    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转像素坐标"""
        px = int((x - self.origin_x) / self.resolution)
        py = int((y - self.origin_y) / self.resolution)
        return px, py
    
    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """像素坐标转世界坐标"""
        x = self.origin_x + px * self.resolution
        y = self.origin_y + py * self.resolution
        return x, y
    
    def update_with_slope_handling(self, 
                                     points_3d: np.ndarray,
                                     robot_pose: RobotPose) -> np.ndarray:
        """
        使用坡度处理更新地图
        
        核心思路：
        1. 检测前方区域的坡度
        2. 根据坡度动态调整高度阈值
        3. 将坡面上的点标记为地面而非障碍物
        4. 使用自适应阈值进行障碍物判断
        
        Returns:
            updated_grid_map: 更新后的占据栅格地图
        """
        if len(points_3d) == 0:
            return self.grid_map
        
        # 步骤1：检测机器人前方区域的坡度
        front_points = self._get_front_points(points_3d, robot_pose, angle_range=60)
        slope_angle, slope_dir, ground_z = self.slope_handler.estimate_slope(
            front_points, robot_pose.x, robot_pose.y, radius=3.0
        )
        
        # 步骤2：获取自适应高度阈值
        adaptive_threshold = self.slope_handler.get_adaptive_height_threshold(slope_angle)
        
        # 步骤3：根据坡度判断可通行性
        is_traversable = self.slope_handler.is_traversable(slope_angle)
        
        # 步骤4：处理点云，区分地面点、坡面点和障碍物点
        obstacle_points = []
        ground_points = []
        slope_points = []
        
        for point in points_3d:
            # 计算相对于地面的高度
            local_ground_z = self._get_local_ground_height(point[0], point[1], points_3d)
            rel_height = point[2] - local_ground_z
            
            # 判断点类型
            if rel_height < -adaptive_threshold:
                # 低于地面的点（可能是噪声或下坡）
                ground_points.append(point)
            elif rel_height < adaptive_threshold:
                # 地面或坡面上的点
                if is_traversable and slope_angle > 5:
                    # 有坡度且可通行，标记为坡面点（不是障碍物）
                    slope_points.append(point)
                else:
                    ground_points.append(point)
            else:
                # 高于阈值的点，可能是障碍物
                obstacle_points.append(point)
        
        # 步骤5：更新高度图
        self._update_height_map(ground_points + slope_points)
        
        # 步骤6：使用贝叶斯更新占据概率
        if len(obstacle_points) > 0:
            obstacles_xy = np.array([p[:2] for p in obstacle_points])
            self._update_with_raycasting(obstacles_xy, robot_pose)
        
        # 步骤7：更新栅格地图
        self._update_grid_from_log_odds()
        
        return self.grid_map
    
    def update_with_dynamic_filtering(self,
                                        points_3d: np.ndarray,
                                        robot_pose: RobotPose) -> np.ndarray:
        """
        使用动态物体滤除更新地图
        
        核心思路：
        1. 维护多帧历史观测
        2. 对每个点进行分类（静态/动态）
        3. 只使用静态点更新地图
        4. 动态点被滤除，不产生残影
        
        Returns:
            updated_grid_map: 更新后的占据栅格地图
        """
        if len(points_3d) == 0:
            return self.grid_map
        
        # 步骤1：投影到2D并转换到全局坐标系
        points_xy = points_3d[:, :2]
        
        # 步骤2：分类动态点和静态点
        static_points, dynamic_points = self.dynamic_filter.classify_dynamic_static(
            points_xy, robot_pose, self.resolution
        )
        
        # 步骤3：只用静态点更新地图
        if len(static_points) > 0:
            self._update_with_raycasting(static_points, robot_pose)
        
        # 步骤4：更新历史
        self.dynamic_filter.update_history(points_xy, robot_pose)
        
        # 步骤5：更新栅格地图
        self._update_grid_from_log_odds()
        
        return self.grid_map
    
    def update_combined(self,
                        points_3d: np.ndarray,
                        robot_pose: RobotPose) -> np.ndarray:
        """
        综合更新：同时处理坡度和动态物体
        """
        # 第一层：坡度处理，获取障碍物候选点
        front_points = self._get_front_points(points_3d, robot_pose, angle_range=60)
        slope_angle, _, _ = self.slope_handler.estimate_slope(
            front_points, robot_pose.x, robot_pose.y, radius=3.0
        )
        adaptive_threshold = self.slope_handler.get_adaptive_height_threshold(slope_angle)
        
        # 提取候选障碍物点
        candidate_obstacles = []
        for point in points_3d:
            local_ground = self._get_local_ground_height(point[0], point[1], points_3d)
            if point[2] - local_ground > adaptive_threshold:
                candidate_obstacles.append(point[:2])
        
        if len(candidate_obstacles) == 0:
            return self.grid_map
        
        candidate_xy = np.array(candidate_obstacles)
        
        # 第二层：动态物体滤除
        static_points, dynamic_points = self.dynamic_filter.classify_dynamic_static(
            candidate_xy, robot_pose, self.resolution
        )
        
        # 第三层：更新地图
        if len(static_points) > 0:
            self._update_with_raycasting(static_points, robot_pose)
        
        self.dynamic_filter.update_history(candidate_xy, robot_pose)
        self._update_grid_from_log_odds()
        
        return self.grid_map
    
    def _get_front_points(self, points_3d: np.ndarray, robot_pose: RobotPose,
                          angle_range: float = 60) -> np.ndarray:
        """获取机器人前方的点"""
        if len(points_3d) == 0:
            return points_3d
        
        angles = np.arctan2(points_3d[:, 1] - robot_pose.y,
                           points_3d[:, 0] - robot_pose.x)
        angle_diff = np.abs(angles - robot_pose.yaw)
        angle_diff = np.minimum(angle_diff, 2*np.pi - angle_diff)
        
        front_mask = angle_diff < np.radians(angle_range / 2)
        return points_3d[front_mask]
    
    def _get_local_ground_height(self, x: float, y: float, 
                                   points_3d: np.ndarray) -> float:
        """获取局部地面高度"""
        # 查找附近的点
        distances = np.sqrt((points_3d[:, 0] - x)**2 + (points_3d[:, 1] - y)**2)
        nearby_mask = distances < 0.5
        nearby_points = points_3d[nearby_mask]
        
        if len(nearby_points) < 5:
            return 0.0
        
        # 取最低点作为地面高度
        return np.min(nearby_points[:, 2])
    
    def _update_height_map(self, ground_points: List[np.ndarray]):
        """更新高度图"""
        for point in ground_points:
            px, py = self.world_to_pixel(point[0], point[1])
            if 0 <= px < self.width and 0 <= py < self.height:
                current = self.height_map[py, px]
                if current < 0 or point[2] < current:
                    self.height_map[py, px] = point[2]
    
    def _update_with_raycasting(self, obstacles_xy: np.ndarray, robot_pose: RobotPose):
        """使用射线投射更新对数几率地图"""
        robot_px, robot_py = self.world_to_pixel(robot_pose.x, robot_pose.y)
        
        if not (0 <= robot_px < self.width and 0 <= robot_py < self.height):
            return
        
        for obs_point in obstacles_xy:
            obs_px, obs_py = self.world_to_pixel(obs_point[0], obs_point[1])
            
            if not (0 <= obs_px < self.width and 0 <= obs_py < self.height):
                continue
            
            # 射线投射
            line_points = self._bresenham_line(robot_px, robot_py, obs_px, obs_py)
            
            # 标记空闲
            for i in range(len(line_points) - 1):
                x, y = line_points[i]
                if 0 <= x < self.width and 0 <= y < self.height:
                    self.log_odds[y, x] += self.l_free
            
            # 标记占据
            self.log_odds[obs_py, obs_px] += self.l_occ
    
    def _bresenham_line(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """Bresenham线段生成算法"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        
        return points
    
    def _update_grid_from_log_odds(self):
        """从对数几率更新占据栅格地图"""
        probabilities = 1 - 1 / (1 + np.exp(self.log_odds))
        probabilities = np.clip(probabilities, 0, 1)
        
        # 转换为0-100的整数
        self.grid_map = (probabilities * 100).astype(np.int8)
        
        # 标记未知区域（没有观测过的）
        unknown_mask = (self.log_odds == 0)
        self.grid_map[unknown_mask] = -1
    
    def apply_morphological_filter(self, kernel_size: int = 3):
        """应用形态学滤波"""
        # 二值化
        binary = (self.grid_map > 70).astype(np.uint8) * 255
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        
        # 开运算（去除孤立噪点）
        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        
        # 闭运算（填充小空洞）
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)
        
        # 更新地图
        self.grid_map[closed == 255] = 100
        self.grid_map[(self.grid_map == 100) & (closed == 0)] = 0
    
    def get_map(self) -> np.ndarray:
        """获取当前地图"""
        return self.grid_map.copy()
    
    def get_height_map(self) -> np.ndarray:
        """获取高度图"""
        return self.height_map.copy()
    
    def save_as_image(self, filename: str = "accumulated_map.png"):
        """保存为图像"""
        map_vis = np.clip(self.grid_map, 0, 100).astype(np.uint8)
        map_vis = (map_vis * 2.55).astype(np.uint8)
        
        # 使用OpenCV的COLORMAP_JET进行可视化
        colored = cv2.applyColorMap(255 - map_vis, cv2.COLORMAP_JET)
        
        cv2.imwrite(filename, colored)
        print(f"累计地图已保存到 {filename}")
    
    def save_height_map_image(self, filename: str = "height_map.png"):
        """保存高度图为图像"""
        # 归一化高度图
        height_vis = self.height_map.copy()
        valid = height_vis > -90
        if np.any(valid):
            min_h = np.min(height_vis[valid])
            max_h = np.max(height_vis[valid])
            if max_h > min_h:
                height_vis = (height_vis - min_h) / (max_h - min_h) * 255
        height_vis[~valid] = 0
        height_vis = height_vis.astype(np.uint8)
        
        colored = cv2.applyColorMap(height_vis, cv2.COLORMAP_JET)
        cv2.imwrite(filename, colored)
        print(f"高度图已保存到 {filename}")


# ============================================================
# 第四部分：测试和演示
# ============================================================

def generate_test_pointcloud(num_points: int = 5000) -> np.ndarray:
    """生成测试点云"""
    # 生成随机点（模拟环境）
    points = np.random.randn(num_points, 3)
    
    # 添加一些结构（模拟墙）
    for i in range(int(num_points * 0.3)):
        points[i, 0] = np.random.uniform(-10, 10)
        points[i, 1] = np.random.uniform(8, 10)  # 墙在y=8-10
        points[i, 2] = np.random.uniform(0, 2)
    
    # 添加一些地面点
    for i in range(int(num_points * 0.5)):
        points[i, 2] = np.random.uniform(-0.1, 0.1)
    
    return points


def test_single_frame_mapper():
    """测试单帧地图构建器"""
    print("=" * 60)
    print("测试单帧地图构建器")
    print("=" * 60)
    
    config = MapConfig(size_m=30, resolution=0.05, origin_x=-15, origin_y=-15)
    mapper = SingleFrameMapper(config)
    
    # 生成测试点云
    points_3d = generate_test_pointcloud(3000)
    robot_pose = RobotPose(x=0, y=0, z=0.3, yaw=0)
    
    # 处理单帧
    grid_map = mapper.process_frame(points_3d, robot_pose)
    
    # 保存结果
    mapper.save_as_image(grid_map, "test_single_frame.png")
    
    print(f"单帧地图尺寸: {grid_map.shape}")
    print(f"占据像素数: {np.sum(grid_map == 100)}")
    print(f"空闲像素数: {np.sum(grid_map == 0)}")
    print("✓ 单帧地图测试完成\n")


def test_accumulated_mapper():
    """测试累计地图构建器（含坡度和动态物体处理）"""
    print("=" * 60)
    print("测试累计地图构建器")
    print("=" * 60)
    
    config = MapConfig(size_m=30, resolution=0.05, origin_x=-15, origin_y=-15)
    mapper = AccumulatedMapper(config)
    
    # 模拟多帧数据
    for frame in range(20):
        # 生成测试点云
        points_3d = generate_test_pointcloud(2000)
        
        # 模拟机器人移动（圆周运动）
        angle = frame * 0.1
        robot_pose = RobotPose(
            x=5 * np.cos(angle),
            y=5 * np.sin(angle),
            z=0.3,
            yaw=angle + np.pi/2
        )
        
        # 更新地图（综合模式）
        grid_map = mapper.update_combined(points_3d, robot_pose)
        
        # 每5帧应用一次形态学滤波
        if frame % 5 == 0 and frame > 0:
            mapper.apply_morphological_filter(kernel_size=3)
    
    # 保存结果
    mapper.save_as_image("test_accumulated_map.png")
    mapper.save_height_map_image("test_height_map.png")
    
    grid_map = mapper.get_map()
    height_map = mapper.get_height_map()
    
    print(f"累计地图尺寸: {grid_map.shape}")
    print(f"占据像素数: {np.sum(grid_map == 100)}")
    print(f"空闲像素数: {np.sum(grid_map == 0)}")
    print(f"未知