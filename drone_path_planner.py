#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机路径规划算法
================

本模块提供了无人机巡航路径规划的核心功能，使用A*寻路算法和螺旋遍历优化，
实现在禁飞区约束下的完整覆盖巡航路径规划。

主要功能：
- A*寻路算法进行路径规划
- 螺旋遍历优化提高巡航效率
- 多区域处理能力
- 路径去重功能
- 输入参数验证和错误处理

作者: 
版本: 1.0
创建日期: 2024
"""

import logging
from typing import List, Tuple, Set, Dict, Optional
import heapq
from collections import deque

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 全局配置常量
GRID_COLS = 9  # 网格列数
GRID_ROWS = 7  # 网格行数
END_POINT_CANDIDATES = ["A9B3", "A7B1"]  # 终点候选列表


def plan_drone_patrol_path(forbidden_zones: List[Tuple[int, int]], 
                          start_point: Tuple[int, int] = (9, 1)) -> Tuple[List[Tuple[int, int]], int]:
    """
    无人机巡航路径规划主函数
    
    使用A*寻路算法和螺旋遍历优化，在给定的禁飞区约束下规划完整的覆盖巡航路径。
    
    Args:
        forbidden_zones: 禁飞区坐标列表，格式为元组列表 [(列, 行), ...]
        start_point: 起飞点坐标，默认为 (9, 1)，格式为元组 (列, 行)
        
    Returns:
        Tuple[List[Tuple[int, int]], int]: 返回元组 (航点坐标列表, 航点总数)
        - 航点坐标列表格式：[(9,1), (8,1), (7,1), ...]
        - 航点总数：整数值
        
    Raises:
        ValueError: 当输入参数不合法时抛出
        
    Examples:
        >>> # 定义禁飞区
        >>> forbidden_zones = [(8, 4), (8, 5), (8, 3)]
        >>> # 使用默认起点
        >>> waypoints, total_count = plan_drone_patrol_path(forbidden_zones)
        >>> print(f"生成路径包含 {total_count} 个航点")
        >>> print(f"前5个航点: {waypoints[:5]}")
        
        >>> # 使用自定义起点  
        >>> waypoints, total_count = plan_drone_patrol_path(forbidden_zones, start_point=(1, 1))
    """
    
    # 输入参数验证
    try:
        _validate_input_parameters(forbidden_zones, start_point)
    except ValueError as e:
        logger.error(f"输入参数验证失败: {e}")
        return [], 0
    
    logger.info(f"开始规划无人机巡航路径，起点: {start_point}, 禁飞区数量: {len(forbidden_zones)}")
    
    try:
        # 执行路径规划
        waypoints = _plan_coverage_tour_internal(forbidden_zones, start_point)
        
        # 去重处理
        unique_waypoints = _remove_duplicates(waypoints)
        
        total_count = len(unique_waypoints)
        logger.info(f"路径规划完成，生成 {total_count} 个航点")
        
        return unique_waypoints, total_count
        
    except Exception as e:
        logger.error(f"路径规划过程中发生错误: {e}")
        return [], 0


def _validate_input_parameters(forbidden_zones: List[Tuple[int, int]], 
                              start_point: Tuple[int, int]) -> None:
    """验证输入参数的合法性"""
    
    # 验证起点坐标
    if not isinstance(start_point, tuple) or len(start_point) != 2:
        raise ValueError("起点坐标必须是包含两个元素的元组")
    
    col, row = start_point
    if not (1 <= col <= GRID_COLS and 1 <= row <= GRID_ROWS):
        raise ValueError(f"起点坐标超出范围，有效范围：列[1-{GRID_COLS}]，行[1-{GRID_ROWS}]")
    
    # 验证禁飞区坐标
    if not isinstance(forbidden_zones, list):
        raise ValueError("禁飞区必须是列表类型")
    
    for zone in forbidden_zones:
        if not isinstance(zone, tuple) or len(zone) != 2:
            raise ValueError("禁飞区坐标必须是包含两个元素的元组")
        
        col, row = zone
        if not (1 <= col <= GRID_COLS and 1 <= row <= GRID_ROWS):
            raise ValueError(f"禁飞区坐标超出范围，有效范围：列[1-{GRID_COLS}]，行[1-{GRID_ROWS}]")
    
    # 验证起点不在禁飞区内
    if start_point in forbidden_zones:
        raise ValueError("起点不能位于禁飞区内")


# 坐标转换函数
def _tuple_to_idx(pos: Tuple[int, int]) -> int:
    """将坐标元组转换为网格索引"""
    col, row = pos
    return (row - 1) * GRID_COLS + (col - 1)


def _idx_to_tuple(idx: int) -> Tuple[int, int]:
    """将网格索引转换为坐标元组"""
    row = idx // GRID_COLS + 1
    col = idx % GRID_COLS + 1
    return (col, row)


def _code_to_tuple(code: str) -> Tuple[int, int]:
    """将编码字符串转换为坐标元组，例如 'A9B3' -> (9, 3)"""
    try:
        # 解析格式：A{col}B{row}
        if not code.startswith('A') or 'B' not in code:
            raise ValueError(f"无效的编码格式: {code}")
        
        parts = code[1:].split('B')
        if len(parts) != 2:
            raise ValueError(f"无效的编码格式: {code}")
        
        col = int(parts[0])
        row = int(parts[1])
        return (col, row)
    except (ValueError, IndexError) as e:
        raise ValueError(f"编码解析失败: {code}, 错误: {e}")


def _tuple_to_code(pos: Tuple[int, int]) -> str:
    """将坐标元组转换为编码字符串"""
    col, row = pos
    return f"A{col}B{row}"


def _get_neighbors(pos: Tuple[int, int]) -> List[Tuple[int, int]]:
    """获取指定位置的相邻位置列表（四方向）"""
    col, row = pos
    neighbors = []
    
    # 四个方向：上、下、左、右
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]
    
    for dc, dr in directions:
        new_col, new_row = col + dc, row + dr
        # 检查边界
        if 1 <= new_col <= GRID_COLS and 1 <= new_row <= GRID_ROWS:
            neighbors.append((new_col, new_row))
    
    return neighbors


def _heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> int:
    """计算两点间的曼哈顿距离作为启发函数"""
    return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])


def _a_star_search(start: Tuple[int, int], 
                  goal: Tuple[int, int], 
                  forbidden_zones: Set[Tuple[int, int]]) -> Optional[List[Tuple[int, int]]]:
    """
    A*寻路算法实现
    
    Args:
        start: 起始点坐标
        goal: 目标点坐标
        forbidden_zones: 禁飞区坐标集合
        
    Returns:
        路径列表，如果无法找到路径则返回None
    """
    
    if start == goal:
        return [start]
    
    # 检查起点和终点是否在禁飞区
    if start in forbidden_zones or goal in forbidden_zones:
        return None
    
    # A*算法数据结构
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: _heuristic(start, goal)}
    
    closed_set = set()
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            # 重构路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # 反转路径
        
        closed_set.add(current)
        
        for neighbor in _get_neighbors(current):
            if neighbor in closed_set or neighbor in forbidden_zones:
                continue
            
            tentative_g_score = g_score[current] + 1
            
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + _heuristic(neighbor, goal)
                
                if neighbor not in [item[1] for item in open_set]:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return None  # 未找到路径


def _spiral_traverse_region(region_points: Set[Tuple[int, int]], 
                           start: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    螺旋遍历区域内的所有点
    
    Args:
        region_points: 区域内的点集合
        start: 起始点
        
    Returns:
        螺旋遍历的点序列
    """
    
    if not region_points or start not in region_points:
        return []
    
    visited = set()
    path = [start]
    visited.add(start)
    current = start
    
    # 螺旋方向：右、下、左、上
    directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
    direction_idx = 0
    
    while len(visited) < len(region_points):
        found_next = False
        
        # 尝试当前方向
        for _ in range(4):  # 最多尝试4个方向
            dc, dr = directions[direction_idx]
            next_pos = (current[0] + dc, current[1] + dr)
            
            if (next_pos in region_points and 
                next_pos not in visited):
                current = next_pos
                path.append(current)
                visited.add(current)
                found_next = True
                break
            else:
                # 转向下一个方向
                direction_idx = (direction_idx + 1) % 4
        
        if not found_next:
            # 如果螺旋无法继续，寻找最近的未访问点
            remaining = region_points - visited
            if remaining:
                min_dist = float('inf')
                next_point = None
                for point in remaining:
                    dist = _heuristic(current, point)
                    if dist < min_dist:
                        min_dist = dist
                        next_point = point
                
                if next_point:
                    current = next_point
                    path.append(current)
                    visited.add(current)
                else:
                    break
    
    return path


def _identify_connected_regions(all_points: Set[Tuple[int, int]], 
                               forbidden_zones: Set[Tuple[int, int]]) -> List[Set[Tuple[int, int]]]:
    """识别连通区域"""
    
    available_points = all_points - forbidden_zones
    regions = []
    visited = set()
    
    for point in available_points:
        if point not in visited:
            # BFS寻找连通区域
            region = set()
            queue = deque([point])
            
            while queue:
                current = queue.popleft()
                if current in visited:
                    continue
                
                visited.add(current)
                region.add(current)
                
                for neighbor in _get_neighbors(current):
                    if (neighbor in available_points and 
                        neighbor not in visited):
                        queue.append(neighbor)
            
            if region:
                regions.append(region)
    
    return regions


def _plan_coverage_tour_internal(forbidden_zones: List[Tuple[int, int]], 
                                start_point: Tuple[int, int]) -> List[Tuple[int, int]]:
    """内部路径规划核心逻辑"""
    
    forbidden_set = set(forbidden_zones)
    
    # 生成所有网格点
    all_points = set()
    for col in range(1, GRID_COLS + 1):
        for row in range(1, GRID_ROWS + 1):
            all_points.add((col, row))
    
    # 识别连通区域
    regions = _identify_connected_regions(all_points, forbidden_set)
    logger.info(f"识别到 {len(regions)} 个连通区域")
    
    if not regions:
        logger.warning("未找到可用的连通区域")
        return []
    
    # 找到包含起点的区域
    start_region = None
    for region in regions:
        if start_point in region:
            start_region = region
            break
    
    if start_region is None:
        logger.error("起点不在任何可用区域内")
        return []
    
    total_path = []
    current_pos = start_point
    
    # 首先遍历起点所在区域
    region_path = _spiral_traverse_region(start_region, current_pos)
    total_path.extend(region_path)
    current_pos = region_path[-1] if region_path else current_pos
    
    # 遍历其他区域
    remaining_regions = [r for r in regions if r != start_region]
    
    for region in remaining_regions:
        # 找到区域中距离当前位置最近的点作为入口
        min_dist = float('inf')
        entry_point = None
        
        for point in region:
            dist = _heuristic(current_pos, point)
            if dist < min_dist:
                min_dist = dist
                entry_point = point
        
        if entry_point:
            # 规划到达该区域的路径
            path_to_region = _a_star_search(current_pos, entry_point, forbidden_set)
            if path_to_region:
                # 避免重复添加起点
                if len(total_path) > 0 and len(path_to_region) > 0:
                    total_path.extend(path_to_region[1:])
                else:
                    total_path.extend(path_to_region)
                
                current_pos = entry_point
                
                # 螺旋遍历该区域
                region_path = _spiral_traverse_region(region, current_pos)
                if len(region_path) > 1:  # 避免重复添加起点
                    total_path.extend(region_path[1:])
                    current_pos = region_path[-1]
    
    # 尝试返回到终点候选位置之一
    for end_code in END_POINT_CANDIDATES:
        try:
            end_point = _code_to_tuple(end_code)
            if end_point not in forbidden_set:
                path_to_end = _a_star_search(current_pos, end_point, forbidden_set)
                if path_to_end and len(path_to_end) > 1:
                    total_path.extend(path_to_end[1:])
                    break
        except ValueError:
            continue
    
    return total_path


def _remove_duplicates(waypoints: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    """移除路径中的重复点，保持顺序"""
    seen = set()
    unique_waypoints = []
    
    for waypoint in waypoints:
        if waypoint not in seen:
            seen.add(waypoint)
            unique_waypoints.append(waypoint)
    
    return unique_waypoints


# 测试和示例代码
def _run_examples():
    """运行示例代码"""
    
    print("=== 无人机路径规划算法示例 ===\n")
    
    # 示例1：使用默认起点
    print("示例1：使用默认起点")
    forbidden_zones = [(8, 4), (8, 5), (8, 3)]
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones)
    
    print(f"禁飞区: {forbidden_zones}")
    print(f"生成路径包含 {total_count} 个航点")
    if waypoints:
        print(f"前5个航点: {waypoints[:5]}")
        print(f"后5个航点: {waypoints[-5:]}")
    print()
    
    # 示例2：使用自定义起点
    print("示例2：使用自定义起点")
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones, start_point=(1, 1))
    
    print(f"起点: (1, 1)")
    print(f"禁飞区: {forbidden_zones}")
    print(f"生成路径包含 {total_count} 个航点")
    if waypoints:
        print(f"前5个航点: {waypoints[:5]}")
        print(f"后5个航点: {waypoints[-5:]}")
    print()
    
    # 示例3：空禁飞区
    print("示例3：空禁飞区")
    waypoints, total_count = plan_drone_patrol_path([])
    
    print(f"禁飞区: []")
    print(f"生成路径包含 {total_count} 个航点")
    if waypoints:
        print(f"前5个航点: {waypoints[:5]}")
        print(f"后5个航点: {waypoints[-5:]}")
    print()


if __name__ == "__main__":
    _run_examples()