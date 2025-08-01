#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机路径规划算法演示程序
========================

这个程序演示了无人机路径规划算法的各种使用场景和功能特性。

使用方法:
    python demo_drone_planner.py

功能特性:
- A*寻路算法进行路径规划
- 螺旋遍历优化提高巡航效率  
- 多区域处理能力
- 路径去重功能
- 完整的输入验证和错误处理
"""

import time
from typing import List, Tuple
from drone_path_planner import plan_drone_patrol_path


def visualize_grid(waypoints: List[Tuple[int, int]], 
                  forbidden_zones: List[Tuple[int, int]], 
                  start_point: Tuple[int, int]) -> None:
    """
    在控制台可视化显示网格和路径
    
    Args:
        waypoints: 航点列表
        forbidden_zones: 禁飞区列表
        start_point: 起始点
    """
    
    # 创建9x7网格
    grid = [['.' for _ in range(9)] for _ in range(7)]
    
    # 标记禁飞区
    for col, row in forbidden_zones:
        if 1 <= col <= 9 and 1 <= row <= 7:
            grid[row-1][col-1] = 'X'
    
    # 标记起始点
    if 1 <= start_point[0] <= 9 and 1 <= start_point[1] <= 7:
        grid[start_point[1]-1][start_point[0]-1] = 'S'
    
    # 标记路径点（用数字表示访问顺序）
    for i, (col, row) in enumerate(waypoints):
        if 1 <= col <= 9 and 1 <= row <= 7:
            if grid[row-1][col-1] == '.':
                # 显示前10个点的顺序，其他用*表示
                if i < 10:
                    grid[row-1][col-1] = str(i)
                else:
                    grid[row-1][col-1] = '*'
    
    # 打印网格
    print("   1 2 3 4 5 6 7 8 9")
    print("  ┌─────────────────┐")
    for row_idx, row in enumerate(grid):
        print(f"{row_idx+1} │ {' '.join(row)} │")
    print("  └─────────────────┘")
    print()
    print("图例:")
    print("  S = 起始点")
    print("  X = 禁飞区")
    print("  0-9 = 路径前10个点的访问顺序")
    print("  * = 其他路径点")
    print("  . = 未访问点")


def demo_basic_usage():
    """演示基本使用方法"""
    
    print("=" * 60)
    print("演示1：基本使用方法")
    print("=" * 60)
    
    # 定义禁飞区
    forbidden_zones = [(8, 4), (8, 5), (8, 3)]
    
    print("禁飞区坐标:", forbidden_zones)
    print("起始点: 默认 (9, 1)")
    print()
    
    start_time = time.time()
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones)
    end_time = time.time()
    
    print(f"规划完成！耗时: {end_time - start_time:.3f} 秒")
    print(f"生成路径包含 {total_count} 个航点")
    print()
    
    if waypoints:
        print("路径可视化:")
        visualize_grid(waypoints, forbidden_zones, (9, 1))
        
        print("前10个航点:")
        for i, waypoint in enumerate(waypoints[:10]):
            print(f"  {i+1:2d}. {waypoint}")
        
        if len(waypoints) > 10:
            print(f"  ... (还有 {len(waypoints)-10} 个航点)")
        
        print(f"最后一个航点: {waypoints[-1]}")
    
    print()


def demo_custom_start():
    """演示自定义起点"""
    
    print("=" * 60)
    print("演示2：自定义起点")
    print("=" * 60)
    
    forbidden_zones = [(5, 3), (5, 4), (6, 3)]
    start_point = (1, 1)
    
    print("禁飞区坐标:", forbidden_zones)
    print("起始点:", start_point)
    print()
    
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones, start_point)
    
    print(f"生成路径包含 {total_count} 个航点")
    
    if waypoints:
        print("路径可视化:")
        visualize_grid(waypoints, forbidden_zones, start_point)
    
    print()


def demo_complex_forbidden_zones():
    """演示复杂禁飞区场景"""
    
    print("=" * 60)
    print("演示3：复杂禁飞区场景")
    print("=" * 60)
    
    # 创建一个更复杂的禁飞区布局
    forbidden_zones = [
        (4, 3), (4, 4), (4, 5),  # 垂直障碍
        (6, 2), (7, 2), (8, 2),  # 水平障碍
        (2, 6), (3, 6)           # 角落障碍
    ]
    
    print("禁飞区坐标:", forbidden_zones)
    print("起始点: 默认 (9, 1)")
    print()
    
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones)
    
    print(f"生成路径包含 {total_count} 个航点")
    
    if waypoints:
        print("路径可视化:")
        visualize_grid(waypoints, forbidden_zones, (9, 1))
    
    print()


def demo_empty_forbidden_zones():
    """演示空禁飞区场景"""
    
    print("=" * 60)
    print("演示4：空禁飞区场景（完整覆盖）")
    print("=" * 60)
    
    forbidden_zones = []
    
    print("禁飞区坐标: []（无禁飞区）")
    print("起始点: 默认 (9, 1)")
    print()
    
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones)
    
    print(f"生成路径包含 {total_count} 个航点")
    print(f"网格总点数: {9 * 7} 个")
    print(f"覆盖率: {total_count / (9 * 7) * 100:.1f}%")
    
    if waypoints:
        print("路径可视化:")
        visualize_grid(waypoints, forbidden_zones, (9, 1))
    
    print()


def demo_error_handling():
    """演示错误处理"""
    
    print("=" * 60)
    print("演示5：错误处理")
    print("=" * 60)
    
    test_cases = [
        {
            "name": "起点超出范围",
            "forbidden_zones": [],
            "start_point": (10, 8),
            "expected": "应该返回空路径"
        },
        {
            "name": "起点在禁飞区内",
            "forbidden_zones": [(9, 1)],
            "start_point": (9, 1),
            "expected": "应该返回空路径"
        },
        {
            "name": "禁飞区坐标超出范围",
            "forbidden_zones": [(10, 8)],
            "start_point": (9, 1),
            "expected": "应该返回空路径"
        }
    ]
    
    for test_case in test_cases:
        print(f"测试: {test_case['name']}")
        print(f"禁飞区: {test_case['forbidden_zones']}")
        print(f"起点: {test_case['start_point']}")
        
        waypoints, total_count = plan_drone_patrol_path(
            test_case['forbidden_zones'], 
            test_case['start_point']
        )
        
        print(f"结果: 生成 {total_count} 个航点")
        print(f"预期: {test_case['expected']}")
        print("✓ " if total_count == 0 else "✗", "符合预期" if total_count == 0 else "不符合预期")
        print()


def demo_performance_test():
    """性能测试"""
    
    print("=" * 60)
    print("演示6：性能测试")
    print("=" * 60)
    
    test_cases = [
        {"name": "小规模禁飞区", "zones": [(5, 3)]},
        {"name": "中等规模禁飞区", "zones": [(3, 2), (3, 3), (3, 4), (5, 5), (6, 5)]},
        {"name": "大规模禁飞区", "zones": [(i, j) for i in range(3, 7) for j in range(2, 5) if (i + j) % 2 == 0]},
    ]
    
    for test_case in test_cases:
        print(f"测试: {test_case['name']}")
        print(f"禁飞区数量: {len(test_case['zones'])}")
        
        start_time = time.time()
        waypoints, total_count = plan_drone_patrol_path(test_case['zones'])
        end_time = time.time()
        
        print(f"耗时: {end_time - start_time:.3f} 秒")
        print(f"生成航点: {total_count} 个")
        print(f"平均每秒生成: {total_count / (end_time - start_time):.1f} 个航点")
        print()


def main():
    """主演示程序"""
    
    print("🚁 无人机路径规划算法演示程序")
    print("=================================\n")
    
    print("本程序将演示无人机路径规划算法的各种功能特性：")
    print("- A*寻路算法")
    print("- 螺旋遍历优化")
    print("- 多区域处理")
    print("- 路径去重")
    print("- 输入验证和错误处理")
    print()
    
    input("按 Enter 键开始演示...")
    print()
    
    # 运行各个演示
    demo_basic_usage()
    input("按 Enter 键继续下一个演示...")
    
    demo_custom_start()
    input("按 Enter 键继续下一个演示...")
    
    demo_complex_forbidden_zones()
    input("按 Enter 键继续下一个演示...")
    
    demo_empty_forbidden_zones()
    input("按 Enter 键继续下一个演示...")
    
    demo_error_handling()
    input("按 Enter 键继续下一个演示...")
    
    demo_performance_test()
    
    print("=" * 60)
    print("演示完成！")
    print("=" * 60)
    print()
    
    print("🎯 主要功能总结:")
    print("✅ 完整的网格覆盖路径规划")
    print("✅ 智能避障（A*寻路算法）")
    print("✅ 螺旋遍历优化")
    print("✅ 多区域连通性处理")
    print("✅ 路径去重和优化")
    print("✅ 完善的输入验证")
    print("✅ 详细的日志记录")
    print()
    
    print("📚 使用方法:")
    print("from drone_path_planner import plan_drone_patrol_path")
    print("waypoints, count = plan_drone_patrol_path(forbidden_zones, start_point)")
    print()
    
    print("感谢使用无人机路径规划算法！🚁")


if __name__ == "__main__":
    main()