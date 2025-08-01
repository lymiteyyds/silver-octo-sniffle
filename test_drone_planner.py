#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机路径规划算法测试
==================

测试 drone_path_planner.py 中的主要功能
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from drone_path_planner import plan_drone_patrol_path


def test_basic_functionality():
    """测试基本功能"""
    print("=== 测试基本功能 ===")
    
    # 测试1：基本路径规划
    forbidden_zones = [(8, 4), (8, 5), (8, 3)]
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones)
    
    assert isinstance(waypoints, list), "返回的航点应该是列表"
    assert isinstance(total_count, int), "返回的总数应该是整数"
    assert total_count > 0, "应该生成至少一个航点"
    assert len(waypoints) == total_count, "航点列表长度应该等于总数"
    
    # 验证所有航点都是有效坐标
    for waypoint in waypoints:
        assert isinstance(waypoint, tuple), "每个航点应该是元组"
        assert len(waypoint) == 2, "每个航点应该包含两个坐标"
        col, row = waypoint
        assert 1 <= col <= 9, f"列坐标应该在1-9范围内，实际: {col}"
        assert 1 <= row <= 7, f"行坐标应该在1-7范围内，实际: {row}"
    
    # 验证起点
    assert waypoints[0] == (9, 1), f"起点应该是(9, 1)，实际: {waypoints[0]}"
    
    # 验证没有禁飞区坐标
    for waypoint in waypoints:
        assert waypoint not in forbidden_zones, f"航点 {waypoint} 不应该在禁飞区内"
    
    print("✓ 基本功能测试通过")


def test_custom_start_point():
    """测试自定义起点"""
    print("=== 测试自定义起点 ===")
    
    forbidden_zones = [(5, 3), (5, 4)]
    start_point = (1, 1)
    waypoints, total_count = plan_drone_patrol_path(forbidden_zones, start_point)
    
    assert waypoints[0] == start_point, f"起点应该是{start_point}，实际: {waypoints[0]}"
    assert total_count > 0, "应该生成至少一个航点"
    
    print("✓ 自定义起点测试通过")


def test_empty_forbidden_zones():
    """测试空禁飞区"""
    print("=== 测试空禁飞区 ===")
    
    waypoints, total_count = plan_drone_patrol_path([])
    
    assert total_count > 50, "没有禁飞区时应该覆盖大部分网格点"
    assert waypoints[0] == (9, 1), "默认起点应该是(9, 1)"
    
    print("✓ 空禁飞区测试通过")


def test_input_validation():
    """测试输入验证"""
    print("=== 测试输入验证 ===")
    
    # 测试无效起点
    waypoints, total_count = plan_drone_patrol_path([], start_point=(0, 0))
    assert total_count == 0, "无效起点应该返回空路径"
    
    waypoints, total_count = plan_drone_patrol_path([], start_point=(10, 8))
    assert total_count == 0, "超出范围的起点应该返回空路径"
    
    # 测试起点在禁飞区
    waypoints, total_count = plan_drone_patrol_path([(9, 1)], start_point=(9, 1))
    assert total_count == 0, "起点在禁飞区应该返回空路径"
    
    print("✓ 输入验证测试通过")


def test_path_uniqueness():
    """测试路径去重"""
    print("=== 测试路径去重 ===")
    
    waypoints, total_count = plan_drone_patrol_path([(5, 3)])
    
    # 检查没有重复点
    unique_points = set(waypoints)
    assert len(unique_points) == len(waypoints), "路径中不应该有重复点"
    
    print("✓ 路径去重测试通过")


def run_all_tests():
    """运行所有测试"""
    print("开始运行无人机路径规划算法测试...\n")
    
    try:
        test_basic_functionality()
        test_custom_start_point()
        test_empty_forbidden_zones()
        test_input_validation()
        test_path_uniqueness()
        
        print("\n🎉 所有测试通过！")
        return True
        
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        return False
    except Exception as e:
        print(f"\n💥 测试过程中发生错误: {e}")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)