# 无人机路径规划算法 (Drone Path Planning Algorithm)

这是一个功能完整的无人机巡航路径规划算法实现，使用A*寻路算法和螺旋遍历优化，能够在禁飞区约束下规划完整的覆盖巡航路径。

## 🚁 主要功能

- **A*寻路算法**: 智能路径规划和避障
- **螺旋遍历优化**: 提高巡航效率，减少重复路径
- **多区域处理**: 自动识别和处理连通区域
- **路径去重**: 智能去除重复航点
- **完整验证**: 输入参数验证和错误处理
- **详细日志**: 完整的执行过程记录

## 📦 文件结构

```
├── drone_path_planner.py      # 主要算法实现
├── demo_drone_planner.py      # 演示程序
├── test_drone_planner.py      # 测试程序
└── README.md                  # 项目文档
```

## 🚀 快速开始

### 基本使用

```python
from drone_path_planner import plan_drone_patrol_path

# 定义禁飞区
forbidden_zones = [(8, 4), (8, 5), (8, 3)]

# 使用默认起点 (9, 1)
waypoints, total_count = plan_drone_patrol_path(forbidden_zones)

print(f"生成路径包含 {total_count} 个航点")
print(f"前5个航点: {waypoints[:5]}")
```

### 自定义起点

```python
# 使用自定义起点
waypoints, total_count = plan_drone_patrol_path(
    forbidden_zones, 
    start_point=(1, 1)
)
```

## 📖 API 文档

### 主函数

```python
plan_drone_patrol_path(
    forbidden_zones: List[Tuple[int, int]], 
    start_point: Tuple[int, int] = (9, 1)
) -> Tuple[List[Tuple[int, int]], int]
```

**参数:**
- `forbidden_zones`: 禁飞区坐标列表，格式为 `[(列, 行), ...]`
- `start_point`: 起飞点坐标，默认为 `(9, 1)`，格式为 `(列, 行)`

**返回值:**
- 元组: `(航点坐标列表, 航点总数)`
- 航点坐标列表格式: `[(9,1), (8,1), (7,1), ...]`
- 航点总数: 整数值

**坐标系统:**
- 网格尺寸: 9列 × 7行
- 坐标范围: 列[1-9]，行[1-7]
- 坐标格式: (列, 行)

## 🧪 运行测试

```bash
# 运行基本测试
python test_drone_planner.py

# 运行演示程序
python demo_drone_planner.py

# 运行示例
python drone_path_planner.py
```

## 🎯 算法特性

### 1. A*寻路算法
- 使用曼哈顿距离作为启发函数
- 智能避开禁飞区
- 保证找到最优或近似最优路径

### 2. 螺旋遍历优化
- 从起点开始螺旋式遍历
- 减少往返和重复路径
- 提高巡航效率

### 3. 多区域处理
- 自动识别连通区域
- 智能规划跨区域路径
- 确保完整覆盖所有可达区域

### 4. 错误处理
- 输入参数验证
- 坐标范围检查
- 异常情况处理
- 详细错误信息

## 📊 性能表现

- **执行速度**: 毫秒级路径规划
- **内存使用**: 低内存占用
- **覆盖率**: 接近100%的网格覆盖
- **路径质量**: 优化的航点序列

## 🛠️ 技术实现

### 核心算法
- **A*搜索**: 最优路径查找
- **BFS遍历**: 连通区域识别
- **螺旋算法**: 区域内优化遍历
- **贪心策略**: 跨区域路径选择

### 数据结构
- **优先队列**: A*算法的开放集
- **集合**: 快速查找和去重
- **列表**: 路径存储和返回

## 🔧 配置参数

所有配置参数都在算法内部固定：

```python
GRID_COLS = 9                    # 网格列数
GRID_ROWS = 7                    # 网格行数  
END_POINT_CANDIDATES = ["A9B3", "A7B1"]  # 终点候选
```

## 📈 使用示例

查看 `demo_drone_planner.py` 获取完整的使用示例，包括：

1. **基本使用方法**
2. **自定义起点**
3. **复杂禁飞区场景**
4. **完整覆盖场景**
5. **错误处理演示**
6. **性能测试**

## 🤝 贡献指南

欢迎贡献代码！请确保：

1. 代码符合现有的风格规范
2. 添加必要的测试用例
3. 更新相关文档
4. 通过所有测试

## 📄 许可证

本项目采用 MIT 许可证。

## 🐛 问题反馈

如果您发现任何问题或有改进建议，请创建 Issue 或提交 Pull Request。

---

**作者**: lymiteyyds  
**版本**: 1.0  
**创建日期**: 2024
