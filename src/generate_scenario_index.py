#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
场景库索引生成器
功能：为 12 种基础场景生成索引文件，方便快速查询和统计
"""

import json
import os
from datetime import datetime


def generate_scenario_index():
    """生成场景库索引"""
    
    # 读取配置文件
    config_file = "<PATH>"
    
    with open(config_file, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    scenarios = config['scenarios']
    global_config = config['global_config']
    
    # 生成索引数据
    index_data = {
        "library_name": "12 种基础仿真场景库",
        "version": "v1.0",
        "created_date": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "total_scenarios": len(scenarios),
        "categories": {
            "single_ship": [s for s in scenarios if s['ship_num'] == 1],
            "two_ships": [s for s in scenarios if s['ship_num'] == 2],
            "three_ships": [s for s in scenarios if s['ship_num'] == 3]
        },
        "statistics": {
            "single_ship_count": len([s for s in scenarios if s['ship_num'] == 1]),
            "two_ships_count": len([s for s in scenarios if s['ship_num'] == 2]),
            "three_ships_count": len([s for s in scenarios if s['ship_num'] == 3]),
            "behavior_modes": {
                "standon": len([s for s in scenarios if s['behavior_mode'] == 'standon']),
                "avoid": len([s for s in scenarios if s['behavior_mode'] == 'avoid']),
                "violate": len([s for s in scenarios if s['behavior_mode'] == 'violate']),
                "mixed": len([s for s in scenarios if s['behavior_mode'] == 'mixed'])
            }
        },
        "scenarios": scenarios,
        "encounter_types_summary": {
            "crossing-giveway": "交叉相遇 - 本船为让路船 (COLREGs 第 15 条)",
            "crossing-standon": "交叉相遇 - 本船为直航船 (COLREGs 第 15 条)",
            "overtaking": "追越 - 本船为让路船 (COLREGs 第 13 条)",
            "head-on": "对遇 - 双方均为让路船 (COLREGs 第 14 条)",
            "special-vessel": "特殊船舶遭遇 (COLREGs 第 18 条)"
        }
    }
    
    # 保存索引文件
    index_file = "<PATH>"
    with open(index_file, 'w', encoding='utf-8') as f:
        json.dump(index_data, f, indent=2, ensure_ascii=False)
    
    print("=" * 80)
    print("场景库索引已生成")
    print("=" * 80)
    print(f"索引文件：{index_file}")
    print(f"场景总数：{len(scenarios)}")
    print(f"单船场景：{index_data['statistics']['single_ship_count']}")
    print(f"双船场景：{index_data['statistics']['two_ships_count']}")
    print(f"三船场景：{index_data['statistics']['three_ships_count']}")
    print("")
    print("行为模式分布:")
    print(f"  保向保速 (standon): {index_data['statistics']['behavior_modes']['standon']}")
    print(f"  主动避碰 (avoid):   {index_data['statistics']['behavior_modes']['avoid']}")
    print(f"  违规行为 (violate): {index_data['statistics']['behavior_modes']['violate']}")
    print(f"  混合模式 (mixed):   {index_data['statistics']['behavior_modes']['mixed']}")
    print("=" * 80)
    
    return index_data


def print_scenario_table():
    """打印场景列表表格"""
    config_file = "<PATH>"
    
    with open(config_file, 'r', encoding='utf-8') as f:
        config = json.load(f)
    
    scenarios = config['scenarios']
    
    print("\n" + "=" * 100)
    print("12种基础仿真场景详细列表")
    print("=" * 100)
    print(f"{'ID':<4} {'场景名称':<25} {'船数':<4} {'本船角色':<12} {'行为模式':<10} {'会遇类型':<30}")
    print("-" * 100)
    
    for scenario in scenarios:
        print(f"{scenario['id']:<4} {scenario['name']:<25} {scenario['ship_num']:<4} "
              f"{'让路船' if 'giveway' in scenario.get('encounter_type', '') else '直航船':<12} "
              f"{scenario['behavior_mode']:<10} {scenario.get('encounter_type', 'N/A'):<30}")
    
    print("=" * 100)


if __name__ == "__main__":
    # 生成索引
    index_data = generate_scenario_index()
    
    # 打印表格
    print_scenario_table()
    
    print("\n✅ 场景库索引生成完成！")
    print("\n使用方法:")
    print("  1. 查看索引：cat scenario_library_index.json")
    print("  2. 运行全部场景：bash run_12_scenarios.sh all")
    print("  3. 运行单个场景：bash run_12_scenarios.sh <场景 ID>")
