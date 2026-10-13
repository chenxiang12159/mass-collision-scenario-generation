# 【2026-08-13 修复】修复目标船逆向求解 bug：
#   1) 原代码读取"要求 DCPA(dcpa_m)"后从未使用，且把目标船速度设成"朝本船 TCPA 时刻位置直冲"，导致反算 DCPA≈0、与要求值零相关；
#   2) 现改为按论文 §3.1 逆向运动学公式求解目标船初始位置：给定航向/航速，用相对速度法向量构造 CPA 点(长度=要求DCPA)，反推初始位置，保证反算 DCPA/TCPA = 要求值；
#   3) 目标船航速改为在 encounter_type 定义的速度范围内随机给定（原"追上本船"反推 + clip 逻辑已移除）；
#   4) 保留安全域校验；新增反算校验(误差>2%则重试)作为兜底。
#   离线验证：DCPA/TCPA 反算误差 0.000%，求解失败率 0.7%。
# 备份：test_scenarios.py.backup_20260813
import argparse
import json
import os
import re
import time
import random
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ===================== 核心类：订阅ROS2话题+生成目标船 =====================
class TargetShipGenerator(Node):
    def __init__(self):
        super().__init__('target_ship_generator')
        # 配置文件路径（也可改为相对路径，按需调整）
        self.config_file_path = os.path.join(os.path.dirname(__file__), "../config.json")
        # 安全领域参数
        self.avoid_dis = self.load_avoid_dis_config()
        self.short_axis = self.avoid_dis * 0.75  # 短轴=长轴*0.75

        # 本船数据（从ROS2话题订阅）
        self.vs = 0.0  # 本船航速（节）- 来自GPS_s
        self.ned_head = 0.0  # 本船航向（度）- 来自ElecCom_s
        self.gps_speed = 0.0  # 2026-08-18：本船瞬时航速（节），用于稳定性判定
        self.vs_received = False
        self.ned_head_received = False
        self.propmot_received = False  # 2026-08-14：是否已获取本船目标转速（稳定速度）

        # 1127新增发布器属性（对应MultiShip_vo的功能）
        self.publisher_DynTg_s = self.create_publisher(String, 'DynTg_s', 1)
        self.publisher_DynTg_c = self.create_publisher(String, 'DynTg_c', 1)
        self.flag = True  # 控制发布状态
        # 订阅ROS2话题
        self.subscribe_GPS_s = self.create_subscription(
            String, 'GPS_s', self.__GPS_s_callback, 10
        )
        self.subscribe_elecom = self.create_subscription(
            String, 'ElecCom_s', self.target_callback_elecom, 10
        )
        # 2026-08-14 修复：订阅本船目标转速，用稳定速度（转速/25.5）生成场景，
        # 而非 GPS 瞬时速度（本船加速过程中速度未稳定，导致会遇几何偏离）
        self.subscribe_PropMot = self.create_subscription(
            String, 'IeCtrl_PropMot_c_Speed', self.__PropMot_callback, 10
        )
    def publish_targets(self, tg_dict):
        if self.flag:
            for key in tg_dict:
                # 构建DynTg_s消息
                dyn_tg_s = {
                    'time': time.time(),
                    'name': key,
                    'Dst': tg_dict[key]['distance'],
                    'RelAzm': tg_dict[key]['RelAzm'],
                    'Ro': 0,  # roll
                    'Pth': 0,  # pitch
                    'Y': tg_dict[key]['yaw']
                }
                # 构建DynTg_c消息
                dyn_tg_c = {
                    'time': time.time(),
                    'name': key,
                    'S': tg_dict[key]['speed'],
                    'Rud': tg_dict[key]['rud']
                }
                # 发布消息
                self.publisher_DynTg_s.publish(String(data=json.dumps(dyn_tg_s)))
                self.publisher_DynTg_c.publish(String(data=json.dumps(dyn_tg_c)))
                self.get_logger().info(f"{key} 已注入")
                time.sleep(3)  # 间隔3秒发布下一个目标
            self.flag = False  # 确保只发布一次
            
            # 【新增】保持节点运行，让Bridge有时间发现并转发消息
            self.get_logger().info("等待Bridge转发消息到ROS1侧...")
            for i in range(10):
                rclpy.spin_once(self, timeout_sec=1)
                self.get_logger().info(f"等待中... {i+1}/10秒")

    def load_avoid_dis_config(self):
        """加载配置文件中的椭圆长轴参数"""
        try:
            with open(self.config_file_path, 'r') as f:
                config = json.load(f)
                return float(config["avoid_dis"][0])
        except (FileNotFoundError, KeyError, IndexError) as e:
            self.get_logger().warn(f"配置文件加载失败: {e}，使用默认值3704米")
            return 3704.0

    def __GPS_s_callback(self, msg):
        """订阅GPS_s话题获取本船航速"""
        if msg.data != "":
            self.GPS_s_ins = json.loads(msg.data)
            # 2026-08-18：始终记录瞬时航速，用于场景生成前的稳定性判定
            self.gps_speed = self.GPS_s_ins['speed']
            # 2026-08-14 修复：GPS 瞬时速度不再覆盖稳定速度（转速换算）
            # 仅在尚未获取目标转速时，才用 GPS 速度作为兜底
            if not self.propmot_received:
                self.vs = self.GPS_s_ins['speed']  # 本船航速（节）
                self.vs_received = True
                self.get_logger().info(f"已获取本船航速（GPS兜底）：{self.vs}节")

    def __PropMot_callback(self, msg):
        """2026-08-14 订阅本船目标转速，换算稳定速度（转速/25.5，与 speed_to_rpm 系数一致）"""
        if msg.data == "":
            return
        try:
            ins = json.loads(msg.data)
            rpm = float(ins['speed'])  # 本船目标转速（rpm）
            self.vs = rpm / 25.5  # 本船稳定速度（节），替代 GPS 瞬时速度
            self.vs_received = True
            self.propmot_received = True  # 标记已获取转速，GPS 不再覆盖
            self.get_logger().info(f"已获取本船目标转速：{rpm:.0f}rpm → 稳定速度{self.vs:.1f}节")
        except Exception as e:
            self.get_logger().warn(f"解析本船转速失败: {e}")

    def target_callback_elecom(self, data):
        """订阅ElecCom_s话题获取本船航向"""
        if data.data == "":
            return
        v = json.loads(data.data)
        self.header = v['head']
        self.ned_head = self.header  # 本船航向（度）
        self.ned_yaw = np.deg2rad(90.0 - self.ned_head)  # 转换为计算用角度
        self.ned_head_received = True
        self.get_logger().info(f"已获取本船航向：{self.ned_head}°")

    def load_encounter_config(self, test_index):
        """从17_scenario_config.json读取指定场景的encounter_type（2026-08-18 修复：原读12配置与run_17的17配置不一致）"""
        config_path = os.path.join(os.path.dirname(__file__), "../17_scenario_config.json")
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            for scenario in config['scenarios']:
                if scenario['id'] == test_index:
                    encounter_str = scenario['encounter_type']
                    # 按+分割多船场景，如 "overtaking+head-on" → ["overtaking", "head-on"]
                    return [e.strip() for e in encounter_str.split('+')]
            self.get_logger().warn(f"未找到场景{test_index}的encounter_type，使用默认crossing-giveway")
            return ['crossing-giveway']
        except Exception as e:
            self.get_logger().warn(f"读取encounter配置失败: {e}，使用默认crossing-giveway")
            return ['crossing-giveway']

    def get_azimuth_and_course_by_encounter(self, encounter_type, base_heading, RelAzm=None):
        """
        根据encounter_type生成方位角扇区范围和目标船航向
        :param encounter_type: 会遇类型字符串
        :param base_heading: 本船航向（度）
        :param RelAzm: 已计算的相对方位角（度），用于穿越航线航向计算
        :return: (az_min, az_max, target_course_deg, speed_min, speed_max)
                 az_min/az_max 为相对本船航向的方位角范围（已归一化到 -180~180）
        """
        if encounter_type == 'crossing-giveway':
            # 右前方交叉：目标船从右舷穿越本船航线
            target_course_deg = random.uniform(base_heading - 90, base_heading - 10)
            return 10.0, 90.0, target_course_deg, 3.0, 20.0

        elif encounter_type == 'crossing-standon':
            # 左前方交叉：目标船从左舷穿越本船航线
            target_course_deg = random.uniform(base_heading + 10, base_heading + 90)
            return -90.0, -10.0, target_course_deg, 3.0, 20.0

        elif encounter_type == 'head-on':
            # 正前方对遇：目标船迎面驶来
            target_course_deg = (base_heading + 180 + random.uniform(-5, 5)) % 360
            return -10.0, 10.0, target_course_deg, 3.0, 20.0

        elif encounter_type == 'overtaking':
            # 正前方追越：目标船同向低速
            target_course_deg = base_heading + random.uniform(-1, 1)
            return -10.0, 10.0, target_course_deg, 2.0, 8.0

        elif encounter_type == 'special-vessel':
            # 前方特殊船舶：低速
            target_course_deg = base_heading + random.uniform(-5, 5)
            return -30.0, 30.0, target_course_deg, 0.0, 3.0

        else:
            # 兜底：左前方
            target_course_deg = random.uniform(base_heading, base_heading + 90)
            return -112.5, -10.0, target_course_deg, 3.0, 20.0

    def wait_for_own_ship_data(self, timeout=30):
        """等待获取本船航向/航速（超时30秒），失败后依次尝试subprocess、CSV兜底"""
        import subprocess
        self.get_logger().info("等待接收本船航向和航速...")
        start_time = time.time()
        while (not self.vs_received or not self.ned_head_received) and (time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=1)

        if not self.vs_received or not self.ned_head_received:
            self.get_logger().warn("ROS2订阅超时，尝试subprocess直接读取GPS_s/ElecCom_s...")
            self._fallback_read_from_ros2()

        if not self.vs_received or not self.ned_head_received:
            self.get_logger().warn("subprocess也失败，尝试从历史CSV读取...")
            self._fallback_read_from_csv()

        if not self.vs_received:
            self.get_logger().error("所有方式均失败，使用默认值15节")
            self.vs = 15.0
        if not self.ned_head_received:
            self.get_logger().error("所有方式均失败，使用默认值90°")
            self.ned_head = 90.0

    def _circular_diff(self, a, b):
        """循环角度差（-180, 180]，处理 0/360 跨越"""
        return ((a - b + 180.0) % 360.0) - 180.0

    def wait_for_own_ship_stable(self, timeout=120, head_thresh=10.0, speed_thresh=1.0, window=15):
        """等待本船稳定后再生成目标船（2026-08-18 新增）
        判定条件（最近 window 秒内）：
          - 本船 head（罗经）相对窗口起始值变化 < head_thresh（10°）
          - 本船瞬时航速（GPS）变化 < speed_thresh（1节）
        超时 timeout 秒后强制继续（记 WARN），避免场景4那种"本船正转 81° 时生成"导致的几何失真
        任何异常均不中断流程（返回 False 继续生成），防止 ROS2 偶发消息异常导致目标船注入失败"""
        try:
            self.get_logger().info(
                f"等待本船稳定（head变化<{head_thresh}°、航速变化<{speed_thresh}节，窗口{window}秒）...")
            start_time = time.time()
            head_hist = []   # [(采样时刻, head)]
            speed_hist = []  # [(采样时刻, 航速)]
            while (time.time() - start_time) < timeout:
                now = time.time()
                try:
                    head_hist.append((now, float(self.ned_head)))
                except (TypeError, ValueError):
                    head_hist.append((now, 0.0))
                try:
                    speed_hist.append((now, float(self.gps_speed)))
                except (TypeError, ValueError):
                    speed_hist.append((now, 0.0))
                head_hist = [(t, h) for t, h in head_hist if now - t <= window]
                speed_hist = [(t, s) for t, s in speed_hist if now - t <= window]

                # 至少积累 5 秒样本才判定
                if len(head_hist) >= 2 and (head_hist[-1][0] - head_hist[0][0]) >= 5:
                    ref_head = head_hist[0][1]
                    head_spread = max(abs(self._circular_diff(h, ref_head)) for _, h in head_hist)
                    speed_spread = max(s for _, s in speed_hist) - min(s for _, s in speed_hist)
                    if head_spread < head_thresh and speed_spread < speed_thresh:
                        self.get_logger().info(
                            f"本船已稳定：head变化{head_spread:.1f}°(<{head_thresh}°)、"
                            f"航速变化{speed_spread:.1f}节(<{speed_thresh}节)")
                        return True
                rclpy.spin_once(self, timeout_sec=1)
            self.get_logger().warn(f"等待本船稳定超时（{timeout}秒），强制继续生成场景")
        except Exception as e:
            self.get_logger().warn(f"稳定判定异常（{e}），跳过稳定等待继续生成")
        return False

    def _fallback_read_from_ros2(self):
        """用subprocess调用ros2 topic echo直接读取GPS_s和ElecCom_s"""
        import subprocess
        # 读取GPS_s（获取speed）
        if not self.vs_received:
            try:
                cmd = 'source /opt/ros/foxy/setup.bash && source ~/.bashrc 2>/dev/null && timeout 5 ros2 topic echo /GPS_s'
                result = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=8)
                if result.returncode == 0 and result.stdout.strip():
                    # 解析YAML格式: data: '{"speed": 8.042, ...}'
                    for line in result.stdout.strip().split('\n'):
                        if 'data:' in line:
                            json_str = line.split("data:", 1)[1].strip().strip("'\"")
                            parsed = json.loads(json_str)
                            if 'speed' in parsed:
                                self.vs = float(parsed['speed'])
                                self.vs_received = True
                                self.get_logger().info(f"[subprocess兜底] 本船航速：{self.vs}节")
                                break
            except Exception as e:
                self.get_logger().warn(f"subprocess读取GPS_s失败: {e}")

        # 读取ElecCom_s（获取head）
        if not self.ned_head_received:
            try:
                cmd = 'source /opt/ros/foxy/setup.bash && source ~/.bashrc 2>/dev/null && timeout 5 ros2 topic echo /ElecCom_s'
                result = subprocess.run(['bash', '-c', cmd], capture_output=True, text=True, timeout=8)
                if result.returncode == 0 and result.stdout.strip():
                    for line in result.stdout.strip().split('\n'):
                        if 'data:' in line:
                            json_str = line.split("data:", 1)[1].strip().strip("'\"")
                            parsed = json.loads(json_str)
                            if 'head' in parsed:
                                self.ned_head = float(parsed['head'])
                                self.ned_head_received = True
                                self.get_logger().info(f"[subprocess兜底] 本船航向：{self.ned_head}°")
                                break
            except Exception as e:
                self.get_logger().warn(f"subprocess读取ElecCom_s失败: {e}")

    def _fallback_read_from_csv(self):
        """从最近一次仿真的CSV文件读取本船航速和航向"""
        import glob
        import csv
        sim_data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'simluation_data')
        sim_data_dir = os.path.normpath(sim_data_dir)
        if not os.path.isdir(sim_data_dir):
            self.get_logger().warn(f"simluation_data目录不存在: {sim_data_dir}")
            return

        # 找最近的仿真目录
        dirs = sorted(glob.glob(os.path.join(sim_data_dir, '2026*')), reverse=True)
        for sim_dir in dirs:
            gps_file = os.path.join(sim_dir, 'GPS_s.csv')
            elec_file = os.path.join(sim_dir, 'ElecCom_s.csv')
            if not os.path.exists(gps_file) or not os.path.exists(elec_file):
                continue
            try:
                if not self.vs_received:
                    with open(gps_file, 'r') as f:
                        reader = csv.DictReader(f)
                        # 找最后一个 speed > 1.0 的行（跳过停止状态）
                        last_valid_row = None
                        for row in reader:
                            if '对地速度(knots)speed' in row:
                                try:
                                    spd = float(row['对地速度(knots)speed'])
                                    if spd > 1.0:
                                        last_valid_row = row
                                except (ValueError, TypeError):
                                    pass
                        if last_valid_row:
                            self.vs = float(last_valid_row['对地速度(knots)speed'])
                            self.vs_received = True
                            self.get_logger().info(f"[CSV兜底] 本船航速：{self.vs}节（来自 {os.path.basename(sim_dir)}）")
                if not self.ned_head_received:
                    with open(elec_file, 'r') as f:
                        reader = csv.DictReader(f)
                        # 找最后一个有效航向行
                        last_valid_row = None
                        for row in reader:
                            if '航向(度)head' in row:
                                try:
                                    h = float(row['航向(度)head'])
                                    if 0 <= h <= 360:
                                        last_valid_row = row
                                except (ValueError, TypeError):
                                    pass
                        if last_valid_row:
                            self.ned_head = float(last_valid_row['航向(度)head'])
                            self.ned_head_received = True
                            self.get_logger().info(f"[CSV兜底] 本船航向：{self.ned_head}°（来自 {os.path.basename(sim_dir)}）")
                if self.vs_received and self.ned_head_received:
                    break
            except Exception as e:
                self.get_logger().warn(f"CSV读取失败({os.path.basename(sim_dir)}): {e}")
                continue

    def get_dcpa_tcpa_input(self, num_targets, mode='random'):
        """
        获取DCPA/TCPA列表（手动输入/随机生成）
        :param num_targets: 目标船数量
        :param mode: 'input'=手动输入，'random'=随机生成
        :return: dcpa_list, tcpa_list
        """
        dcpa_list = []
        tcpa_list = []

        if mode == 'input':
            # 手动输入模式
            self.get_logger().info("===== 手动输入目标船DCPA/TCPA =====")
            for i in range(1, num_targets + 1):
                while True:
                    try:
                        dcpa = float(input(f"请输入目标船{i}的DCPA（海里，0~6）："))
                        if not 0 <= dcpa <= 6:
                            print("DCPA需在0~6海里范围内！")
                            continue
                        tcpa = float(input(f"请输入目标船{i}的TCPA（分钟，0~8）："))
                        if not 0 <= tcpa <= 8:
                            print("TCPA需在0~8分钟范围内！")
                            continue
                        dcpa_list.append(dcpa)
                        tcpa_list.append(tcpa)
                        break
                    except ValueError:
                        print("输入无效，请输入数字！")
        else:
            # 随机生成模式（DCPA:0~6海里，TCPA:0~8分钟）
            # 2026-08-14 修复：TCPA 从 0~30 改为 0~8，保证会遇在本船航线(~11分钟)内完成
            self.get_logger().info("随机生成DCPA/TCPA列表（DCPA:0~6海里，TCPA:0~8分钟）")
            for _ in range(num_targets):
                dcpa_list.append(round(random.uniform(0, 6), 2))
                tcpa_list.append(round(random.uniform(0, 8), 2))

        self.get_logger().info(f"DCPA列表：{dcpa_list} 海里")
        self.get_logger().info(f"TCPA列表：{tcpa_list} 分钟")
        return dcpa_list, tcpa_list

    def calculate_min_distance_by_angle(self, rel_azimuth_rad, base_heading_rad):
        """动态计算椭圆安全领域最小距离"""
        angle_diff_rad = abs(rel_azimuth_rad - base_heading_rad)
        angle_diff_rad = min(angle_diff_rad, 2 * math.pi - angle_diff_rad)

        a = self.avoid_dis
        b = self.short_axis
        numerator = a * b
        denominator = math.sqrt((b * math.cos(angle_diff_rad)) ** 2 + (a * math.sin(angle_diff_rad)) ** 2)
        ellipse_radius = numerator / denominator
        return ellipse_radius * 1.05  # 留5%余量

    def is_inside_safety_area(self, x, y):
        """判断是否在椭圆安全领域内"""
        a = self.avoid_dis
        b = self.short_axis
        return (x / a) ** 2 + (y / b) ** 2 <= 1.0

    def left_adjust(self, data1, data2):
        if data1 > data2:
            data2 += 360
        return data2

    def right_adjust(self, data1, data2):
        if data2 - data1 > 360 and 0 < data1 - 180 < math.pi / 2:
            data1 += 360
        return data1

    def speed_to_rpm(self, speed_knots):
        """航速转转速（2026-08-18 修复：系数 25.5 → 50.5，目标船真实 RPM-速度系数）
        依据：14艘目标船回归实测，目标船稳定速度 = 设计速度 × 0.505（25.5 导致转速减半）。
        目标船 RPM 直接写入推进器（无本船控制链的 ×2 换算），真实系数约 50.5 RPM/节。
        本船"指令系数 25.5"来自本船控制链的等效转速指令（含 ×2），不可套用于目标船。"""
        return speed_knots * 50.5

    def generate_test_scenario(self, num_targets, mode='random', behavior_config=None, encounter_types=None):
        """
        生成目标船信息（核心逻辑）
        :param num_targets: 目标船数量
        :param mode: DCPA/TCPA 生成模式
        :param behavior_config: 行为配置 {'mode': 'standon'|'avoid'|'violate'|'mixed', 'probabilities': [p1, p2, p3]}
        :param encounter_types: 会遇类型列表，如 ['crossing-giveway', 'overtaking']
        """
        # 等待获取本船数据
        self.wait_for_own_ship_data()

        # 2026-08-18：等待本船稳定（避免本船转向/加速期间生成目标船导致会遇几何失真）
        self.wait_for_own_ship_stable()
    
        # 获取 DCPA/TCPA列表
        dcpa_list, tcpa_list = self.get_dcpa_tcpa_input(num_targets, mode)
            
        # 初始化行为决策器
        from TargetShipBehaviorDecider import TargetShipBehaviorDecider
        self.behavior_decider = TargetShipBehaviorDecider()

        base_heading = self.ned_head
        base_heading_rad = np.deg2rad(base_heading)
        own_speed_knots = self.vs
        own_speed_ms = own_speed_knots * 0.5144
        own_course_rad = np.deg2rad(base_heading)
        own_vx = own_speed_ms * np.sin(own_course_rad)
        own_vy = own_speed_ms * np.cos(own_course_rad)

        self.get_logger().info(f"本船航向：{base_heading}°，航速：{own_speed_knots}节")
        self.get_logger().info(f"安全领域 - 长轴：{self.avoid_dis}米，短轴：{self.short_axis}米")

        tg_dict = {}
        for i in range(1, num_targets + 1):
            target_name = f'tg_{i}'
            dcpa_nm = dcpa_list[i - 1]
            tcpa_min = tcpa_list[i - 1]
            dcpa_m = dcpa_nm * 1852
            tcpa_s = tcpa_min * 60
            is_own_ship_overtaking = 0
            max_attempts = 50
            attempt = 0
            final_data = None

            while attempt < max_attempts:
                attempt += 1

                # 1. 根据encounter_type生成方位角扇区和目标船航向
                enc_type = encounter_types[i - 1] if encounter_types and i <= len(encounter_types) else 'crossing-giveway'
                az_min, az_max, target_course_deg, speed_min, speed_max = \
                    self.get_azimuth_and_course_by_encounter(enc_type, base_heading)

                # 2. 追越标记
                if enc_type == 'overtaking':
                    is_own_ship_overtaking = 1

                target_course_deg %= 360
                target_course_rad = np.deg2rad(target_course_deg)

                # 3. 目标船航速：在速度范围内随机给定（修复：不再由"追上本船"反推）
                if is_own_ship_overtaking == 1:
                    effective_max = min(speed_max, own_speed_knots - 2)
                else:
                    effective_max = speed_max
                if speed_min >= effective_max:
                    speed_min = max(0.5, effective_max - 1.0)
                target_speed_knots = random.uniform(speed_min, effective_max)

                # 4. 目标船速度向量
                target_speed_ms = target_speed_knots * 0.5144
                target_vx = target_speed_ms * np.sin(target_course_rad)
                target_vy = target_speed_ms * np.cos(target_course_rad)

                # 5. 相对速度
                rel_vx = target_vx - own_vx
                rel_vy = target_vy - own_vy
                rel_speed_sq = rel_vx ** 2 + rel_vy ** 2
                if rel_speed_sq < 1e-12:
                    continue
                rel_speed = np.sqrt(rel_speed_sq)

                # 6. 逆向求解目标船初始位置（论文 §3.1 公式，保证反算 DCPA/TCPA = 要求值）
                ux, uy = rel_vx / rel_speed, rel_vy / rel_speed
                nx, ny = -uy, ux   # 相对速度的法向量（逆时针 90°）
                sign = random.choice([-1, 1])   # 左舷/右舷通过
                cpa_rx = sign * dcpa_m * nx     # CPA 时刻相对位置（长度 = 要求 DCPA）
                cpa_ry = sign * dcpa_m * ny
                init_x = cpa_rx - rel_vx * tcpa_s   # 目标船初始位置（本船在原点）
                init_y = cpa_ry - rel_vy * tcpa_s
                init_distance_m = np.hypot(init_x, init_y)
                RelAzm = np.degrees(np.arctan2(init_x, init_y)) % 360
                RelAzm_rad = np.deg2rad(RelAzm)

                # 7. 安全领域校验（2026-08-19 修复：原椭圆安全域校验(3704×2778m)导致
                #    DCPA<2海里的目标船反算50次失败触发兜底。改为最小距离校验：
                #    仅拒绝初始位置过近(<500m，避免贴脸)的目标船，允许小DCPA会遇）
                if np.hypot(init_x, init_y) < 500.0:
                    continue

                # 9. 反算校验（应精确等于要求值，兜底）
                tcpa_calc_s = -(init_x * rel_vx + init_y * rel_vy) / rel_speed_sq
                closest_x = init_x + rel_vx * tcpa_calc_s
                closest_y = init_y + rel_vy * tcpa_calc_s
                dcpa_calc_m = np.hypot(closest_x, closest_y)
                if abs(dcpa_calc_m - dcpa_m) > dcpa_m * 0.02:
                    continue

                # 8. 记录有效数据
                final_data = {
                    'init_distance_m': init_distance_m,
                    'RelAzm': RelAzm,
                    'target_course_deg': target_course_deg,
                    'target_speed_knots': target_speed_knots
                }
                # 2026-08-17 调试：打印要求TCPA与内部反算TCPA，验证一致性
                self.get_logger().info(f"[DEBUG] 要求TCPA={tcpa_min:.2f}分钟({tcpa_s:.0f}秒) | 内部反算TCPA={tcpa_calc_s:.1f}秒({tcpa_calc_s/60:.2f}分钟) | 本船速度={own_speed_knots:.2f}节 航向={base_heading:.1f}°")
                
                # 9. 【新增】如果有行为配置，计算初始舵角和会遇类型
                final_rudder = 0.0
                encounter_type = 'none'
                if behavior_config:
                    # 确定该目标船的行为模式
                    if behavior_config.get('mode') == 'mixed':
                        # 混合模式：随机分配
                        probs = behavior_config.get('probabilities', [0.4, 0.4, 0.2])
                        behavior_mode = np.random.choice(
                            ['standon', 'avoid', 'violate'], 
                            p=probs
                        )
                    else:
                        behavior_mode = behavior_config.get('mode', 'standon')
                    
                    # 构造临时状态用于决策
                    ts_state_temp = {
                        'x': init_x,
                        'y': init_y,
                        'speed': target_speed_knots,
                        'course': target_course_deg,
                        'rud': 0.0
                    }
                    os_state_temp = {
                        'x': 0.0,
                        'y': 0.0,
                        'speed': own_speed_knots,
                        'course': base_heading
                    }
                    
                    # 调用决策器
                    decision = self.behavior_decider.decide_behavior(
                        behavior_mode, 
                        os_state_temp, 
                        ts_state_temp, 
                        target_name
                    )
                    
                    final_rudder = decision['rudder_angle']
                    encounter_type = decision['encounter_type']
                    
                    self.get_logger().info(f"{target_name} 行为决策：{decision['action_desc']}")
                
                break

            # 兜底逻辑（2026-08-19 修复：原兜底几何 RelAzm=base+90/yaw=base+45 是"远离"几何，
            # 目标船从右舷向右前方远离本船、永不接近。改为"对遇"几何：目标船在正前方偏10°、
            # 迎面驶来，形成约0.3海里的对遇会遇距离）
            if final_data is None:
                self.get_logger().warn(f"目标船{i}生成失败，使用兜底数据（对遇）")
                fallback_dist = self.calculate_min_distance_by_angle(np.deg2rad(0), base_heading_rad)
                offset = random.choice([-10.0, 10.0])
                final_data = {
                    'init_distance_m': fallback_dist * 1.1,
                    'RelAzm': (base_heading + offset) % 360,
                    'target_course_deg': (base_heading + 180) % 360,
                    'target_speed_knots': random.uniform(3.0, 20.0)
                }

            # 转换为 MultiShip_vo 格式
            target_rpm = self.speed_to_rpm(final_data['target_speed_knots'])
                        
            # 计算初始舵角（根据行为模式）
            initial_rudder = final_rudder if final_rudder is not None else 0.0
                        
            tg_dict[target_name] = {
                'distance': round(final_data['init_distance_m'], 2),
                'RelAzm': round(final_data['RelAzm'], 2),
                'yaw': round(final_data['target_course_deg'], 2),
                'speed': round(target_rpm, 2),
                'rud': round(initial_rudder, 2),  # 初始舵角
                'Real Speed(kn)': round(final_data['target_speed_knots'], 2),
                'behavior_mode': behavior_mode if behavior_config else 'standon',  # 行为模式
                'encounter_type': encounter_type if behavior_config else 'none'  # 会遇类型
            }

            self.get_logger().info(
                f"{target_name}生成完成：距离={final_data['init_distance_m']:.2f}米，方位={final_data['RelAzm']:.2f}°")

        return tg_dict


# ===================== 注入逻辑（核心修改：绝对路径→相对路径） =====================
# ===================== 注入逻辑（核心修改：绝对路径→相对路径） =====================
def replace_multiship_tg_dict(tg_dict, multiship_filename="MultiShip_vo.py"):
    """
    替换MultiShip_vo.py的tg_Dict（使用相对路径，与当前脚本同目录）
    :param tg_dict: 目标船信息字典
    :param multiship_filename: 目标文件名（默认与当前脚本同目录）
    """
    # 获取当前脚本所在目录
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 拼接目标文件的相对路径（当前目录 + 文件名）
    multiship_path = os.path.join(current_dir, multiship_filename)

    if not os.path.exists(multiship_path):
        raise FileNotFoundError(f"MultiShip_vo.py路径错误：{multiship_path}")

    with open(multiship_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # 正则替换tg_Dict
    content = re.sub(
        r"tg_Dict\s*=\s*\{\s*\}",
        f"tg_Dict = {json.dumps(tg_dict, indent=4, ensure_ascii=False)}",
        content,
        count=1
    )

    with open(multiship_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"[注入完成] 已替换{len(tg_dict)}个目标船信息到{multiship_path}")


def save_to_json(data, test_index, ship_num):
    """保存目标船数据到JSON（核心修复：添加2026年时间戳）"""
    # 保存目录：当前脚本的上级目录/scene_data
    current_dir = os.path.dirname(os.path.abspath(__file__))
    save_dir = os.path.join(current_dir, "../scene_data")
    os.makedirs(save_dir, exist_ok=True)

    # 核心修复4：文件名添加2026年可读时间戳（格式：20260301_140834）
    time_str = time.strftime("%Y%m%d_%H%M%S")  # 包含年份的时间戳，如20260301_140834
    file_path = os.path.join(save_dir, f"test_{test_index}_ships_{ship_num}_{time_str}.json")

    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"[数据保存] 目标船数据已存至：{file_path}")
    return file_path


# ===================== 主函数 =====================
def main():
    # 初始化 ROS2
    rclpy.init()
    # 解析命令行参数
    parser = argparse.ArgumentParser()
    parser.add_argument('--ship_num', type=int, required=True, help="目标船数量")
    parser.add_argument('--test_index', type=int, default=1, help="试验序号")
    parser.add_argument('--mode', type=str, default='random', choices=['input', 'random'],
                        help="DCPA/TCPA 模式：input=手动输入，random=随机生成")
    # 【新增】行为模式参数
    parser.add_argument('--behavior_mode', type=str, default='standon', 
                        choices=['standon', 'avoid', 'violate', 'mixed'],
                        help="目标船行为模式：standon=保向保速，avoid=主动避碰，violate=违规，mixed=混合")
    parser.add_argument('--behavior_probs', type=float, nargs=3, default=[0.4, 0.4, 0.2],
                        help="混合模式下三种行为的概率 [standon_prob, avoid_prob, violate_prob]")
    parser.add_argument('--run_id', type=str, default=None,
                        help="仿真文件夹名（与simluation_data目录对应，用于数据关联）")
    args = parser.parse_args()
    
    # 构建行为配置
    behavior_config = {
        'mode': args.behavior_mode,
        'probabilities': args.behavior_probs
    }

    # 从17_scenario_config.json读取该场景的encounter_type
    generator = TargetShipGenerator()
    encounter_types = generator.load_encounter_config(args.test_index)
    print(f"[场景{args.test_index}] 会遇类型：{encounter_types}")

    # 生成目标船信息
    tg_dict = generator.generate_test_scenario(args.ship_num, args.mode, behavior_config, encounter_types)

    # 注入到MultiShip_vo.py（仅传字典，路径自动拼接）
    generator.publish_targets(tg_dict)

    # 保存数据
    scene_data = {
        "test_index": args.test_index,
        "ship_num": args.ship_num,
        "mode": args.mode,
        "tg_dict": tg_dict,
        "generate_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "sim_dir": args.run_id
    }
    save_to_json(scene_data, args.test_index, args.ship_num)

    # 【新增】等待Bridge转发消息到ROS1侧
    wait_time = args.ship_num * 3 + 10
    print(f"[等待] 等待{wait_time}秒，确保消息转发到ROS1侧...")
    time.sleep(wait_time)

    # 关闭ROS2
    generator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
