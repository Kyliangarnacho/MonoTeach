import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import time
import numpy as np
import threading
import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import matplotlib.transforms as transforms
from scipy.spatial.transform import Rotation as R



SERVO_CONFIG = {
    0: (1500, 2500, -120, 120, 0),  # 关节1
    1: (750, 1500, 0, 180, 90),  # 关节2
    2: (1500, 2250, -120, 120, 0),  # 关节3
    3: (750, 1500, -30, 210, 90),  # 关节4
    4: (1500, 2500, -120, 120, 0),  # 关节5
}


def DH_matrix(a, alpha, d, theta):  # 输入为deg
    alpha = np.deg2rad(alpha)
    theta = np.deg2rad(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)

    T = np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -sa * d],
        [st * sa, ct * sa, ca, ca * d],
        [0, 0, 0, 1]
    ])
    return T

def FK(theta1, theta2, theta3, theta4, theta5):  # 0为世界坐标系 1-5为机械臂的五个自由度 输入为deg
    T0_1 = DH_matrix(0, 0, 0.068, theta1)
    T1_2 = DH_matrix(0, 90, 0, theta2)
    T2_3 = DH_matrix(0.085, 180, 0, theta3)
    T3_4 = DH_matrix(0.080, 0, 0, theta4)
    T4_5 = DH_matrix(0, 90, 0.105, theta5)

    T0_5 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5
    x = T0_5[0, 3]
    y = T0_5[1, 3]
    z = T0_5[2, 3]
    return (x, y, z), T0_5

def IK(T_target: list):
    T_target = np.array(T_target)
    px = T_target[0, 3]
    py = T_target[1, 3]
    pz = T_target[2, 3]
    XT = T_target[0:3, 0]
    YT = T_target[0:3, 1]
    ZT = T_target[0:3, 2]
    s = px ** 2 + py ** 2
    if s < 1e-6:
        M = np.array([1, 0, 0])
        theta1 = 0
    else:
        M = 1 / (px ** 2 + py ** 2) * np.array([-py, px, 0])
        theta1 = np.arctan2(py, px)
    K = np.cross(M, ZT)
    ZT_new = np.cross(K, M)
    cos_theta = np.dot(ZT, ZT_new)
    sin_theta = np.dot(np.cross(ZT, ZT_new), K)
    YT_new = cos_theta * YT + sin_theta * (np.cross(K, YT)) + (1 - cos_theta) * np.dot(K, YT) * K
    XT_new = np.cross(YT_new, ZT_new)

    T_target_new = np.eye(4)
    T_target_new[0:3, 0] = XT_new
    T_target_new[0:3, 1] = YT_new
    T_target_new[0:3, 2] = ZT_new
    T_target_new[0:3, 3] = [px, py, pz]

    r11, r12, r13 = T_target_new[0, 0], T_target_new[0, 1], T_target_new[0, 2]
    r21, r22, r23 = T_target_new[1, 0], T_target_new[1, 1], T_target_new[1, 2]
    r31, r32, r33 = T_target_new[2, 0], T_target_new[2, 1], T_target_new[2, 2]

    a11, a12, a13 = T_target[0, 0], T_target[0, 1], T_target[0, 2]
    a21, a22, a23 = T_target[1, 0], T_target[1, 1], T_target[1, 2]

    c1, s1 = np.cos(theta1), np.sin(theta1)
    
    theta5 = np.arctan2(a21 * c1 - a11 * s1, a22 * c1 - a12 * s1)

    theta234 = np.arctan2(r33, np.sqrt(r13 ** 2 + r23 ** 2))  # 实际是 2 - 3 - 4
    r_tool = np.sqrt(px ** 2 + py ** 2)
    z_tool = pz
    r_wrist = r_tool - 0.105 * np.cos(theta234)
    z_wrist = z_tool - 0.068 - 0.105 * np.sin(theta234)
    cos_theta3 = (r_wrist ** 2 + z_wrist ** 2 - 0.013625) / (2 * 0.08 * 0.085)
    cos_theta3 = np.clip(cos_theta3, -1, 1)
    theta3 = np.arccos(cos_theta3)
    theta2 = np.arctan2(z_wrist, r_wrist) + np.arctan2(0.08 * np.sin(theta3), 0.085 + 0.08 * np.cos(theta3))
    theta4 = theta2 - theta3 - theta234
    return theta1, theta2, theta3, theta4 + np.pi / 2, theta5  # 输出为rad

def solve_ik(x, y, z, pitch_deg):  # 90表示垂直竖直向下，0表示水平向前
    pitch_rad = np.deg2rad(pitch_deg)
    s = x ** 2 + y ** 2
    if s < 1e-6:
        theta1 = 0
    else:
        theta1 = np.arctan2(y, x)
    nz = -np.sin(pitch_rad)
    nr = np.cos(pitch_rad)
    nx = nr * np.cos(theta1)
    ny = nr * np.sin(theta1)
    T_target_z = np.array([nx, ny, nz])
    T_target_y = np.array([-np.sin(theta1), np.cos(theta1), 0])
    T_target_x = np.cross(T_target_y, T_target_z)
    T_target = np.eye(4)
    T_target[0: 3, 0] = T_target_x
    T_target[0: 3, 1] = T_target_y
    T_target[0: 3, 2] = T_target_z
    T_target[0: 3, 3] = [x, y, z]

    try:
        rad_theta = IK(T_target)
        deg_theta = list(np.degrees(rad_theta))
        return deg_theta
    except:
        print("求逆解出错")
        return [0, 0, 0, 0, 0]

def calculate_target_matrix(x, y, z, pitch_deg):
    """根据输入的XYZ和俯仰角计算目标变换矩阵"""
    pitch_rad = np.deg2rad(pitch_deg)
    s = x ** 2 + y ** 2
    if s < 1e-6:
        theta1 = 0
    else:
        theta1 = np.arctan2(y, x)
    
    # 计算Z轴向量（接近矢量）
    nz = -np.sin(pitch_rad)
    nr = np.cos(pitch_rad)
    nx = nr * np.cos(theta1)
    ny = nr * np.sin(theta1)
    T_target_z = np.array([nx, ny, nz])
    
    # 计算Y轴向量（方向矢量）
    T_target_y = np.array([-np.sin(theta1), np.cos(theta1), 0])
    
    # 计算X轴向量（法线矢量）
    T_target_x = np.cross(T_target_y, T_target_z)
    
    # 构建完整的齐次变换矩阵
    T_target = np.eye(4)
    T_target[0:3, 0] = T_target_x  # X轴
    T_target[0:3, 1] = T_target_y  # Y轴  
    T_target[0:3, 2] = T_target_z  # Z轴
    T_target[0:3, 3] = [x, y, z]   # 位置向量
    
    return T_target

def format_matrix(matrix, decimals=4):
    """将矩阵格式化为易读的字符串"""
    formatted = ""
    for i in range(4):
        row = "["
        for j in range(4):
            value = matrix[i, j]
            if abs(value) < 1e-10:  # 处理接近零的值
                value = 0.0
            row += f"{value:8.{decimals}f}"
            if j < 3:
                row += ", "
        row += "]"
        if i < 3:
            row += "\n"
        formatted += row
    return formatted

# ==================== PWM相关函数开始 ====================
def angle2pwm(i: int, theta: float, is_degree=True):  # 0-4为机械臂的五个自由度 输入为角度
    """角度转PWM函数 - 将关节角度转换为PWM信号值"""
    if not is_degree:
        theta = np.rad2deg(theta)  # IK输出是弧度，这里转为角度来计算pwm
    if i == 0:
        k = 8.33333333
        pwm0 = 1500
    elif i == 1:
        k = 8.33333333
        pwm0 = 750
    elif i == 2:
        k = 8.33333333
        pwm0 = 1500
    elif i == 3:
        k = 8.33333333
        pwm0 = 750
    elif i == 4:
        k = 8.33333333
        pwm0 = 1500
    else:
        print("警告警告警告")
    pwm = pwm0 + k * theta
    pwm = np.clip(pwm, 500, 2500)  # PWM值限制在500-2500范围内
    return int(pwm)
# ==================== PWM相关函数结束 ====================

# ==================== 轨迹规划相关函数开始 ====================
class TrajectoryPlanner:
    """轨迹规划器 - 使用五次多项式插值实现平滑轨迹规划"""
    
    def __init__(self):
        self.waypoints = []  # 路径点列表
        self.trajectory_points = []  # 轨迹点列表
        self.joint_trajectory = []  # 关节轨迹
        # 自定义轨迹相关变量
        self.custom_waypoints = []  # 存储自定义中间点
        self.custom_trajectory = []  # 存储自定义轨迹
        
    def five_order_polynomial(self, t, t_total, q0, qf, v0=0, vf=0, a0=0, af=0):
        """五次多项式插值函数[6](@ref)
        
        参数:
            t: 当前时间
            t_total: 总时间
            q0: 起始位置
            qf: 终止位置
            v0: 起始速度 (默认0)
            vf: 终止速度 (默认0)
            a0: 起始加速度 (默认0)
            af: 终止加速度 (默认0)
            
        返回:
            position, velocity, acceleration
        """
        if t_total == 0:
            return qf, 0, 0
            
        # 归一化时间
        tau = t / t_total
        
        # 五次多项式系数[6](@ref)
        a0 = q0
        a1 = v0
        a2 = a0 / 2.0
        a3 = (20*qf - 20*q0 - (8*vf + 12*v0)*t_total - (3*a0 - af)*t_total**2) / (2*t_total**3)
        a4 = (30*q0 - 30*qf + (14*vf + 16*v0)*t_total + (3*a0 - 2*af)*t_total**2) / (2*t_total**4)
        a5 = (12*qf - 12*q0 - (6*vf + 6*v0)*t_total - (a0 - af)*t_total**2) / (2*t_total**5)
        
        # 计算位置、速度、加速度
        position = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
        velocity = a1 + 2*a2*t + 3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4
        acceleration = 2*a2 + 6*a3*t + 12*a4*t**2 + 20*a5*t**3
        
        return position, velocity, acceleration
    
    def plan_cartesian_trajectory(self, start_pose, target_pose, num_points=10, total_time=5.0):
        """笛卡尔空间轨迹规划[1,4](@ref)
        
        参数:
            start_pose: 起始位姿 [x, y, z, pitch]
            target_pose: 目标位姿 [x, y, z, pitch]
            num_points: 中间点数量
            total_time: 总时间
            
        返回:
            轨迹点列表
        """
        self.waypoints = []
        self.trajectory_points = []
        
        # 生成时间序列
        t_values = np.linspace(0, total_time, num_points + 2)  # 包括起点和终点
        
        # 对每个自由度进行五次多项式插值
        for i in range(4):  # x, y, z, pitch
            q0 = start_pose[i]
            qf = target_pose[i]
            
            for j, t in enumerate(t_values):
                if j >= len(self.trajectory_points):
                    self.trajectory_points.append([0, 0, 0, 0])
                
                position, _, _ = self.five_order_polynomial(t, total_time, q0, qf)
                self.trajectory_points[j][i] = position
        
        return self.trajectory_points
    
    def calculate_joint_trajectory(self, trajectory_points, current_joint_angles):
        """计算关节空间轨迹[7](@ref)
        
        参数:
            trajectory_points: 笛卡尔空间轨迹点
            current_joint_angles: 当前关节角度
            
        返回:
            关节轨迹列表
        """
        self.joint_trajectory = []
        prev_joint_angles = np.array(current_joint_angles)
        
        for i, point in enumerate(trajectory_points):
            x, y, z, pitch = point
            
            # 计算目标变换矩阵
            T_target = calculate_target_matrix(x, y, z, pitch)
            
            try:
                # 计算逆运动学解
                joint_angles = solve_ik(x, y, z, pitch)
                joint_angles = np.array(joint_angles)
                
                # 处理多解问题：选择与上一组关节角度最接近的解[7](@ref)
                if i > 0 and len(self.joint_trajectory) > 0:
                    # 计算关节角度变化量
                    angle_diff = np.sum(np.abs(joint_angles - prev_joint_angles))
                    
                    # 这里可以添加其他解的尝试和比较，选择变化最小的解
                    # 目前直接使用第一个解
                    pass
                
                # 检查关节限位
                if self.check_joint_limits(joint_angles):
                    self.joint_trajectory.append({
                        'point_index': i,
                        'cartesian_point': point,
                        'joint_angles': joint_angles,
                        'T_matrix': T_target
                    })
                    prev_joint_angles = joint_angles
                else:
                    print(f"警告: 路径点 {i} 的关节角度超出限位")
                    return None
                    
            except Exception as e:
                print(f"逆运动学求解失败 at point {i}: {e}")
                return None
        
        return self.joint_trajectory
    
    def check_joint_limits(self, joint_angles):
        """检查关节角度是否在限位内"""
        limits = [(-120, 120), (0, 180), (-120, 120), (-30, 210), (-120, 120)]
        
        for i, angle in enumerate(joint_angles):
            if angle < limits[i][0] or angle > limits[i][1]:
                return False
        return True

# ==================== 轨迹规划相关函数结束 ====================
# ==================== 画圆功能核心类开始 ====================
class CirclePlanner:
    """画圆轨迹规划器 - 专门处理平面画圆功能"""
    
    def __init__(self):
        self.circle_points = []  # 圆形路径点
        self.joint_trajectory = []  # 关节轨迹
        self.plane_normal = None  # 平面法向量
        self.plane_origin = None  # 平面原点
        
    def define_plane_from_three_points(self, point1, point2, point3):
        """通过三个点定义平面
        参数: point1, point2, point3 - 三个三维点坐标
        返回: 平面法向量和原点
        """
        point1 = np.array(point1)
        point2 = np.array(point2)
        point3 = np.array(point3)
        
        # 计算平面法向量
        v1 = point2 - point1
        v2 = point3 - point1
        normal = np.cross(v1, v2)
        normal = normal / np.linalg.norm(normal)  # 单位化
        
        self.plane_normal = normal
        self.plane_origin = point1
        
        return normal, point1
    
    def generate_circle_points(self, center, radius, num_points, normal=None):
        """在指定平面上生成圆形路径点
        参数: center - 圆心坐标 [x, y, z]
              radius - 半径
              num_points - 点数（360的约数）
              normal - 平面法向量，如果为None则使用已定义的平面
        """
        if normal is None:
            if self.plane_normal is None:
                raise ValueError("未定义平面，请先调用define_plane_from_three_points")
            normal = self.plane_normal
        
        center = np.array(center)
        
        # 生成平面上的两个正交基向量
        if not np.allclose(normal, [0, 0, 1]):
            u = np.array([0, 0, 1])
        else:
            u = np.array([1, 0, 0])
        
        # 计算平面上的两个正交向量
        v1 = np.cross(normal, u)
        v1 = v1 / np.linalg.norm(v1)
        v2 = np.cross(normal, v1)
        v2 = v2 / np.linalg.norm(v2)
        
        # 生成圆形点
        self.circle_points = []
        angle_step = 2 * np.pi / num_points
        
        for i in range(num_points + 1):  # +1 用于闭合圆
            angle = i * angle_step
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            
            # 将点从平面坐标系转换到世界坐标系
            point = center + x * v1 + y * v2
            self.circle_points.append(point)
        
        return self.circle_points
    
    def calculate_circle_trajectory(self, center, radius, num_points, pitch=90, 
                                  current_joint_angles=None):
        """计算画圆的完整关节轨迹
        参数: center - 圆心
              radius - 半径
              num_points - 点数
              pitch - 末端俯仰角
              current_joint_angles - 当前关节角度，用于多解选择
        """
        if current_joint_angles is None:
            current_joint_angles = [0, 90, 0, 90, 0]
        
        # 生成圆形点
        circle_points = self.generate_circle_points(center, radius, num_points)
        
        self.joint_trajectory = []
        prev_angles = np.array(current_joint_angles)
        
        for i, point in enumerate(circle_points):
            x, y, z = point
            
            try:
                # 计算逆运动学解
                joint_angles = solve_ik(x, y, z, pitch)
                joint_angles = np.array(joint_angles)
                
                # 处理多解问题：选择与上一组关节角度最接近的解
                if i > 0 and len(self.joint_trajectory) > 0:
                    # 这里可以扩展为检查所有可能的解，选择变化最小的
                    # 目前使用基本解
                    pass
                
                # 检查关节限位
                if self.check_joint_limits(joint_angles):
                    trajectory_point = {
                        'point_index': i,
                        'cartesian_point': [x, y, z, pitch],
                        'joint_angles': joint_angles,
                        'position': point
                    }
                    self.joint_trajectory.append(trajectory_point)
                    prev_angles = joint_angles
                else:
                    print(f"警告: 圆点 {i} 的关节角度超出限位")
                    return None
                    
            except Exception as e:
                print(f"逆运动学求解失败 at circle point {i}: {e}")
                return None
        
        return self.joint_trajectory
    
    def check_joint_limits(self, joint_angles):
        """检查关节角度限位"""
        limits = [(-120, 120), (0, 180), (-120, 120), (-30, 210), (-120, 120)]
        
        for i, angle in enumerate(joint_angles):
            if angle < limits[i][0] or angle > limits[i][1]:
                return False
        return True

    def get_circle_metrics(self):
        """获取圆形轨迹的度量信息"""
        if not self.circle_points:
            return None
            
        points = np.array(self.circle_points)
        circumference = 2 * np.pi * np.linalg.norm(points[0] - points[len(points)//2])
        
        return {
            'point_count': len(self.circle_points),
            'approx_circumference': circumference,
            'points_array': points
        }
# ==================== 画圆功能核心类结束 ====================

# ==================== 上位机 ====================
class RobotArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("5 DOF 机械臂上位机")
        # 增大窗口尺寸以适应新的布局比例
        self.root.geometry("1400x900")  # 增大窗口以适应新标签页
        
        # 设置权重使左侧可以扩展得更大
        self.root.grid_columnconfigure(0, weight=3)  # 左侧权重为3
        self.root.grid_columnconfigure(1, weight=1)  # 右侧权重为1

        self.ser = None
        self.is_connected = False
        self.current_angles = [0, 90, 0, 90, 0]
        
        # 初始化轨迹规划器
        self.trajectory_planner = TrajectoryPlanner()
        self.trajectory_thread = None
        self.is_running_trajectory = False

        # 创建界面
        self.create_interface()
        
        # 延迟初始化矩阵显示，避免启动错误
        self.root.after(100, self.initialize_matrix_display)

    def initialize_matrix_display(self):
        """延迟初始化矩阵显示，避免启动时的错误"""
        try:
            self.update_fk_matrix()
            self.update_ik_matrix()
        except Exception as e:
            # 静默处理初始化错误，不显示给用户
            print(f"矩阵初始化轻微问题: {str(e)}")

    def create_interface(self):
        """创建主界面"""
    # 使用Notebook（标签页）来组织界面
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # 创建原有主界面框架作为第一个标签页
        main_frame = ttk.Frame(self.notebook)
        self.notebook.add(main_frame, text="基本控制")
    
    # 创建轨迹规划标签页
        trajectory_frame = ttk.Frame(self.notebook)
        self.notebook.add(trajectory_frame, text="轨迹规划")
    
    # 创建画圆功能标签页（新增） - 修改这里
        circle_frame = ttk.Frame(self.notebook)
        self.notebook.add(circle_frame, text="画圆功能")  # 只在这里添加一次
    
    # 在原有主界面中创建内容
        self.create_main_interface(main_frame)
    
    # 创建轨迹规划界面
        self.create_trajectory_interface(trajectory_frame)
    
    # 创建画圆功能界面（新增）- 直接传递circle_frame
        self.create_circle_drawing_interface(circle_frame)

    def create_main_interface(self, parent):
        """创建原有主界面"""
        # 使用grid布局管理器替代pack，以便更好控制比例
        main_frame = ttk.Frame(parent)
        main_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)
        main_frame.grid_columnconfigure(0, weight=3)  # 左侧权重为3
        main_frame.grid_columnconfigure(1, weight=1)  # 右侧权重为1
        main_frame.grid_rowconfigure(0, weight=1)
        
        # 左侧控制面板 - 权重更大
        left_frame = ttk.Frame(main_frame, width=700)  # 增加宽度
        left_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        left_frame.grid_propagate(False)  # 防止自动调整大小

        # 右侧显示面板 - 权重较小
        right_frame = ttk.Frame(main_frame, width=300)  # 减小宽度
        right_frame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
        right_frame.grid_propagate(False)  # 防止自动调整大小

        # 创建各个功能区域
        self.create_serial_section(left_frame)
        self.create_joint_control(left_frame)
        self.create_cartesian_control(left_frame)
        self.create_status_section(right_frame)

    def create_trajectory_interface(self, parent):
        """创建轨迹规划界面"""
        # 主框架
        main_frame = ttk.Frame(parent)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 左侧控制面板
        control_frame = ttk.LabelFrame(main_frame, text="轨迹规划控制")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5, ipadx=5, ipady=5)
        
        # 右侧显示面板
        display_frame = ttk.Frame(main_frame)
        display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 创建控制面板内容
        self.create_trajectory_control(control_frame)
        
        # 创建显示面板内容
        self.create_trajectory_display(display_frame)
        

    def create_trajectory_control(self, parent):
        """创建轨迹规划控制面板"""
        # 目标点设置
        target_frame = ttk.LabelFrame(parent, text="目标点设置")
        target_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 当前位置显示
        ttk.Label(target_frame, text="当前位置:", font=("Arial", 9, "bold")).grid(row=0, column=0, sticky=tk.W, pady=2)
        self.current_pos_label = ttk.Label(target_frame, text="X:0.000, Y:0.000, Z:0.000, Pitch:90.0°")
        self.current_pos_label.grid(row=0, column=1, columnspan=3, sticky=tk.W, pady=2)
        
        # 目标点输入
        ttk.Label(target_frame, text="X目标:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.target_x_var = tk.DoubleVar(value=0.3)
        ttk.Entry(target_frame, textvariable=self.target_x_var, width=8).grid(row=1, column=1, pady=2)
        ttk.Label(target_frame, text="m").grid(row=1, column=2, pady=2)
        
        ttk.Label(target_frame, text="Y目标:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.target_y_var = tk.DoubleVar(value=0.1)
        ttk.Entry(target_frame, textvariable=self.target_y_var, width=8).grid(row=2, column=1, pady=2)
        ttk.Label(target_frame, text="m").grid(row=2, column=2, pady=2)
        
        ttk.Label(target_frame, text="Z目标:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.target_z_var = tk.DoubleVar(value=0.2)
        ttk.Entry(target_frame, textvariable=self.target_z_var, width=8).grid(row=3, column=1, pady=2)
        ttk.Label(target_frame, text="m").grid(row=3, column=2, pady=2)
        
        ttk.Label(target_frame, text="俯仰角:").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.target_pitch_var = tk.DoubleVar(value=0)
        ttk.Entry(target_frame, textvariable=self.target_pitch_var, width=8).grid(row=4, column=1, pady=2)
        ttk.Label(target_frame, text="°").grid(row=4, column=2, pady=2)
        
        # 轨迹参数设置
        param_frame = ttk.LabelFrame(parent, text="轨迹参数")
        param_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(param_frame, text="中间点数:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.point_count_var = tk.IntVar(value=5)
        point_combo = ttk.Combobox(param_frame, textvariable=self.point_count_var, 
                                 values=[3, 5, 8, 10, 15], width=6)
        point_combo.grid(row=0, column=1, pady=2)
        
        # 只保留点间隔设置，删除总时间设置
        ttk.Label(param_frame, text="点间隔:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.point_interval_var = tk.DoubleVar(value=2.0)
        ttk.Entry(param_frame, textvariable=self.point_interval_var, width=8).grid(row=1, column=1, pady=2)
        ttk.Label(param_frame, text="s").grid(row=1, column=2, pady=2)
        # 自定义中间点设置
        custom_frame = ttk.LabelFrame(parent, text="自定义中间点设置")
        custom_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(custom_frame, text="自定义中间点（每行格式: x,y,z,pitch）:").pack(anchor=tk.W)
        self.custom_points_text = tk.Text(custom_frame, height=6, width=50)
        self.custom_points_text.pack(fill=tk.X, padx=5, pady=2)
        
        # 示例数据按钮
        example_frame = ttk.Frame(custom_frame)
        example_frame.pack(fill=tk.X, padx=5, pady=2)
        
        ttk.Button(example_frame, text="加载示例数据", command=self.load_example_points).pack(side=tk.LEFT, padx=2)
        ttk.Button(example_frame, text="清空自定义点", command=self.clear_custom_points).pack(side=tk.LEFT, padx=2)
        ttk.Button(example_frame, text="加载自定义点", command=self.load_custom_points).pack(side=tk.LEFT, padx=2)
        
        # 状态显示
        self.custom_points_status = ttk.Label(custom_frame, text="就绪", foreground="blue")
        self.custom_points_status.pack(pady=2)
        # 控制按钮
        button_frame = ttk.Frame(parent)
        button_frame.pack(fill=tk.X, padx=5, pady=10)
        
        ttk.Button(button_frame, text="规划轨迹", command=self.plan_trajectory).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="执行轨迹", command=self.execute_trajectory).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="停止轨迹", command=self.stop_trajectory).pack(side=tk.LEFT, padx=2)
        ttk.Button(button_frame, text="清除轨迹", command=self.clear_trajectory).pack(side=tk.LEFT, padx=2)
        # 在现有的控制按钮后添加
        ttk.Button(button_frame, text="规划自定义轨迹", command=self.plan_custom_trajectory).pack(side=tk.LEFT, padx=2)
        # 在自定义轨迹按钮后添加
        ttk.Button(button_frame, text="执行自定义轨迹", command=self.execute_custom_trajectory).pack(side=tk.LEFT, padx=2)
        # 状态显示
        status_frame = ttk.LabelFrame(parent, text="轨迹状态")
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.trajectory_status_var = tk.StringVar(value="就绪")
        ttk.Label(status_frame, textvariable=self.trajectory_status_var).pack(pady=5)



    def create_trajectory_display(self, parent):
        """创建轨迹规划显示面板"""
        # 使用PanedWindow实现可调整的分割
        paned_window = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # 上部：路径点显示
        points_frame = ttk.LabelFrame(paned_window, text="路径点详情")
        paned_window.add(points_frame, weight=1)
        
        # 创建树形视图显示路径点
        columns = ("序号", "X坐标", "Y坐标", "Z坐标", "俯仰角", "关节1", "关节2", "关节3", "关节4", "关节5")
        self.points_tree = ttk.Treeview(points_frame, columns=columns, show="tree headings", height=8)
        
        # 设置列宽
        for col in columns:
            self.points_tree.heading(col, text=col)
            self.points_tree.column(col, width=70)
        
        # 添加滚动条
        scrollbar = ttk.Scrollbar(points_frame, orient=tk.VERTICAL, command=self.points_tree.yview)
        self.points_tree.configure(yscrollcommand=scrollbar.set)
        
        self.points_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 下部：图表显示
        chart_frame = ttk.LabelFrame(paned_window, text="轨迹可视化")
        paned_window.add(chart_frame, weight=2)
        
        # 创建matplotlib图表
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, chart_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 目标变换矩阵显示
        matrix_frame = ttk.LabelFrame(parent, text="目标变换矩阵")
        matrix_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.target_matrix_text = tk.Text(matrix_frame, height=6, width=80)
        scrollbar_y = ttk.Scrollbar(matrix_frame, orient=tk.VERTICAL, command=self.target_matrix_text.yview)
        scrollbar_x = ttk.Scrollbar(matrix_frame, orient=tk.HORIZONTAL, command=self.target_matrix_text.xview)
        self.target_matrix_text.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)
        
        self.target_matrix_text.grid(row=0, column=0, sticky="nsew")
        scrollbar_y.grid(row=0, column=1, sticky="ns")
        scrollbar_x.grid(row=1, column=0, sticky="ew")
        
        matrix_frame.grid_rowconfigure(0, weight=1)
        matrix_frame.grid_columnconfigure(0, weight=1)

    def create_serial_section(self, parent):
        """创建串口设置区域"""
        frame = ttk.LabelFrame(parent, text="串口设置")
        frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(frame, text="串口:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.port_combo = ttk.Combobox(frame, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=2)
        self.refresh_ports()

        ttk.Label(frame, text="波特率:").grid(row=0, column=2, padx=5, pady=2, sticky=tk.W)
        self.baud_combo = ttk.Combobox(frame, values=["9600", "115200"], width=10)
        self.baud_combo.set("115200")
        self.baud_combo.grid(row=0, column=3, padx=5, pady=2)

        self.connect_btn = ttk.Button(frame, text="连接", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=5, pady=2)

        ttk.Button(frame, text="刷新", command=self.refresh_ports).grid(row=0, column=5, padx=5, pady=2)

    def create_joint_control(self, parent):
        """创建关节空间控制区域"""
        frame = ttk.LabelFrame(parent, text="关节空间控制")
        frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)  # 允许扩展

        # 使用PanedWindow实现可调整的分割
        paned_window = ttk.PanedWindow(frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧关节控制框架
        left_joint_frame = ttk.Frame(paned_window)
        paned_window.add(left_joint_frame, weight=2)  # 左侧权重

        # 右侧矩阵显示框架
        right_matrix_frame = ttk.Frame(paned_window)
        paned_window.add(right_matrix_frame, weight=1)  # 右侧权重

        self.joint_vars = []
        joints = ["关节1", "关节2", "关节3", "关节4", "关节5"]
        limits = [(-120, 120), (0, 180), (-120, 120), (-30, 210), (-120, 120)]

        for i, (joint, limit) in enumerate(zip(joints, limits)):
            ttk.Label(left_joint_frame, text=f"{joint}:").grid(row=i, column=0, padx=5, pady=2, sticky=tk.W)

            var = tk.DoubleVar(value=0)
            self.joint_vars.append(var)
            scale = ttk.Scale(left_joint_frame, from_=limit[0], to=limit[1], variable=var,
                            orient=tk.HORIZONTAL, length=200)  # 增加滑动条长度
            scale.grid(row=i, column=1, padx=5, pady=2, sticky=tk.EW)

            entry = ttk.Entry(left_joint_frame, width=8, textvariable=var)
            entry.grid(row=i, column=2, padx=5, pady=2)
            ttk.Label(left_joint_frame, text="°").grid(row=i, column=3, padx=5, pady=2)

        # 配置列权重使滑动条可以扩展
        left_joint_frame.grid_columnconfigure(1, weight=1)

        # 控制按钮
        btn_frame = ttk.Frame(left_joint_frame)
        btn_frame.grid(row=5, column=0, columnspan=4, pady=10)

        ttk.Button(btn_frame, text="读取角度", command=self.read_angles).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="设置角度", command=self.set_angles).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="回零", command=self.home_position).pack(side=tk.LEFT, padx=2)

        # 正运动学变换矩阵显示
        ttk.Label(right_matrix_frame, text="正运动学变换矩阵:", font=("Arial", 10, "bold")).pack(anchor=tk.W, pady=(0, 5))
        
        # 创建框架包含文本框和滚动条
        matrix_frame = ttk.Frame(right_matrix_frame)
        matrix_frame.pack(fill=tk.BOTH, expand=True)
        
        self.fk_matrix_text = tk.Text(matrix_frame, height=8, width=35, wrap=tk.NONE)
        fk_scrollbar_x = ttk.Scrollbar(matrix_frame, orient=tk.HORIZONTAL, command=self.fk_matrix_text.xview)
        fk_scrollbar_y = ttk.Scrollbar(matrix_frame, orient=tk.VERTICAL, command=self.fk_matrix_text.yview)
        self.fk_matrix_text.configure(xscrollcommand=fk_scrollbar_x.set, yscrollcommand=fk_scrollbar_y.set)
        
        self.fk_matrix_text.grid(row=0, column=0, sticky="nsew")
        fk_scrollbar_y.grid(row=0, column=1, sticky="ns")
        fk_scrollbar_x.grid(row=1, column=0, sticky="ew")
        
        matrix_frame.grid_rowconfigure(0, weight=1)
        matrix_frame.grid_columnconfigure(0, weight=1)

        ttk.Button(right_matrix_frame, text="更新正运动学矩阵", command=self.update_fk_matrix).pack(pady=5)

    def create_cartesian_control(self, parent):
        """创建笛卡尔空间控制区域"""
        frame = ttk.LabelFrame(parent, text="笛卡尔空间控制")
        frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 使用PanedWindow实现可调整的分割
        paned_window = ttk.PanedWindow(frame, orient=tk.HORIZONTAL)
        paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧输入框架
        left_input_frame = ttk.Frame(paned_window)
        paned_window.add(left_input_frame, weight=1)

        # 右侧矩阵显示框架
        right_matrix_frame = ttk.Frame(paned_window)
        paned_window.add(right_matrix_frame, weight=1)

        # 位置输入
        ttk.Label(left_input_frame, text="X坐标:").grid(row=0, column=0, padx=5, pady=2, sticky=tk.W)
        self.x_var = tk.DoubleVar(value=0.0)
        ttk.Entry(left_input_frame, textvariable=self.x_var, width=10).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(left_input_frame, text="m").grid(row=0, column=2, padx=5, pady=2)

        ttk.Label(left_input_frame, text="Y坐标:").grid(row=1, column=0, padx=5, pady=2, sticky=tk.W)
        self.y_var = tk.DoubleVar(value=0.0)
        ttk.Entry(left_input_frame, textvariable=self.y_var, width=10).grid(row=1, column=1, padx=5, pady=2)
        ttk.Label(left_input_frame, text="m").grid(row=1, column=2, padx=5, pady=2)

        ttk.Label(left_input_frame, text="Z坐标:").grid(row=2, column=0, padx=5, pady=2, sticky=tk.W)
        self.z_var = tk.DoubleVar(value=0.0)
        ttk.Entry(left_input_frame, textvariable=self.z_var, width=10).grid(row=2, column=1, padx=5, pady=2)
        ttk.Label(left_input_frame, text="m").grid(row=2, column=2, padx=5, pady=2)

        ttk.Label(left_input_frame, text="俯仰角:").grid(row=3, column=0, padx=5, pady=2, sticky=tk.W)
        self.pitch_var = tk.DoubleVar(value=0)
        ttk.Entry(left_input_frame, textvariable=self.pitch_var, width=10).grid(row=3, column=1, padx=5, pady=2)
        ttk.Label(left_input_frame, text="°").grid(row=3, column=2, padx=5, pady=2)

        # 控制按钮
        btn_frame = ttk.Frame(left_input_frame)
        btn_frame.grid(row=4, column=0, columnspan=3, pady=10)

        ttk.Button(btn_frame, text="计算逆解", command=self.calculate_ik).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_frame, text="运动到目标", command=self.move_to_target).pack(side=tk.LEFT, padx=2)

        # 目标变换矩阵显示
        ttk.Label(right_matrix_frame, text="目标变换矩阵:", font=("Arial", 10, "bold")).pack(anchor=tk.W, pady=(0, 5))
        
        # 创建框架包含文本框和滚动条
        matrix_frame = ttk.Frame(right_matrix_frame)
        matrix_frame.pack(fill=tk.BOTH, expand=True)
        
        self.ik_matrix_text = tk.Text(matrix_frame, height=8, width=35, wrap=tk.NONE)
        ik_scrollbar_x = ttk.Scrollbar(matrix_frame, orient=tk.HORIZONTAL, command=self.ik_matrix_text.xview)
        ik_scrollbar_y = ttk.Scrollbar(matrix_frame, orient=tk.VERTICAL, command=self.ik_matrix_text.yview)
        self.ik_matrix_text.configure(xscrollcommand=ik_scrollbar_x.set, yscrollcommand=ik_scrollbar_y.set)
        
        self.ik_matrix_text.grid(row=0, column=0, sticky="nsew")
        ik_scrollbar_y.grid(row=0, column=1, sticky="ns")
        ik_scrollbar_x.grid(row=1, column=0, sticky="ew")
        
        matrix_frame.grid_rowconfigure(0, weight=1)
        matrix_frame.grid_columnconfigure(0, weight=1)

        ttk.Button(right_matrix_frame, text="更新目标矩阵", command=self.update_ik_matrix).pack(pady=5)

        # ==================== 新增部分：变换矩阵输入逆运动学 ====================
        # 在笛卡尔空间控制部分下方添加新的变换矩阵输入区域
        matrix_input_frame = ttk.LabelFrame(frame, text="变换矩阵输入逆运动学")
        matrix_input_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 变换矩阵输入标签和文本框
        ttk.Label(matrix_input_frame, text="输入4x4变换矩阵:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        
        # 创建矩阵输入文本框框架
        matrix_text_frame = ttk.Frame(matrix_input_frame)
        matrix_text_frame.grid(row=1, column=0, columnspan=3, padx=5, pady=5, sticky="nsew")
        
        # 矩阵输入文本框（4行，每行对应矩阵的一行）
        self.matrix_input_text = tk.Text(matrix_text_frame, height=6, width=50)
        matrix_scrollbar = ttk.Scrollbar(matrix_text_frame, orient=tk.VERTICAL, command=self.matrix_input_text.yview)
        self.matrix_input_text.configure(yscrollcommand=matrix_scrollbar.set)
        
        self.matrix_input_text.grid(row=0, column=0, sticky="nsew")
        matrix_scrollbar.grid(row=0, column=1, sticky="ns")
        
        matrix_text_frame.grid_rowconfigure(0, weight=1)
        matrix_text_frame.grid_columnconfigure(0, weight=1)
        
        # 默认填充单位矩阵作为示例
        default_matrix = "1.0, 0.0, 0.0, 0.2\n0.0, 1.0, 0.0, 0.0\n0.0, 0.0, 1.0, 0.1\n0.0, 0.0, 0.0, 1.0"
        self.matrix_input_text.insert(tk.END, default_matrix)
        
        # 按钮框架
        matrix_btn_frame = ttk.Frame(matrix_input_frame)
        matrix_btn_frame.grid(row=2, column=0, columnspan=3, pady=10)
        
        ttk.Button(matrix_btn_frame, text="从矩阵计算逆解", command=self.calculate_ik_from_matrix).pack(side=tk.LEFT, padx=5)
        ttk.Button(matrix_btn_frame, text="运动到矩阵目标", command=self.move_to_matrix_target).pack(side=tk.LEFT, padx=5)
        ttk.Button(matrix_btn_frame, text="清空矩阵", command=self.clear_matrix_input).pack(side=tk.LEFT, padx=5)
        
        # 结果显示标签
        self.matrix_result_label = ttk.Label(matrix_input_frame, text="", foreground="blue")
        self.matrix_result_label.grid(row=3, column=0, columnspan=3, padx=5, pady=5)


    def create_status_section(self, parent):
        """创建状态显示区域 - 右侧较小的面板"""
        # 创建选项卡
        notebook = ttk.Notebook(parent)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 状态信息标签页
        status_frame = ttk.Frame(notebook)
        notebook.add(status_frame, text="状态信息")

        # 当前角度显示
        angle_frame = ttk.LabelFrame(status_frame, text="当前关节角度")
        angle_frame.pack(fill=tk.X, padx=5, pady=5)

        self.angle_labels = []
        joints = ["关节1", "关节2", "关节3", "关节4", "关节5"]
        for i, joint in enumerate(joints):
            row = ttk.Frame(angle_frame)
            row.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(row, text=f"{joint}:").pack(side=tk.LEFT)
            label = ttk.Label(row, text="0°", foreground="blue")
            label.pack(side=tk.RIGHT)
            self.angle_labels.append(label)

        # 当前位置显示
        pos_frame = ttk.LabelFrame(status_frame, text="当前位置坐标")
        pos_frame.pack(fill=tk.X, padx=5, pady=5)

        self.pos_labels = []
        coords = ["X", "Y", "Z"]
        for i, coord in enumerate(coords):
            row = ttk.Frame(pos_frame)
            row.pack(fill=tk.X, padx=5, pady=2)
            ttk.Label(row, text=f"{coord}:").pack(side=tk.LEFT)
            label = ttk.Label(row, text="0.000 m", foreground="blue")
            label.pack(side=tk.RIGHT)
            self.pos_labels.append(label)

        # 串口监控标签页
        monitor_frame = ttk.Frame(notebook)
        notebook.add(monitor_frame, text="串口监控")

        # 监控文本框框架
        monitor_text_frame = ttk.Frame(monitor_frame)
        monitor_text_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.monitor_text = tk.Text(monitor_text_frame, height=12)
        scrollbar_y = ttk.Scrollbar(monitor_text_frame, orient=tk.VERTICAL, command=self.monitor_text.yview)
        scrollbar_x = ttk.Scrollbar(monitor_text_frame, orient=tk.HORIZONTAL, command=self.monitor_text.xview)
        self.monitor_text.configure(yscrollcommand=scrollbar_y.set, xscrollcommand=scrollbar_x.set)
        
        self.monitor_text.grid(row=0, column=0, sticky="nsew")
        scrollbar_y.grid(row=0, column=1, sticky="ns")
        scrollbar_x.grid(row=1, column=0, sticky="ew")
        
        monitor_text_frame.grid_rowconfigure(0, weight=1)
        monitor_text_frame.grid_columnconfigure(0, weight=1)

        # 清空监控按钮
        ttk.Button(monitor_frame, text="清空监控", command=self.clear_monitor).pack(side=tk.BOTTOM, pady=5)

    def refresh_ports(self):
        """刷新可用串口列表"""
        try:
            ports = [port.device for port in serial.tools.list_ports.comports()]
            self.port_combo['values'] = ports
            if ports:
                self.port_combo.set(ports[0])
        except Exception as e:
            print(f"刷新端口列表时出错: {e}")

    def toggle_connection(self):
        """切换串口连接状态"""
        if not self.is_connected:
            self.connect_serial()
        else:
            self.disconnect_serial()

    def connect_serial(self):
        """连接串口"""
        try:
            port = self.port_combo.get()
            baud = int(self.baud_combo.get())

            self.ser = serial.Serial(port, baud, timeout=1)
            self.is_connected = True
            self.connect_btn.config(text="断开")
            self.log_message(f"已连接到 {port}，波特率 {baud}")

        except Exception as e:
            messagebox.showerror("连接错误", f"无法连接串口: {str(e)}")

    def disconnect_serial(self):
        """断开串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.is_connected = False
        self.connect_btn.config(text="连接")
        self.log_message("串口已断开")

    def log_message(self, message):
        """在监控区域添加日志"""
        try:
            timestamp = time.strftime("%H:%M:%S")
            self.monitor_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.monitor_text.see(tk.END)
        except Exception as e:
            print(f"日志记录错误: {e}")

    def clear_monitor(self):
        """清空监控区域"""
        try:
            self.monitor_text.delete(1.0, tk.END)
        except Exception as e:
            print(f"清空监控错误: {e}")

    def read_angles(self):
        """读取当前角度（模拟）"""
        try:
            simulated_angles = [0, 90, 0, 90, 0]
            for i, angle in enumerate(simulated_angles):
                self.joint_vars[i].set(angle)

            self.update_status_display()
            self.log_message("已读取当前关节角度")
        except Exception as e:
            print(f"读取角度错误: {e}")

    def set_angles(self):
        """设置关节角度"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return

        try:
            angles = [var.get() for var in self.joint_vars]

            command = self.build_servo_command(angles)
            if command and self.ser:
                self.ser.write(command.encode())
                self.log_message(f"发送关节角度命令: {angles}")

                self.current_angles = angles
                self.update_status_display()
                self.update_fk_matrix()

        except Exception as e:
            messagebox.showerror("错误", f"设置角度失败: {str(e)}")

    def home_position(self):
        """回零位置"""
        try:
            home_angles = [0, 90, 0, 90, 0]
            for i, angle in enumerate(home_angles):
                self.joint_vars[i].set(angle)
            self.set_angles()
        except Exception as e:
            print(f"回零错误: {e}")

    def calculate_ik(self):
        """计算逆运动学"""
        try:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            pitch = self.pitch_var.get()

            angles = solve_ik(x, y, z, pitch)

            for i, angle in enumerate(angles):
                self.joint_vars[i].set(round(angle, 2))

            self.log_message(f"逆解计算完成: 位置({x},{y},{z}), 俯仰角{pitch}° -> 角度{angles}")
            return angles

        except Exception as e:
            messagebox.showerror("计算错误", f"逆运动学计算失败: {str(e)}")
            return None

    def move_to_target(self):
        """运动到目标位置"""
        angles = self.calculate_ik()
        if angles is not None:
            for i, angle in enumerate(angles):
                self.joint_vars[i].set(round(angle, 2))
            self.set_angles()

    def update_fk_matrix(self):
        """更新正运动学变换矩阵显示"""
        try:
            angles = [var.get() for var in self.joint_vars]
            position, T_matrix = FK(*angles)
            matrix_text = format_matrix(T_matrix)
            
            self.fk_matrix_text.delete(1.0, tk.END)
            self.fk_matrix_text.insert(1.0, matrix_text)
            
        except Exception as e:
            # 静默处理错误，避免启动时弹出错误消息
            print(f"更新正运动学矩阵时出错: {e}")

    def update_ik_matrix(self):
        """更新目标变换矩阵显示"""
        try:
            x = self.x_var.get()
            y = self.y_var.get()
            z = self.z_var.get()
            pitch = self.pitch_var.get()
            
            T_target = calculate_target_matrix(x, y, z, pitch)
            matrix_text = format_matrix(T_target)
            
            self.ik_matrix_text.delete(1.0, tk.END)
            self.ik_matrix_text.insert(1.0, matrix_text)
            
        except Exception as e:
            # 静默处理错误，避免启动时弹出错误消息
            print(f"更新目标矩阵时出错: {e}")

    def build_servo_command(self, angles, move_time=1000):
        """构建舵机控制命令"""
        try:
            command = "{"
            for i, angle in enumerate(angles):
                pwm = angle2pwm(i, angle)
                command += f"#{i:03d}P{pwm:04d}T{move_time:04d}!"
            command += "}"
            return command
        except Exception as e:
            self.log_message(f"构建命令失败: {str(e)}")
            return None

    def update_status_display(self):
        """更新状态显示"""
        try:
            for i, label in enumerate(self.angle_labels):
                label.config(text=f"{self.current_angles[i]:.1f}°")

            position, _ = FK(*self.current_angles)
            for i, label in enumerate(self.pos_labels):
                label.config(text=f"{position[i]:.3f} m")
                
            # 更新轨迹规划页面的当前位置显示
            if hasattr(self, 'current_pos_label'):
                self.current_pos_label.config(
                    text=f"X:{position[0]:.3f}, Y:{position[1]:.3f}, Z:{position[2]:.3f}, Pitch:{self.current_angles[4]:.1f}°"
                )
        except Exception as e:
            print(f"更新状态显示错误: {e}")

    # ==================== 新增方法：变换矩阵输入相关功能 ====================

    def parse_matrix_input(self, matrix_text):
        """解析用户输入的4x4变换矩阵"""
        try:
            # 按行分割输入文本
            lines = matrix_text.strip().split('\n')
            if len(lines) != 4:
                raise ValueError("请输入4行矩阵数据")
            
            matrix = np.zeros((4, 4))
            for i, line in enumerate(lines):
                # 预处理：去除行首尾的空白字符和中括号[6](@ref)
                line = line.strip()
                if line.startswith('[') and line.endswith(']'):
                    line = line[1:-1].strip()  # 去除中括号并再次去除空白
                
                # 移除空格和制表符，然后按逗号或空格分割
                elements = line.replace('\t', ' ').replace(' ', ',').split(',')
                # 过滤空字符串
                elements = [elem for elem in elements if elem.strip() != '']
                
                if len(elements) != 4:
                    raise ValueError(f"第{i+1}行必须有4个元素")
                
                for j, elem in enumerate(elements):
                    matrix[i, j] = float(elem.strip())
            
            return matrix
        except Exception as e:
            raise ValueError(f"矩阵格式错误: {str(e)}")

    def calculate_ik_from_matrix(self):
        """从变换矩阵计算逆运动学"""
        try:
            # 获取用户输入的矩阵文本
            matrix_text = self.matrix_input_text.get(1.0, tk.END).strip()
            if not matrix_text:
                messagebox.showwarning("输入错误", "请输入4x4变换矩阵")
                return None
            
            # 解析矩阵
            T_target = self.parse_matrix_input(matrix_text)
            
            # 检查矩阵是否为有效的齐次变换矩阵
            if T_target[3, 3] != 1.0 or not np.allclose(T_target[3, 0:3], 0):
                messagebox.showwarning("矩阵错误", "输入的矩阵不是有效的齐次变换矩阵")
                return None
            
            # 使用现有的IK函数计算逆解
            rad_angles = IK(T_target)
            deg_angles = list(np.degrees(rad_angles))
            
            # 更新关节角度显示
            for i, angle in enumerate(deg_angles):
                self.joint_vars[i].set(round(angle, 2))
            
            # 显示计算结果
            result_text = f"逆解计算成功: {[f'{a:.2f}°' for a in deg_angles]}"
            self.matrix_result_label.config(text=result_text, foreground="green")
            self.log_message(f"从变换矩阵计算逆解: {deg_angles}")
            
            return deg_angles
            
        except Exception as e:
            error_msg = f"逆运动学计算失败: {str(e)}"
            self.matrix_result_label.config(text=error_msg, foreground="red")
            messagebox.showerror("计算错误", error_msg)
            return None

    def move_to_matrix_target(self):
        """运动到矩阵目标位置"""
        angles = self.calculate_ik_from_matrix()
        if angles is not None:
            # 设置并发送角度命令
            for i, angle in enumerate(angles):
                self.joint_vars[i].set(round(angle, 2))
            self.set_angles()

    def clear_matrix_input(self):
        """清空矩阵输入框"""
        self.matrix_input_text.delete(1.0, tk.END)
        self.matrix_result_label.config(text="")


    # ==================== 轨迹规划相关方法开始 ====================
    def plan_trajectory(self):
        """规划轨迹"""
        try:
            # 获取起始位置（当前机械臂位置）
            start_position, _ = FK(*self.current_angles)
            start_pitch = self.current_angles[4]  # 使用关节5的角度作为俯仰角近似
            start_pose = [start_position[0], start_position[1], start_position[2], start_pitch]
            
            # 获取目标位置
            target_pose = [
                self.target_x_var.get(),
                self.target_y_var.get(),
                self.target_z_var.get(),
                self.target_pitch_var.get()
            ]
            
            # 获取轨迹参数
            num_points = self.point_count_var.get()
            point_interval = self.point_interval_var.get()
            # 计算总时间：点间隔 * (点数量 + 1) 因为包括起点和终点，有（点数量+1）个间隔
            total_time = point_interval * (num_points + 1)
            
            # 规划笛卡尔空间轨迹
            cartesian_trajectory = self.trajectory_planner.plan_cartesian_trajectory(
                start_pose, target_pose, num_points, total_time
            )
            
            if cartesian_trajectory is None:
                messagebox.showerror("错误", "轨迹规划失败")
                return
                
            # 计算关节空间轨迹
            joint_trajectory = self.trajectory_planner.calculate_joint_trajectory(
                cartesian_trajectory, self.current_angles
            )
            
            if joint_trajectory is None:
                messagebox.showerror("错误", "关节轨迹计算失败，可能存在不可达点或奇异点")
                return
                
            # 显示轨迹信息
            self.display_trajectory_info(joint_trajectory)
            
            # 可视化轨迹
            self.visualize_trajectory(joint_trajectory)
            
            # 显示目标变换矩阵
            self.display_target_matrix(target_pose)
            
            self.trajectory_status_var.set(f"轨迹规划完成，共{len(joint_trajectory)}个点")
            self.log_message(f"轨迹规划完成，从{start_pose}到{target_pose}，共{len(joint_trajectory)}个点")
            
        except Exception as e:
            messagebox.showerror("错误", f"轨迹规划失败: {str(e)}")
            self.log_message(f"轨迹规划错误: {str(e)}")

    def display_trajectory_info(self, joint_trajectory):
        """显示轨迹信息"""
        # 清空现有数据
        for item in self.points_tree.get_children():
            self.points_tree.delete(item)
            
        # 添加新数据
        for i, point in enumerate(joint_trajectory):
            cartesian = point['cartesian_point']
            joints = point['joint_angles']
            
            self.points_tree.insert("", "end", values=(
                i+1,
                f"{cartesian[0]:.3f}",
                f"{cartesian[1]:.3f}",
                f"{cartesian[2]:.3f}",
                f"{cartesian[3]:.1f}°",
                f"{joints[0]:.1f}°",
                f"{joints[1]:.1f}°",
                f"{joints[2]:.1f}°",
                f"{joints[3]:.1f}°",
                f"{joints[4]:.1f}°"
            ))

    def visualize_trajectory(self, joint_trajectory):
        """可视化轨迹"""
        self.fig.clear()
        
        # 提取轨迹数据
        x_values = [point['cartesian_point'][0] for point in joint_trajectory]
        y_values = [point['cartesian_point'][1] for point in joint_trajectory]
        z_values = [point['cartesian_point'][2] for point in joint_trajectory]
        
        # 创建3D轨迹图
        ax1 = self.fig.add_subplot(221, projection='3d')
        ax1.plot(x_values, y_values, z_values, 'b-o', linewidth=2, markersize=4)
        ax1.scatter(x_values[0], y_values[0], z_values[0], color='g', s=100, label='Start')
        ax1.scatter(x_values[-1], y_values[-1], z_values[-1], color='r', s=100, label='End')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('3D Trajectory')
        ax1.legend()
        ax1.grid(True)
        
        # 创建关节角度变化图
        time_points = range(len(joint_trajectory))
        joint_data = [[] for _ in range(5)]
        
        for point in joint_trajectory:
            for i in range(5):
                joint_data[i].append(point['joint_angles'][i])
        
        ax2 = self.fig.add_subplot(222)
        colors = ['r', 'g', 'b', 'c', 'm']
        labels = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5']
        for i in range(5):
            ax2.plot(time_points, joint_data[i], color=colors[i], label=labels[i], marker='o')
        ax2.set_xlabel('Path Point')
        ax2.set_ylabel('Joint Angle (°)')
        ax2.set_title('Joint Angles')
        ax2.legend()
        ax2.grid(True)
        
        # 创建位置分量图
        ax3 = self.fig.add_subplot(223)
        ax3.plot(time_points, x_values, 'r-', label='X', marker='o')
        ax3.plot(time_points, y_values, 'g-', label='Y', marker='o')
        ax3.plot(time_points, z_values, 'b-', label='Z', marker='o')
        ax3.set_xlabel('Path Point')
        ax3.set_ylabel('Position (m)')
        ax3.set_title('Position Components')
        ax3.legend()
        ax3.grid(True)
        
        # 创建俯仰角变化图
        pitch_values = [point['cartesian_point'][3] for point in joint_trajectory]
        ax4 = self.fig.add_subplot(224)
        ax4.plot(time_points, pitch_values, 'm-', marker='o')
        ax4.set_xlabel('Path Point')
        ax4.set_ylabel('Pitch (°)')
        ax4.set_title('Pitch Angle')
        ax4.grid(True)
        
        self.fig.tight_layout()
        self.canvas.draw()

    def display_target_matrix(self, target_pose):
        """显示目标变换矩阵"""
        try:
            x, y, z, pitch = target_pose
            T_target = calculate_target_matrix(x, y, z, pitch)
            matrix_text = format_matrix(T_target)
            
            self.target_matrix_text.delete(1.0, tk.END)
            self.target_matrix_text.insert(1.0, matrix_text)
        except Exception as e:
            print(f"显示目标矩阵错误: {e}")

    def execute_trajectory(self):
        """执行轨迹"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        if not hasattr(self.trajectory_planner, 'joint_trajectory') or not self.trajectory_planner.joint_trajectory:
            messagebox.showwarning("警告", "请先规划轨迹")
            return
            
        self.is_running_trajectory = True
        self.trajectory_status_var.set("正在执行轨迹...")
        
        # 在新线程中执行轨迹
        self.trajectory_thread = threading.Thread(target=self._execute_trajectory_thread)
        self.trajectory_thread.daemon = True
        self.trajectory_thread.start()

    def _execute_trajectory_thread(self):
        """执行轨迹的线程函数"""
        try:
            joint_trajectory = self.trajectory_planner.joint_trajectory
            point_interval = self.point_interval_var.get()
            
            for i, point in enumerate(joint_trajectory):
                if not self.is_running_trajectory:
                    break
                    
                # 更新关节角度
                joints = point['joint_angles']
                for j, angle in enumerate(joints):
                    self.joint_vars[j].set(angle)
                
                # 发送到机械臂
                self.set_angles()
                
                # 更新状态
                self.trajectory_status_var.set(f"执行中: {i+1}/{len(joint_trajectory)}")
                self.log_message(f"执行轨迹点 {i+1}/{len(joint_trajectory)}")
                
                # 等待间隔
                time.sleep(point_interval)
            
            if self.is_running_trajectory:
                self.trajectory_status_var.set("轨迹执行完成")
                self.log_message("轨迹执行完成")
            else:
                self.trajectory_status_var.set("轨迹执行已停止")
                self.log_message("轨迹执行被用户停止")
                
        except Exception as e:
            self.trajectory_status_var.set("轨迹执行错误")
            self.log_message(f"轨迹执行错误: {str(e)}")
        
        self.is_running_trajectory = False

    def stop_trajectory(self):
        """停止轨迹执行"""
        self.is_running_trajectory = False
        self.trajectory_status_var.set("轨迹已停止")
        self.log_message("轨迹执行已停止")

    def clear_trajectory(self):
        """清除轨迹"""
        # 清空轨迹数据
        if hasattr(self.trajectory_planner, 'joint_trajectory'):
            self.trajectory_planner.joint_trajectory = []
            
        # 清空显示
        for item in self.points_tree.get_children():
            self.points_tree.delete(item)
            
        self.target_matrix_text.delete(1.0, tk.END)
        self.fig.clear()
        self.canvas.draw()
        
        self.trajectory_status_var.set("已清除轨迹")
        self.log_message("轨迹已清除")

    # ==================== 轨迹规划相关方法结束 ====================
    def create_circle_drawing_interface(self, parent):
        """创建画圆功能界面 - 第三个标签页"""

    # 主框架 - 直接使用parent作为容器
        main_circle_frame = ttk.Frame(parent)  # parent就是已经添加到notebook的circle_frame
        main_circle_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
    
    # 剩下的代码保持不变...
    # 左侧控制面板
        control_frame = ttk.LabelFrame(main_circle_frame, text="画圆参数设置")
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
    
    # 右侧显示面板
        display_frame = ttk.Frame(main_circle_frame)
        display_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
    
    # 初始化画圆规划器
        self.circle_planner = CirclePlanner()
    
    # 创建控制面板
        self.create_circle_control_panel(control_frame)
    
    # 创建显示面板
        self.create_circle_display_panel(display_frame)
    
    def create_circle_control_panel(self, parent):
        """创建画圆控制面板"""
        # 平面定义框架
        plane_frame = ttk.LabelFrame(parent, text="平面定义")
        plane_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 三个点定义平面
        points = ["点1 (P1)", "点2 (P2)", "点3 (P3)"]
        self.plane_points_vars = []
        
        for i, point_name in enumerate(points):
            point_frame = ttk.Frame(plane_frame)
            point_frame.pack(fill=tk.X, padx=5, pady=2)
            
            ttk.Label(point_frame, text=f"{point_name}:").pack(side=tk.LEFT)
            
            # XYZ输入 - 删除默认值
            for coord in ["X", "Y", "Z"]:
                var = tk.DoubleVar()  # 不设置默认值
                self.plane_points_vars.append(var)
                ttk.Label(point_frame, text=coord).pack(side=tk.LEFT, padx=(10, 0))
                ttk.Entry(point_frame, textvariable=var, width=6).pack(side=tk.LEFT, padx=2)
        
        ttk.Button(plane_frame, text="定义平面", 
                  command=self.define_plane).pack(pady=5)
        
        # 圆参数框架
        circle_param_frame = ttk.LabelFrame(parent, text="圆参数")
        circle_param_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # 圆心输入 - 删除默认值
        ttk.Label(circle_param_frame, text="圆心:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.circle_center_vars = []
        for i, coord in enumerate(["X", "Y", "Z"]):
            ttk.Label(circle_param_frame, text=coord).grid(row=0, column=1+i*2, pady=2)
            var = tk.DoubleVar()  # 不设置默认值
            self.circle_center_vars.append(var)
            ttk.Entry(circle_param_frame, textvariable=var, width=6).grid(row=0, column=2+i*2, padx=2, pady=2)
        
        # 半径和点数 - 删除默认值
        ttk.Label(circle_param_frame, text="半径 (m):").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.radius_var = tk.DoubleVar()  # 不设置默认值
        ttk.Entry(circle_param_frame, textvariable=self.radius_var, width=8).grid(row=1, column=1, columnspan=2, pady=2)
        
        ttk.Label(circle_param_frame, text="点数:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.circle_points_var = tk.IntVar()  # 不设置默认值
        point_combo = ttk.Combobox(circle_param_frame, textvariable=self.circle_points_var,
                                 values=[4, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180, 360], width=8)
        point_combo.grid(row=2, column=1, columnspan=2, pady=2)
        
        ttk.Label(circle_param_frame, text="俯仰角:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.circle_pitch_var = tk.DoubleVar()  # 不设置默认值
        ttk.Entry(circle_param_frame, textvariable=self.circle_pitch_var, width=8).grid(row=3, column=1, columnspan=2, pady=2)
        ttk.Label(circle_param_frame, text="°").grid(row=3, column=3, pady=2)
        
        # 控制按钮框架
        button_frame = ttk.LabelFrame(parent, text="画圆控制")
        button_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Button(button_frame, text="规划圆形轨迹", 
                  command=self.plan_circle_trajectory).pack(pady=2)
        ttk.Button(button_frame, text="执行画圆", 
                  command=self.execute_circle).pack(pady=2)
        ttk.Button(button_frame, text="停止画圆", 
                  command=self.stop_circle).pack(pady=2)
        ttk.Button(button_frame, text="清除轨迹", 
                  command=self.clear_circle_trajectory).pack(pady=2)
        
        # 状态显示
        self.circle_status_var = tk.StringVar(value="就绪")
        status_label = ttk.Label(button_frame, textvariable=self.circle_status_var, 
                                foreground="blue")
        status_label.pack(pady=5)
    
    def create_circle_display_panel(self, parent):
        """创建画圆显示面板"""
        # 使用垂直分割面板
        paned_window = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        paned_window.pack(fill=tk.BOTH, expand=True)
        
        # 上部：3D可视化
        viz_frame = ttk.LabelFrame(paned_window, text="圆形轨迹可视化")
        paned_window.add(viz_frame, weight=2)
        
        # 创建matplotlib图形
        self.circle_fig = Figure(figsize=(8, 6), dpi=100)
        self.circle_canvas = FigureCanvasTkAgg(self.circle_fig, viz_frame)
        self.circle_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 下部：轨迹点信息
        info_frame = ttk.LabelFrame(paned_window, text="轨迹点信息")
        paned_window.add(info_frame, weight=1)
        
        # 创建树形视图显示轨迹点
        columns = ("点号", "X", "Y", "Z", "关节1", "关节2", "关节3", "关节4", "关节5")
        self.circle_tree = ttk.Treeview(info_frame, columns=columns, show="tree headings", height=6)
        
        for col in columns:
            self.circle_tree.heading(col, text=col)
            self.circle_tree.column(col, width=80)
        
        # 添加滚动条
        scrollbar = ttk.Scrollbar(info_frame, orient=tk.VERTICAL, command=self.circle_tree.yview)
        self.circle_tree.configure(yscrollcommand=scrollbar.set)
        
        self.circle_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    
    def define_plane(self):
        """定义平面"""
        try:
            points = []
            for i in range(3):
                x = self.plane_points_vars[i*3].get()
                y = self.plane_points_vars[i*3+1].get()
                z = self.plane_points_vars[i*3+2].get()
                points.append([x, y, z])
            
            normal, origin = self.circle_planner.define_plane_from_three_points(
                points[0], points[1], points[2]
            )
            
            self.circle_status_var.set(f"平面定义成功 - 法向量: [{normal[0]:.3f}, {normal[1]:.3f}, {normal[2]:.3f}]")
            
        except Exception as e:
            messagebox.showerror("错误", f"平面定义失败: {str(e)}")
    
    def plan_circle_trajectory(self):
        """规划圆形轨迹"""
        try:
            center = [var.get() for var in self.circle_center_vars]
            radius = self.radius_var.get()
            num_points = self.circle_points_var.get()
            pitch = self.circle_pitch_var.get()
            
            # 检查点数是否为360的约数
            if 360 % num_points != 0:
                messagebox.showwarning("警告", f"点数{num_points}不是360的约数，可能导致圆不闭合")
            
            # 计算圆形轨迹
            joint_trajectory = self.circle_planner.calculate_circle_trajectory(
                center, radius, num_points, pitch, self.current_angles
            )
            
            if joint_trajectory is None:
                messagebox.showerror("错误", "圆形轨迹规划失败")
                return
            
            # 显示轨迹信息
            self.display_circle_trajectory(joint_trajectory)
            
            # 可视化圆形轨迹
            self.visualize_circle_trajectory(joint_trajectory)
            
            self.circle_status_var.set(f"圆形轨迹规划完成，共{len(joint_trajectory)}个点")
            
        except Exception as e:
            messagebox.showerror("错误", f"轨迹规划失败: {str(e)}")
    
    def display_circle_trajectory(self, joint_trajectory):
        """显示圆形轨迹信息"""
        # 清空现有数据
        for item in self.circle_tree.get_children():
            self.circle_tree.delete(item)
            
        # 添加新数据
        for i, point in enumerate(joint_trajectory):
            cartesian = point['cartesian_point']
            joints = point['joint_angles']
            
            self.circle_tree.insert("", "end", values=(
                i+1,
                f"{cartesian[0]:.3f}",
                f"{cartesian[1]:.3f}",
                f"{cartesian[2]:.3f}",
                f"{joints[0]:.1f}°",
                f"{joints[1]:.1f}°",
                f"{joints[2]:.1f}°",
                f"{joints[3]:.1f}°",
                f"{joints[4]:.1f}°"
            ))
    
    def visualize_circle_trajectory(self, joint_trajectory):
        """可视化圆形轨迹"""
        self.circle_fig.clear()
        
        # 提取轨迹数据
        positions = [point['position'] for point in joint_trajectory]
        x_values = [p[0] for p in positions]
        y_values = [p[1] for p in positions]
        z_values = [p[2] for p in positions]
        
        # 创建3D轨迹图
        ax = self.circle_fig.add_subplot(111, projection='3d')
        
        # 绘制圆形轨迹
        ax.plot(x_values, y_values, z_values, 'b-o', linewidth=2, markersize=4, label='Planned Circle')
        ax.scatter(x_values[0], y_values[0], z_values[0], color='g', s=100, label='Start/End')
        
        # 设置标签
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('3D Circular Trajectory')
        ax.legend()
        ax.grid(True)
        
        self.circle_canvas.draw()
    
    def execute_circle(self):
        """执行画圆运动"""
        if not hasattr(self.circle_planner, 'joint_trajectory') or not self.circle_planner.joint_trajectory:
            messagebox.showwarning("警告", "请先规划圆形轨迹")
            return
            
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        self.is_running_circle = True
        self.circle_status_var.set("正在执行画圆...")
        
        # 在新线程中执行画圆
        self.circle_thread = threading.Thread(target=self._execute_circle_thread)
        self.circle_thread.daemon = True
        self.circle_thread.start()
    
    def _execute_circle_thread(self):
        """执行画圆的线程函数"""
        try:
            joint_trajectory = self.circle_planner.joint_trajectory
            
            for i, point in enumerate(joint_trajectory):
                if not hasattr(self, 'is_running_circle') or not self.is_running_circle:
                    break
                    
                # 更新关节角度
                joints = point['joint_angles']
                for j, angle in enumerate(joints):
                    self.joint_vars[j].set(angle)
                
                # 发送到机械臂
                self.set_angles()
                
                # 更新状态
                self.circle_status_var.set(f"画圆中: {i+1}/{len(joint_trajectory)}")
                
                # 等待间隔
                time.sleep(0.5)  # 固定时间间隔
            
            if hasattr(self, 'is_running_circle') and self.is_running_circle:
                self.circle_status_var.set("画圆完成")
            else:
                self.circle_status_var.set("画圆已停止")
                
        except Exception as e:
            self.circle_status_var.set("画圆执行错误")
            print(f"画圆错误: {str(e)}")
        
        if hasattr(self, 'is_running_circle'):
            self.is_running_circle = False
    
    def stop_circle(self):
        """停止画圆"""
        if hasattr(self, 'is_running_circle'):
            self.is_running_circle = False
        self.circle_status_var.set("画圆已停止")
    
    def clear_circle_trajectory(self):
        """清除圆形轨迹"""
        if hasattr(self, 'circle_planner'):
            self.circle_planner.circle_points = []
            self.circle_planner.joint_trajectory = []
            
        # 清空显示
        for item in self.circle_tree.get_children():
            self.circle_tree.delete(item)
            
        self.circle_fig.clear()
        self.circle_canvas.draw()
        
        self.circle_status_var.set("已清除轨迹")

    def load_example_points(self):
        """加载示例自定义点数据"""
        example_points = """0.2, 0.0, 0.15, 90
0.25, 0.05, 0.18, 80
0.3, 0.1, 0.2, 90
0.25, 0.15, 0.18, 100
0.2, 0.1, 0.15, 90"""
        
        self.custom_points_text.delete(1.0, tk.END)
        self.custom_points_text.insert(1.0, example_points)
        self.custom_points_status.config(text="示例数据已加载", foreground="green")
        self.log_message("已加载示例自定义点数据")

    def clear_custom_points(self):
        """清空自定义点"""
        self.custom_points_text.delete(1.0, tk.END)
        self.custom_waypoints = []
        self.custom_points_status.config(text="已清空自定义点", foreground="blue")
        self.log_message("已清空自定义点")

    def load_custom_points(self):
        """加载并解析自定义中间点"""
        try:
            text = self.custom_points_text.get(1.0, tk.END).strip()
            if not text:
                self.custom_points_status.config(text="请输入自定义点数据", foreground="red")
                return
                
            lines = text.split('\n')
            self.custom_waypoints = []
            
            for i, line in enumerate(lines):
                line = line.strip()
                if not line:
                    continue
                    
                parts = line.split(',')
                if len(parts) != 4:
                    self.custom_points_status.config(text=f"第{i+1}行格式错误：需要4个值", foreground="red")
                    return
                    
                try:
                    x = float(parts[0].strip())
                    y = float(parts[1].strip())
                    z = float(parts[2].strip())
                    pitch = float(parts[3].strip())
                    self.custom_waypoints.append([x, y, z, pitch])
                except ValueError as e:
                    self.custom_points_status.config(text=f"第{i+1}行包含无效数字", foreground="red")
                    return
            
            self.custom_points_status.config(text=f"成功加载 {len(self.custom_waypoints)} 个自定义点", foreground="green")
            self.log_message(f"已加载 {len(self.custom_waypoints)} 个自定义点")
            
        except Exception as e:
            self.custom_points_status.config(text=f"加载失败: {str(e)}", foreground="red")
            self.log_message(f"加载自定义点错误: {str(e)}")

    def plan_custom_trajectory(self):
        """规划自定义中间点的轨迹"""
        if not self.custom_waypoints:
            messagebox.showwarning("警告", "请先加载自定义点")
            return
            
        try:
            # 获取起始位置（当前机械臂位置）
            start_position, _ = FK(*self.current_angles)
            start_pitch = self.current_angles[4]
            start_pose = [start_position[0], start_position[1], start_position[2], start_pitch]
            
            # 获取目标位置
            target_pose = [
                self.target_x_var.get(),
                self.target_y_var.get(),
                self.target_z_var.get(),
                self.target_pitch_var.get()
            ]
            
            # 组合所有点：起始点 + 自定义中间点 + 目标点
            all_points = [start_pose] + self.custom_waypoints + [target_pose]
            
            # 获取点间隔时间
            point_interval = self.point_interval_var.get()
            
            # 生成完整的轨迹
            self.custom_trajectory = []
            current_joint_angles = self.current_angles
            
            for i in range(len(all_points) - 1):
                segment_start = all_points[i]
                segment_end = all_points[i + 1]
                
                # 使用原有的轨迹规划方法规划每一段
                segment_points = self.trajectory_planner.plan_cartesian_trajectory(
                    segment_start, segment_end, 0, point_interval * 1  # 时间根据间隔计算
                )
                
                if segment_points is None:
                    messagebox.showerror("错误", f"第{i+1}段轨迹规划失败")
                    return
                
                # 计算关节轨迹
                segment_joint_trajectory = self.trajectory_planner.calculate_joint_trajectory(
                    segment_points, current_joint_angles
                )
                
                if segment_joint_trajectory is None:
                    messagebox.showerror("错误", f"第{i+1}段关节轨迹计算失败")
                    return
                
                # 添加到完整轨迹（避免重复点）
                if i > 0 and self.custom_trajectory:
                    # 移除第一点（与上一段最后一点重复）
                    self.custom_trajectory.extend(segment_joint_trajectory[1:])
                else:
                    self.custom_trajectory.extend(segment_joint_trajectory)
                
                # 更新当前关节角度为段的最后角度
                if segment_joint_trajectory:
                    current_joint_angles = segment_joint_trajectory[-1]['joint_angles']
            
            # 显示轨迹信息
            self.display_trajectory_info(self.custom_trajectory)
            self.visualize_trajectory(self.custom_trajectory)
            self.display_target_matrix(target_pose)
            
            self.trajectory_status_var.set(f"自定义轨迹规划完成，共{len(self.custom_trajectory)}个点")
            self.log_message(f"自定义轨迹规划完成，经过{len(self.custom_waypoints)}个中间点，共{len(self.custom_trajectory)}个轨迹点")
            
        except Exception as e:
            messagebox.showerror("错误", f"自定义轨迹规划失败: {str(e)}")
            self.log_message(f"自定义轨迹规划错误: {str(e)}")

    def execute_custom_trajectory(self):
        """执行自定义轨迹"""
        if not self.is_connected:
            messagebox.showwarning("警告", "请先连接串口")
            return
            
        if not self.custom_trajectory:
            messagebox.showwarning("警告", "请先规划自定义轨迹")
            return
            
        self.is_running_trajectory = True
        self.trajectory_status_var.set("正在执行自定义轨迹...")
        
        # 在新线程中执行轨迹
        self.trajectory_thread = threading.Thread(target=self._execute_custom_trajectory_thread)
        self.trajectory_thread.daemon = True
        self.trajectory_thread.start()

    def _execute_custom_trajectory_thread(self):
        """执行自定义轨迹的线程函数"""
        try:
            point_interval = self.point_interval_var.get()
            
            for i, point in enumerate(self.custom_trajectory):
                if not self.is_running_trajectory:
                    break
                    
                # 更新关节角度
                joints = point['joint_angles']
                for j, angle in enumerate(joints):
                    self.joint_vars[j].set(angle)
                
                # 发送到机械臂
                self.set_angles()
                
                # 更新状态
                self.trajectory_status_var.set(f"执行中: {i+1}/{len(self.custom_trajectory)}")
                self.log_message(f"执行自定义轨迹点 {i+1}/{len(self.custom_trajectory)}")
                
                # 等待间隔
                time.sleep(point_interval)
            
            if self.is_running_trajectory:
                self.trajectory_status_var.set("自定义轨迹执行完成")
                self.log_message("自定义轨迹执行完成")
            else:
                self.trajectory_status_var.set("自定义轨迹执行已停止")
                self.log_message("自定义轨迹执行被用户停止")
                
        except Exception as e:
            self.trajectory_status_var.set("自定义轨迹执行错误")
            self.log_message(f"自定义轨迹执行错误: {str(e)}")
        
        self.is_running_trajectory = False    

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmGUI(root)
    root.mainloop()