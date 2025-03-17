

#CCD用的3和4，
#imudata0是俯仰加速度，1是翻滚加速度，
# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制后执行自己的源文件
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态跳转执行 user_main.py
# C4 LED 会一秒周期闪烁
# 当 D9 引脚电平出现变化时退出测试程序

# 从 machine 库包含所有内容
from machine import *
# 从 seekfree 库包含 MOTOR_CONTROLLER
from seekfree import MOTOR_CONTROLLER
# 包含 display 库
from display import *
# 从 smartcar 库包含 ticker encoder
from smartcar import ticker
from smartcar import encoder
# 从 seekfree 库包含 IMU963RX
from seekfree import IMU963RX
# 从 seekfree 库包含 KEY_HANDLER
from seekfree import KEY_HANDLER
# 从 seekfree 库包含 WIRELESS_UART
from seekfree import WIRELESS_UART
# 从 seekfree 库包含 TSL1401
from seekfree import TSL1401

##自己写的
# 导入 PID 类
# from pid_func import PID
# #导入姿态解算
# from filter_func import MahonyAHRS


# 包含 gc 与 time 类
import gc
import time
import math
# import itertools



#一些定义和初始化----------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------------------

# 核心板上 C4 是 LED
# 学习板上 D9  对应二号拨码开关

# 调用 machine 库的 Pin 类实例化一个引脚对象
# 配置参数为 引脚名称 引脚方向 模式配置 默认电平
# 详细内容参考 固件接口说明
led     = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)
switch2 = Pin('D9' , Pin.IN , pull = Pin.PULL_UP_47K, value = True)

state2  = switch2.value()

##屏幕初始化
# 定义片选引脚
cs = Pin('B29' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(3)
# 清屏 不传入参数就使用当前的 背景色 清屏
# 传入 RGB565 格式参数会直接把传入的颜色设置为背景色 然后清屏
lcd.clear(0x0000)

#串口初始化
wireless = WIRELESS_UART(460800)

# # data_analysis 数据解析接口 适配逐飞助手的无线调参功能
# data_flag = wireless.data_analysis()
data_wave = [0,0,0,0,0,0,0,0]
# for i in range(0,8):
#     # get_data 获取调参通道数据 只有一个参数范围 [0-7]
#     data_wave[i] = wireless.get_data(i)

#CCD初始化
ccd = TSL1401() #曝光时间，数值越大，越适用于光线越暗的地方
ccd.set_resolution(TSL1401.RES_12BIT)
#电机
motor_3 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D4_DIR_D5  , 13000, duty = 0, invert = True)   #左轮   死区425
motor_4 = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_D6_DIR_D7  , 13000, duty = 0, invert = True)   #右轮   死区475
#此时编码器接的
encoder_3 = encoder("C2" , "C3" , True)
encoder_4 = encoder("C0" , "C1" )
#imu初始化
imu = IMU963RX()
#按键初始化
key = KEY_HANDLER(10)


#定义
temp_angle = 0.0
gyro_now = 0.0
error_angle = 0.0

first_angle = 0
last_angle = 0.0

# 中断标志位
ticker_flag = False
ticker_count = 0
ticker_flag1 = False
ticker_count1 = 0
ticker_flag2 = False
ticker_count2 = 0
ticker_flag3 = False
ticker_count3 = 0

#按键标志位
#短按
key_flag01 = 0
key_flag11 = 0
key_flag21 = 0
key_flag31 = 0
#长按
key_flag02 = 0
key_flag12 = 0
key_flag22 = 0
key_flag32 = 0
# 电机
motor_dir = 1
motor_duty = 0
motor_duty_max = 1000





#类1--------------------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------
class PID:
    def __init__(self, kp, ki, kd, imax, target):#初始化
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = abs(imax)  # 绝对值保证积分限幅有效性
        self.integrator = 0.0
        self.last_error = 0.0
        self.out_p = 0.0
        self.out_i = 0.0
        self.out_d = 0.0
        self.out = 0.0
        self.target = target

    def constrain(self, value, min_val, max_val):
        if value > max_val:
            return max_val
        elif value < min_val:
            return min_val
        return value

    def compute(self, error):

        # 累积误差
        self.integrator += error

        # 积分限幅
        if abs(self.integrator) > 10:
            self.integrator = self.constrain(self.integrator, -self.imax, self.imax)

        # 计算各个控制项
        self.out_p = self.kp * error
        self.out_i = self.ki * self.integrator
        self.out_d = self.kd * (error - self.last_error)

        # 更新最后一次误差
        self.last_error = error

        # 总输出
        self.out = self.out_p + self.out_i + self.out_d

        return self.out

#类2--------------------------------------------------------------------------------------------------------------------------------
class SensorData:
    def __init__(self):
        # IMU Data
        self.imu_raw = [0]*6
        self.filtered_angle = 0.0
        self.gyro = [0.0]*3
        
        # Encoder
        self.enc3 = 0
        self.enc4 = 0
        self.last_enc3 = 0
        self.last_enc4 = 0
        
        # CCD
        self.ccd3 = [0]*128
        self.ccd4 = [0]*128
        self.ccd_updated = False
        
        # PID Parameters                                    静止下：(换完963)
        self.speed_pid = PID(2, 0.00, 0.0, 50.0, -200.0)            #(2.5  0.1  0.5)
        self.direction_pid = PID(0, 0, 0.0, 100.0, 0.0)         
        self.angle_pid = PID(6, 0, 1, 0.0, 0.0)             #(2  0.6)pd
        self.gyro_pid = PID(0.5, 0.05, 0, 6000.0, 0.0)         #(1.2  0.2)pd
        
        # 速度环Control Parameters
        self.speed_error = 0
        self.speed_error_last = 0
        
        # 方向环Control Parameters
        self.direction_kp = 22
        self.direction_kp2 = 0.01
        self.direction_kd = 0.1
        self.direction_kd2 = 0.7
        self.direction_error3 = 0
        self.direction_error4 = 0
        self.direction_error3_last = 0
        self.direction_error4_last = 0
        self.direction_out3 = 0.0
        self.direction_out4 = 0.0
        self.direction_out = 0.0
        
sensor_data = SensorData()

#类3--------------------------------------------------------------------------------------------------------------------------------
class State:
    NORMAL = 0
    ENTERING = 1
    IN_LOOP = 2
    EXITING = 3

current_state = State.NORMAL

#类4--------------------------------------------------------------------------------------------------------------------------------

#类5--------------------------------------------------------------------------------------------------------------------------------
class SensorFilter:
    def __init__(self, ratio_gyro=4.2, ratio_acc=2.8, dt=0.005):
        """
        传感器滤波与融合类
        :param ratio_gyro: 陀螺仪比例系数
        :param ratio_acc: 加速度计比例系数 
        :param dt: 采样周期
        """
        self.ratio_gyro = ratio_gyro
        self.ratio_acc = ratio_acc
        self.dt = dt
        self._first_angle = True
        self._last_angle = 0.0

    def iir_lpf(self, raw_value, filtered_value, factor):
        """IIR低通滤波器"""
        return filtered_value + factor * (raw_value - filtered_value)

    def complementary_filter(self, acc_angle, gyro_rate):
        """
        互补滤波角度计算
        :param acc_angle: 加速度计角度（度）
        :param gyro_rate: 陀螺仪角速度（度/秒）
        :return: 融合后的角度（度）
        """
        if self._first_angle:
            if acc_angle is not None:
                self._last_angle = acc_angle
                self._first_angle = False
            return acc_angle or 0.0

        gyro_input = gyro_rate * self.ratio_gyro
        acc_error = (acc_angle - self._last_angle) * self.ratio_acc if acc_angle else 0
        fused_angle = self._last_angle + (acc_error + gyro_input) * self.dt
        self._last_angle = fused_angle
        return fused_angle

    def reset(self):
        """重置滤波器状态"""
        self._first_angle = True
        self._last_angle = 0.0
        
filter_module = SensorFilter(ratio_gyro=4.2, ratio_acc=2.8)
#类6--------------------------------------------------------------------------------------------------------------------------------
# class VisionProcessor:
#     def __init__(self):
#         self.left_sum3 = 0
#         self.last_left_sum3 = 0 
#         self.right_sum3 = 0
#         self.last_right_sum3 = 0
#         self.left_sum4 = 0
#         self.last_left_sum4 = 0
#         self.right_sum4 = 0
#         self.last_right_sum4 = 0
#         self.total_left_sum = 0
#         self.last_total_left_sum = 0    
#         self.total_right_sum = 0
#         self.last_total_right_sum = 0
#         self.left_annual_flag = 0
#         self.right_annual_flag = 0
#         self.left3_edge = 0
#         self.left4_edge = 0
#         self.right3_edge = 127
#         self.right4_edge = 127
#         self.last_valid_error3 = 0
#         self.last_valid_error4 = 0  
#         self.real_track_width3 = 80
#         self.real_track_width4 = 70
#         self.current_track_width3 = 80
#         self.current_left_track_width3 = 40
#         self.current_right_track_width3 = 40
#         self.current_track_width4 = 70
#         self.current_left_track_width4 = 35 
#         self.current_right_track_width4 = 35
#         self.error3 = 0.0
#         self.error4 = 0.0

#     def calculate_error_diff_over_sum3_4(self, ccd_data3, ccd_data4, threshold_ratio=0.92):
#         # 动态阈值计算
#         avg3 = sum(ccd_data3) / len(ccd_data3)
#         avg4 = sum(ccd_data4) / len(ccd_data4)
#         threshold3 = avg3 * threshold_ratio
#         threshold4 = avg4 * threshold_ratio
        
#         # 丢线标志
#         lost_line3 = True
#         lost_line4 = True

#         # 检测是否全白/全黑
#         white_count3 = sum(1 for x in ccd_data3 if x > threshold3)
#         white_count4 = sum(1 for x in ccd_data4 if x > threshold4)
        
#         # 全黑或全白的处理
#         if white_count3 < 10 or white_count3 > 115:  # 几乎全黑或全白
#             error3 = self.last_valid_error3  # 使用上次的有效值
#         else:
#             lost_line3 = False
#             # 寻找左边缘 - CCD3
#             for i in range(64, 0, -1):
#                 if ccd_data3[i] > threshold3 and ccd_data3[i-1] < threshold3:
#                     left3_edge = i
#                     break
                    
#             # 寻找右边缘 - CCD3
#             for i in range(64, 126):
#                 if ccd_data3[i] > threshold3 and ccd_data3[i+1] < threshold3:
#                     right3_edge = i
#                     break
            
#             # 计算中线偏差
#             center3 = (self.left3_edge + self.right3_edge) / 2
#             error3 = 64 - center3 
#             self.last_valid_error3 = error3  # 更新最后有效值

#         if white_count4 < 10 or white_count4 > 115:  # 几乎全黑或全白
#             error4 = self.last_valid_error4  # 使用上次的有效值
#         else:
#             lost_line4 = False
#             # 寻找左边缘 - CCD4
#             for i in range(64, 0, -1):
#                 if ccd_data4[i] > threshold4 and ccd_data4[i-1] < threshold4:
#                     self.left4_edge = i
#                     break
                    
#             # 寻找右边缘 - CCD4
#             for i in range(64, 126):
#                 if ccd_data4[i] > threshold4 and ccd_data4[i+1] < threshold4:
#                     self.right4_edge = i
#                     break
            
#             # 计算中线偏差
#             center4 = (self.left4_edge + self.  right4_edge) / 2
#             error4 = 64 - center4
#             self.last_valid_error4 = error4  # 更新最后有效值
#             self.current_track_width3 = self.right3_edge - self.left3_edge
#             self.current_left_track_width3 = 64 - self.left3_edge
#             self.current_right_track_width3 = self.right3_edge - 64  
#             self.current_track_width4 = self.right4_edge - self.left4_edge
#             self.current_left_track_width4 = 64 - self.left4_edge
#             self.current_right_track_width4 = self.right4_edge - 64    
#         # 特殊情况处理
#         if lost_line3 and lost_line4:
#             # 两个传感器都丢线，可能是十字路口或特殊区域
#             # 使用上一次的有效值，但可以适当增加权重
#             error3 = self.last_valid_error3 * 1.2
#             error4 = self.last_valid_error4 * 1.2
#         elif lost_line3:
#             # 只有CCD3丢线，多依赖CCD4的数据
#             error3 = error4 * 1.1
#         elif lost_line4:
#             # 只有CCD4丢线，多依赖CCD3的数据
#             error4 = error3 * 1.1

# #         self.right_anulusdetect()
# #         self.left_anulusdetect()
# #         self.cross()

#         # 添加简单的滤波
#         error3 = error3 * 0.8 + self.last_valid_error3 * 0.2
#         error4 = error4 * 0.8 + self.last_valid_error4 * 0.2

#         # 归一化处理
#         max_error = 64  # 最大可能偏差
#         error3_normalized = (error3 / max_error) * 100
#         error4_normalized = (error4 / max_error) * 100

#         # 限幅处理
#         error3_normalized = max(min(error3_normalized, 100), -100)
#         error4_normalized = max(min(error4_normalized, 100), -100)

#         return error3_normalized, error4_normalized
class VisionProcessor:
    def __init__(self):
        self.left_sum3 = 0
        self.last_left_sum3 = 0 
        self.right_sum3 = 0
        self.last_right_sum3 = 0
        self.left_sum4 = 0
        self.last_left_sum4 = 0
        self.right_sum4 = 0
        self.last_right_sum4 = 0
        self.total_left_sum = 0
        self.last_total_left_sum = 0    
        self.total_right_sum = 0
        self.last_total_right_sum = 0
        self.left_annual_flag = 0
        self.right_annual_flag = 0
        self.left3_edge = 0
        self.left4_edge = 0
        self.right3_edge = 127
        self.right4_edge = 127
        self.last_valid_error3 = 0
        self.last_valid_error4 = 0  
        self.real_track_width3 = 80
        self.real_track_width4 = 70
        self.current_track_width3 = 80
        self.current_left_track_width3 = 40
        self.current_right_track_width3 = 40
        self.current_track_width4 = 0
        self.current_left_track_width4 = 35 
        self.current_right_track_width4 = 35
        self.error3 = 0.0
        self.error4 = 0.0

    def calculate_error_diff_over_sum3_4(self, ccd_data3, ccd_data4, threshold_ratio=0.92):
        # 动态阈值计算
        avg3 = sum(ccd_data3) / len(ccd_data3)
        avg4 = sum(ccd_data4) / len(ccd_data4)
        threshold3 = avg3 * threshold_ratio
        threshold4 = avg4 * threshold_ratio
        
        # 丢线标志
        lost_line3 = True


        # 检测是否全白/全黑   白1黑0
        white_count3 = sum(1 for x in ccd_data3 if x > threshold3)
        self.current_track_width4 = sum(1 for x in ccd_data4 if x > threshold4)
        
        # 全黑或全白的处理
        if white_count3 < 10 or white_count3 > 115:  # 几乎全黑或全白
            error3 = self.last_valid_error3  # 使用上次的有效值
        else:
            lost_line3 = False
            # 寻找左边缘 - CCD3
            for i in range(64, 0, -1):
                if ccd_data3[i] > threshold3 and ccd_data3[i-1] < threshold3:
                    left3_edge = i
                    break
                    
            # 寻找右边缘 - CCD3
            for i in range(64, 126):
                if ccd_data3[i] > threshold3 and ccd_data3[i+1] < threshold3:
                    right3_edge = i
                    break
            
            # 计算中线偏差
            center3 = (self.left3_edge + self.right3_edge) / 2
            error3 = 64 - center3 
            self.last_valid_error3 = error3  # 更新最后有效值

            # 计算中线偏差
            self.current_track_width3 = self.right3_edge - self.left3_edge
            self.current_left_track_width3 = 64 - self.left3_edge
            self.current_right_track_width3 = self.right3_edge - 64 


        # # 使用生成器表达式统计左侧连续白线长度
        # self.current_track_width4 = sum(1 for _ in itertools.takewhile(
        #     lambda x: x > threshold4, 
        #     ccd_data4[:128]  # 安全切片，确保不超过128个像素
        # ))

        # 计算 CCD4 的有效宽度
        self.current_track_width4 = 0
        for i in range(min(128, len(ccd_data4))):
            if ccd_data4[i] > threshold4:
                self.current_track_width4 += 1
            else:
                break
        # 特殊情况处理
        if lost_line3:
            # 两个传感器都丢线，可能是十字路口或特殊区域
            # 使用上一次的有效值，但可以适当增加权重
            error3 = self.last_valid_error3 * 1.2


        # 添加简单的滤波
        error3 = error3 * 0.8 + self.last_valid_error3 * 0.2
        
        self.right_anulusdetect()
        self.left_anulusdetect()
        self.cross()

        # 归一化处理
        max_error = 64  # 最大可能偏差
        error3_normalized = (error3 / max_error) * 100

        # 限幅处理
        error3_normalized = max(min(error3_normalized, 100), -100)
        
        return error3_normalized

    def cross(self):
        if self.current_track_width4 > 1 and self.current_track_width3 > 1:
            error3 = 0

            
    def left_anulusdetect(self):
        if self.current_track_width4 > 1 and self.current_left_track_width3 > 1 and self.current_right_track_width3 > 1:
            self.left_annual_flag = 1
            error3 = (self.right3_edge - 120) * 1.2  # 偏移到右边线
        if self.current_track_width4 > 1 and self.left_annual_flag == 1 and self.current_right_track_width3 < 1: 
            self.left_annual_flag = 2

        if self.left_annual_flag == 2 and self.current_track_width4 > 1 and self.current_left_track_width3 > 1:
            self.left_annual_flag = 3
            error3 = (self.left3_edge - 38) * 1.8  # 偏移到左边线

        if self.left_annual_flag == 3 and self.current_right_track_width3 < 1:
            self.left_annual_flag = 4

        if self.left_annual_flag == 4 and self.current_track_width4 < 1 and self.current_track_width3 > 1:
            self.left_annual_flag = 5
            error3 = (self.left3_edge - 38) * 1.8  # 偏移到左边线

        if self.left_annual_flag == 5 and self.current_track_width4 > 1 and self.current_left_track_width3 > 1:
            error3 = (self.right3_edge - 120) * 1.2  # 偏移到右边线
            self.left_annual_flag = 0



    def right_anulusdetect(self):
        # 状态1: 检测到右圆环入口
        if self.current_track_width4 > 80 and self.current_left_track_width3 > 40 and self.current_right_track_width3 > 40:
            self.right_annual_flag = 1
            error3 = (self.left3_edge - 8) * 1.2  # 偏移到左边线
            
        # 状态2: 进入圆环前段
        if self.current_track_width4 > 80 and self.right_annual_flag == 1 and self.current_left_track_width3 < 30: 
            self.right_annual_flag = 2
            error3 = (self.left3_edge - 8) * 1.5  # 继续跟随左边线，增大增益
            
        # 状态3: 圆环中段
        if self.right_annual_flag == 2 and self.current_track_width4 > 80 and self.current_right_track_width3 > 40:
            self.right_annual_flag = 3
            error3 = (self.right3_edge - 90) * 1.8  # 切换到右边线跟随
            
        # 状态4: 圆环后段
        if self.right_annual_flag == 3 and self.current_left_track_width3 < 30:
            self.right_annual_flag = 4

            
        # 状态5: 准备退出圆环
        if self.right_annual_flag == 4 and self.current_track_width4 < 60 and self.current_track_width3 > 70:
            self.right_annual_flag = 5
            error3 = (self.right3_edge - 90) * 1.8  # 继续跟随右边线
            
        # 状态6: 完全退出圆环，恢复正常巡线
        if self.right_annual_flag == 5 and self.current_track_width4 > 70 and self.current_right_track_width3 > 35:
            error3 = (self.left3_edge - 8) * 1.2  # 逐渐回归中线
            self.right_annual_flag = 0



vision_module = VisionProcessor()

#类7--------------------------------------------------------------------------------------------------------------------------------        
class MotionController:
    def __init__(self, smooth_factor=0.1):
        """
        运动控制类
        :param smooth_factor: 速度平滑系数（0-1）
        """
        self.smooth_factor = smooth_factor
        self._speed_out_prev = 0
        self._control_counter = 0
        self.start_flag = 0
    def speed_smoothing(self, target_speed, current_speed):
        """
        速度平滑输出
        :param target_speed: 目标速度
        :param current_speed: 当前速度
        :return: 平滑后的速度指令
        """
        # 设置较小的加速度限制
        max_acceleration = 0.5  # 可以根据实际需求调整这个值
        
        # 计算期望变化量
        delta = target_speed - self._speed_out_prev
        self._control_counter += 1
        # 限制速度变化量
        if abs(delta) > max_acceleration:
            if delta > 0:
                smoothed_speed = self._speed_out_prev + max_acceleration
            else:
                smoothed_speed = self._speed_out_prev - max_acceleration
        else:
            smoothed_speed = target_speed
        
        self._speed_out_prev = smoothed_speed
        return smoothed_speed
    

    def balance_control(self, current_angle, target_angle, gyro_rate):
        """
        直立平衡控制
        :return: 电机控制量
        """
        angle_error = target_angle - current_angle
        gyro_compensation = gyro_rate * 0.8  # 角速度补偿系数
        return angle_error * 1.2 + gyro_compensation  # 示例系数####创建实例

motion_module = MotionController(smooth_factor=0.1)
# 定义PID
# 创建一个 PID 实例
#静止


#静止：0.01， 0， 0.8， 0.03， 0.56， 0.07    或者5， 0， 0.6， 0.01， 1， 0.25

#函数----------------------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler0(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    global key_data
    global sensor_data
    # 原子操作更新编码器数据
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 1000) else (1)
    key_data = key.get()
    sensor_data.enc3 = encoder_3.get()
    sensor_data.enc4 = encoder_4.get()
    sensor_data.enc3 = filter_module.iir_lpf(sensor_data.enc3,sensor_data.last_enc3,0.3)
    sensor_data.enc4 = filter_module.iir_lpf(sensor_data.enc4,sensor_data.last_enc3,0.3)
    sensor_data.speed_error = sensor_data.speed_pid.target - (sensor_data.enc3 + sensor_data.enc4) / 2
    sensor_data.speed_pid.out = sensor_data.speed_pid.compute(sensor_data.speed_error * 0.8 + sensor_data.speed_error_last * 0.2)  #一阶低通滤波
    sensor_data.speed_pid.out = sensor_data.speed_pid.constrain(sensor_data.speed_pid.out,-1000,1000)
    sensor_data.speed_error_last = sensor_data.speed_error
    sensor_data.last_enc3 = sensor_data.enc3
    sensor_data.last_enc4 = sensor_data.enc4



    
def time_pit_handler1(time):
    global ticker_flag1  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count1
    global sensor_data
    ticker_flag1 = True  # 否则它会新建一个局部变量
    ticker_count1 = (ticker_count1 + 1) if (ticker_count1 < 100) else (1)
        
    # 原子操作更新CCD数据
    sensor_data.ccd3 = ccd.get(2)
    sensor_data.ccd4 = ccd.get(3)
    sensor_data.ccd_updated = True
    sensor_data.direction_out3 = sensor_data.direction_error3 * sensor_data.direction_kp + abs(sensor_data.direction_error3) * sensor_data.direction_error3 * sensor_data.direction_kp2 + (sensor_data.direction_error3 - sensor_data.direction_error3_last) * sensor_data.direction_kd - sensor_data.imu_raw[5] * sensor_data.direction_kd2
    sensor_data.direction_out = sensor_data.direction_out3 * 1
    sensor_data.direction_error3_last = sensor_data.direction_error3

    
def time_pit_handler2(time):
    global ticker_flag2  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count2
    global sensor_data
    ticker_flag2 = True  # 否则它会新建一个局部变量
    ticker_count2 = (ticker_count2 + 1) if (ticker_count2 < 100) else (1)
    # 原子操作更新IMU数据
 
    sensor_data.filtered_angle = filter_module.complementary_filter(sensor_data.imu_raw[1], sensor_data.imu_raw[3]) - 2150
    sensor_data.angle_pid.out = sensor_data.angle_pid.compute(-sensor_data.filtered_angle + sensor_data.speed_pid.out)
    sensor_data.angle_pid.out = sensor_data.angle_pid.constrain(sensor_data.angle_pid.out,-1000,1000)


def time_pit_handler3(time):
    global ticker_flag3  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count3
    global sensor_data
    ticker_flag3 = True  # 否则它会新建一个局部变量
    ticker_count3 = (ticker_count3 + 1) if (ticker_count3 < 100) else (1)
    sensor_data.imu_raw = imu.get()
    #角速度环
    sensor_data.gyro_pid.out = sensor_data.gyro_pid.compute(-sensor_data.imu_raw[3] + sensor_data.angle_pid.out)
    sensor_data.gyro_pid.out = sensor_data.gyro_pid.constrain(sensor_data.gyro_pid.out,-2000,2000)
#         if (roll < 15 and roll > -35):
    if (sensor_data.gyro_pid.out > 0):
        motor_3.duty(sensor_data.gyro_pid.out - sensor_data.direction_out + 500)
        motor_4.duty(sensor_data.gyro_pid.out + sensor_data.direction_out + 460)
    else:
        motor_3.duty(sensor_data.gyro_pid.out - sensor_data.direction_out - 500)
        motor_4.duty(sensor_data.gyro_pid.out + sensor_data.direction_out - 460)
#         else:
#             motor_3.duty(0)
#             motor_4.duty(0)
    
#----------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------------------------------------------
    
# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit0 = ticker(0)
pit1 = ticker(1)
pit2 = ticker(2)
pit3 = ticker(3)
pit0.capture_list(key, encoder_3, encoder_4)
pit1.capture_list(ccd)
# pit2.capture_list()
pit3.capture_list(imu)
# 关联 Python 回调函数
pit0.callback(time_pit_handler0)
pit1.callback(time_pit_handler1)
pit2.callback(time_pit_handler2)
pit3.callback(time_pit_handler3)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit0.start(20)
pit1.start(10)
pit2.start(5)
pit3.start(1)

while True:
    if (ticker_flag):     
            #按键：
        if key_data[0]:           #短按 
            key_flag01 += 1
            key_flag01 = key_flag01 % 4
            print("key1 = {:>6d}.".format(key_data[0]))
            time.sleep_ms(20)
        if key_data[1]:
            key_flag11 += 1
            print("key2 = {:>6d}.".format(key_data[1]))
            time.sleep_ms(20)
        if key_data[2]:
            print("key3 = {:>6d}.".format(key_data[2]))
            time.sleep_ms(20)
        if key_data[3]:
            motion_module.start_flag = 1
            print("key4 = {:>6d}.".format(key_data[3]))
            time.sleep_ms(20)

    if (ticker_flag and ticker_count % 1 == 0):          #10ms*2
        
        wireless.send_oscilloscope(sensor_data.direction_kp,sensor_data.direction_kp2,sensor_data.direction_kd,sensor_data.direction_kd2,(sensor_data.enc3 + sensor_data.enc4) / 2,motion_module.start_flag)
        data_flag = wireless.data_analysis()
#         sensor_data.direction_kp = wireless.get_data(0)
#         sensor_data.direction_kp2 = wireless.get_data(1)
#         sensor_data.direction_kd = wireless.get_data(2)
#         sensor_data.direction_kd2 = wireless.get_data(3)
        
        
    if (ticker_flag and ticker_count % 5 == 0): #显示  200ms
        if (ticker_flag and ticker_count % 50 == 0):    #2s清屏一次
            lcd.clear(0x0000)
            ticker_flag = False
#         if(key_flag01 == 0):
#             lcd.str24(0,  0, "1.basic", 0xF800)
#             lcd.str24(0,  28, "2.basic", 0xF800)
#             lcd.str24(0,  56, "3.basic", 0xF800)
#             lcd.str24(0,  84, "4.basic", 0xF800)
#             lcd.str24(0,  112, "5.basic", 0xF800)  
#         if(key_flag01 == 1):
        lcd.str12(0,  0,  "CCD3_seed    {:>.1f},{:>.1f}".format(vision_module.left_sum3, vision_module.right_sum3), 0xF800)#(y,x)  (x,228)
        lcd.str12(0,  14, "CCD4_seed    {:>.1f},{:>.1f}".format(vision_module.left_sum4, vision_module.right_sum4), 0xF800)
        lcd.str12(0,  28, "total_left_sum = {:>.1f}, total_right_sum = {:>.1f}".format(vision_module.total_left_sum, vision_module.total_right_sum), 0xF800)
        lcd.str12(0,  42, "enc_l={:>f}, enc_r={:>f}, enc_ave={:>.2f}".format(sensor_data.enc3,sensor_data.enc3,(sensor_data.enc3 + sensor_data.enc3) / 2), 0xF800)
        lcd.str12(0,  56, "speed,kp = {:>.3f}, ki = {:>.3f}, kd = {:>.3f}".format(sensor_data.speed_pid.kp,sensor_data.speed_pid.ki,sensor_data.speed_pid.kd),0xF800)
        lcd.str12(0,  70, "angle,kp = {:>.3f}, ki = {:>.3f}, kd = {:>.3f}".format(sensor_data.angle_pid.kp,sensor_data.angle_pid.ki,sensor_data.angle_pid.kd),0xF800)
        lcd.str12(0,  84, "gyro,kp = {:>.3f}, ki = {:>.3f}, kd = {:>.3f}".format(sensor_data.gyro_pid.kp,sensor_data.gyro_pid.ki,sensor_data.gyro_pid.kd),0xF800)
        lcd.str12(0,  98, "out = {:>.1f}, {:>.1f}, {:>.1f}".format(sensor_data.speed_pid.out,sensor_data.angle_pid.out,sensor_data.gyro_pid.out),0xF800)
        lcd.str12(0,  112, "error3 = {:>.1f},  error4 = {:>.1f}".format(sensor_data.direction_error3,sensor_data.direction_error4),0xF800)
        lcd.str12(0,  126, "{:>.d},{:>.d}".format(vision_module.left_annual_flag,vision_module.right_annual_flag),0xF800)
        lcd.str12(0,  140, "width3 = {:>.1f}, left_width3 = {:>.1f}, right_width3 = {:>.1f}".format(vision_module.current_track_width3,vision_module.current_left_track_width3,vision_module.current_right_track_width3),0xF800)
        lcd.str12(0,  154, "width4 = {:>.d}, left_width4 = {:>.d}, right_width4 = {:>.d}".format(vision_module.current_track_width4,vision_module.current_left_track_width4,vision_module.current_right_track_width4),0xF800)


#                 if(key_flag01 == 2):
#                 if(key_flag01 == 3):
#                 if(key_flag01 == 4):

    # if (ticker_flag1 and ticker_count1 % 1 == 0):  #10ms
    #     wireless.send_ccd_image(WIRELESS_UART.CCD3_4_BUFFER_INDEX)

        
    sensor_data.direction_error3 = vision_module.calculate_error_diff_over_sum3_4(sensor_data.ccd3,sensor_data.ccd4)




# 2. 弯道减速时：
# ```python
# # 检测到弯道，需要减速
# if abs(error) > 50:
#     target_speed = 60
#     smooth_speed = self.speed_smoothing(target_speed, current_speed)
# ```
        
# 3. 直道加速时：
# ```python
# # 检测到直道，可以加速
# if abs(error) < 20:
#     target_speed = 100
#     smooth_speed = self.speed_smoothing(target_speed, current_speed)
# ```


        




            



#--------------------------------------------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------------------------------------------
#紧急处理，不用动
    # 如果拨码开关打开 对应引脚拉低 就退出循环
    # 这么做是为了防止写错代码导致异常 有一个退出的手段
    if switch2.value() != state2:
        print("Test program stop.")
        break
    gc.collect()









 


















