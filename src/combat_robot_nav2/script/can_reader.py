#!/usr/bin/env python3
import sys
import struct
import time
import threading
import queue
import math
import tkinter as tk
from tkinter import ttk

# ==========================================
# ROS 2 라이브러리
# ==========================================
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

# python-can 임포트
try:
    import can
except ImportError as e:
    can = None
    print("python-can not available:", e)

# ==========================================
# 🌟 TinS-17 정밀 설정 (차량 스펙 및 CAN Protocol)
# ==========================================
WHEEL_RADIUS_M = 0.07      # 실제 바퀴(스프로킷) 반지름: 7cm
TRACK_WIDTH_M = 0.90       # 양쪽 궤도 중심 사이의 거리 (회전 부족 시 이 값을 낮추세요)
TRACK_SLIP_FACTOR = 1.2    # 🌟 궤도 차량 회전 슬립 계수 (회전 부족 시 올리세요)

# 1m 주행 시 0.06m(R=0.15기준) 오차 보정치 -> 새 스펙에 맞춘 스케일링
CALIBRATION_SCALING = 35.714  

ENCODER_PPR = 1024         
GEAR_RATIO = 30.0          

TICKS_PER_WHEEL_REV = ENCODER_PPR * GEAR_RATIO
RAD_PER_TICK = ((2.0 * math.pi) / TICKS_PER_WHEEL_REV) * CALIBRATION_SCALING

CMD_CAN_ID = 0x201          # RPDO0 (Master -> Chassis)
FEEDBACK_CAN_ID = 0x181     # TPDO0 (Chassis -> Master)

CAN_SEND_FREQ_HZ = 30       
INTER_MSG_GAP_SEC = 0.0005  

DEAD_ZONE_STEER = 50        
DEAD_ZONE_SPEED = 50        

MAX_STEER = 2000  
MAX_SPEED = 5600  

# ==========================================
# ROS 2 노드 클래스 (/odom 발행, /tf 발행, /cmd_vel 수신)
# ==========================================
class VehicleROSNode(Node):
    def __init__(self, ui_app):
        super().__init__('combat_ctrl_ros_node')
        self.ui_app = ui_app
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) # 🌟 TF Broadcaster 추가
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        v_x = msg.linear.x
        v_yaw = msg.angular.z  # ROS 2 표준: 양수(+)가 좌회전(CCW)
        
        # 1. m/s 를 좌/우 궤도의 목표 선속도(m/s)로 변환
        left_v = v_x - (v_yaw * TRACK_WIDTH_M / 2.0)
        right_v = v_x + (v_yaw * TRACK_WIDTH_M / 2.0)
        
        # 2. 선속도를 틱(모터 엔코더 값)으로 변환
        conv = RAD_PER_TICK * WHEEL_RADIUS_M
        left_line = left_v / conv
        right_line = right_v / conv
        
        # 3. GUI 슬라이더 계산식에 맞게 변환 (좌회전 시 right_line이 더 큼)
        speed = (left_line + right_line) / 2.0
        steer = (right_line - left_line) / 2.0
        
        # 4. GUI 슬라이더 업데이트 (한계값 클램핑)
        self.ui_app.current_speed.set(max(-MAX_SPEED, min(MAX_SPEED, int(speed))))
        self.ui_app.current_steer.set(max(-MAX_STEER, min(MAX_STEER, int(steer))))

    def publish_odom(self, left_line_s, right_line_s):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 정밀 파라미터 기반 선속도 계산
        left_v = left_line_s * RAD_PER_TICK * WHEEL_RADIUS_M
        right_v = right_line_s * RAD_PER_TICK * WHEEL_RADIUS_M

        v_x = (left_v + right_v) / 2.0
        
        # 🌟 궤도 차량 슬립(Slip) 현상 보정 적용
        ideal_v_yaw = (right_v - left_v) / TRACK_WIDTH_M
        v_yaw = ideal_v_yaw * TRACK_SLIP_FACTOR

        delta_x = (v_x * math.cos(self.th)) * dt
        delta_y = (v_x * math.sin(self.th)) * dt
        delta_th = v_yaw * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        q_w = math.cos(self.th / 2.0)
        q_z = math.sin(self.th / 2.0)

        # 1. Odometry 메시지 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.z = q_z
        odom_msg.pose.pose.orientation.w = q_w

        odom_msg.twist.twist.linear.x = float(v_x)
        odom_msg.twist.twist.angular.z = float(v_yaw)

        # 🌟 평면 2D 주행 신뢰도 설정 (회전 공분산을 0.5로 높임)
        covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  999.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  999.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 999.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.5
        ]
        odom_msg.pose.covariance = covariance
        odom_msg.twist.covariance = covariance

        self.odom_pub.publish(odom_msg)

        # 2. TF (Transform) 발행 추가
        # EKF 노드와 충돌을 방지하기 위해 odom -> base_footprint 직접 발행을 주석 처리함
        # t = TransformStamped()
        # t.header.stamp = current_time.to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_footprint'
        # t.transform.translation.x = self.x
        # t.transform.translation.y = self.y
        # t.transform.translation.z = 0.0
        # t.transform.rotation.z = q_z
        # t.transform.rotation.w = q_w
        # self.tf_broadcaster.sendTransform(t)

# ==========================================
# CAN & Thread Workers (안정적인 구조 유지)
# ==========================================
class CanBusWrapper:
    def __init__(self):
        self.bus = None
        if can is not None:
            try:
                self.bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000)
                print("✅ PCAN Bus Initialized (PCAN_USBBUS1, 250kbps)")
            except Exception as e:
                print("❌ CAN init error:", e)

    def send(self, msg):
        if self.bus:
            try: self.bus.send(msg)
            except: pass

    def recv(self, timeout=1.0):
        if self.bus is None:
            time.sleep(timeout)
            return None
        try: return self.bus.recv(timeout=timeout)
        except: return None

class TxWorker(threading.Thread):
    def __init__(self, can_wrapper, tx_queue, stop_event):
        super().__init__(daemon=True)
        self.can, self.tx_queue, self.stop_event = can_wrapper, tx_queue, stop_event

    def run(self):
        while not self.stop_event.is_set():
            try: arb_id, payload = self.tx_queue.get(timeout=0.1)
            except queue.Empty: continue
            self.can.send(can.Message(arbitration_id=arb_id, data=payload, is_extended_id=False))
            time.sleep(INTER_MSG_GAP_SEC)

class RxWorker(threading.Thread):
    def __init__(self, can_wrapper, ui_queue, stop_event):
        super().__init__(daemon=True)
        self.can, self.ui_queue, self.stop_event = can_wrapper, ui_queue, stop_event

    def run(self):
        while not self.stop_event.is_set():
            msg = self.can.recv(timeout=0.5)
            if msg: self.ui_queue.put(("rx", msg))

# ==========================================
# 메인 GUI 앱
# ==========================================
class VehicleControl:
    def __init__(self, root):
        self.root = root
        self.root.title("TinS-17 Control & Precise Odom")
        self.root.geometry("600x380")

        self.current_steer = tk.IntVar(value=0)
        self.current_speed = tk.IntVar(value=0)
        self.control_enabled = tk.BooleanVar(value=False)
        self.laser_enabled = tk.BooleanVar(value=False)

        self.filterd_steer, self.filterd_speed = 0.0, 0.0

        self.can = CanBusWrapper()
        self.tx_queue, self.ui_queue = queue.Queue(), queue.Queue()
        self.stop_event = threading.Event()

        self.tx_worker = TxWorker(self.can, self.tx_queue, self.stop_event)
        self.rx_worker = RxWorker(self.can, self.ui_queue, self.stop_event)

        self.init_ui()

        # 🚀 ROS 2 시스템 가동
        rclpy.init(args=None)
        self.ros_node = VehicleROSNode(self)
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        self.ros_thread.start()

        self.root.bind('<Up>', self.increase_speed)
        self.root.bind('<Down>', self.decrease_speed)
        # 좌우 화살표 방향 반전 반영 (좌측: +, 우측: -)
        self.root.bind('<Left>', self.increase_steer)
        self.root.bind('<Right>', self.decrease_steer)
        self.root.bind('<space>', self.reset_controls)

        self.tx_worker.start()
        self.rx_worker.start()

        self.status_label.config(text="CAN: connected (PCAN 250k)" if self.can.bus else "CAN: interface down.")

        self.process_ui_queue()
        self.push_tx_loop()

    def init_ui(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        ctrl_frame = ttk.LabelFrame(frame, text="System Toggles")
        ctrl_frame.pack(fill=tk.X, pady=10, ipady=5)
        
        ttk.Checkbutton(ctrl_frame, text="✅ Enable Control (Must be ON to move)", variable=self.control_enabled).pack(side=tk.LEFT, padx=10)
        ttk.Checkbutton(ctrl_frame, text="Headlights", variable=self.laser_enabled).pack(side=tk.LEFT, padx=10)

        slider_frame = ttk.LabelFrame(frame, text="Speed & Steering Targets (cmd_vel / Manual)")
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # 슬라이더 방향 (왼쪽이 + 양수가 되도록 from과 to 위치 변경)
        ttk.Label(slider_frame, text="Steering:").grid(row=0, column=0, padx=5, pady=15, sticky="w")
        self.steer_slider = ttk.Scale(slider_frame, from_=MAX_STEER, to=-MAX_STEER, orient=tk.HORIZONTAL, variable=self.current_steer)
        self.steer_slider.grid(row=0, column=1, padx=5, sticky="ew")
        self.steer_val_lbl = ttk.Label(slider_frame, text="0", width=6)
        self.steer_val_lbl.grid(row=0, column=2, padx=5)

        ttk.Label(slider_frame, text="Speed:").grid(row=1, column=0, padx=5, pady=15, sticky="w")
        self.speed_slider = ttk.Scale(slider_frame, from_=MAX_SPEED, to=-MAX_SPEED, orient=tk.HORIZONTAL, variable=self.current_speed)
        self.speed_slider.grid(row=1, column=1, padx=5, sticky="ew")
        self.speed_val_lbl = ttk.Label(slider_frame, text="0", width=6)
        self.speed_val_lbl.grid(row=1, column=2, padx=5)

        slider_frame.columnconfigure(1, weight=1)
        self.status_label = ttk.Label(frame, text="Status: Ready", font=("Arial", 10, "bold"), foreground="blue")
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        self.current_steer.trace_add('write', lambda *args: self.steer_val_lbl.config(text=str(self.current_steer.get())))
        self.current_speed.trace_add('write', lambda *args: self.speed_val_lbl.config(text=str(self.current_speed.get())))

    def increase_speed(self, e=None): self.current_speed.set(min(self.current_speed.get() + 200, MAX_SPEED))
    def decrease_speed(self, e=None): self.current_speed.set(max(self.current_speed.get() - 200, -MAX_SPEED))
    def increase_steer(self, e=None): self.current_steer.set(min(self.current_steer.get() + 100, MAX_STEER))
    def decrease_steer(self, e=None): self.current_steer.set(max(self.current_steer.get() - 100, -MAX_STEER))
    def reset_controls(self, e=None): self.current_speed.set(0); self.current_steer.set(0)

    def process_ui_queue(self):
        while not self.ui_queue.empty():
            msg_type, data = self.ui_queue.get()
            
            if msg_type == "rx":
                if data.arbitration_id == FEEDBACK_CAN_ID and len(data.data) >= 4:
                    left_act, right_act = struct.unpack("<hh", data.data[0:4])
                    
                    # 🚀 수신된 바퀴 속도를 /odom 으로 발행 및 좌표 갱신
                    self.ros_node.publish_odom(left_act, right_act)
                    
                    # UI에 모터 상태와 Odom 좌표 표시 결합
                    odom_info = f"X: {self.ros_node.x:.2f}m | Y: {self.ros_node.y:.2f}m | Yaw: {math.degrees(self.ros_node.th):.1f}°"
                    self.status_label.config(text=f"Motor L: {left_act}, R: {right_act}  ||  {odom_info}")

        self.root.after(33, self.process_ui_queue)

    def apply_deadzone(self, value, dz): return 0 if abs(value) <= dz else value

    def push_tx_loop(self):
        if not self.control_enabled.get():
            self.filterd_speed = 0.0
            self.filterd_steer = 0.0
            left_wheel = 0
            right_wheel = 0
            start_stop = 0x00
        else:
            steer = self.apply_deadzone(self.current_steer.get(), DEAD_ZONE_STEER)
            speed = self.apply_deadzone(self.current_speed.get(), DEAD_ZONE_SPEED)

            # 로우패스 필터로 부드러운 가감속 (기존 코드 강점 유지)
            alpha = 0.3
            self.filterd_steer = (1 - alpha) * self.filterd_steer + alpha * steer
            self.filterd_speed = (1 - alpha) * self.filterd_speed + alpha * speed
            
            # 실제 CAN 명령 전송 부호 (좌회전(+)일 때 좌측 감소, 우측 증가)
            left_wheel = max(-MAX_SPEED, min(MAX_SPEED, int(self.filterd_speed + self.filterd_steer)))
            right_wheel = max(-MAX_SPEED, min(MAX_SPEED, int(self.filterd_speed - self.filterd_steer)))
            start_stop = 0x01

        light_ctrl = 0x03 if self.laser_enabled.get() else 0x02
        payload = struct.pack("<hhBBBB", right_wheel, left_wheel, light_ctrl, 0x00, start_stop, 0x05)

        try: self.tx_queue.put_nowait((CMD_CAN_ID, payload))
        except queue.Full: pass

        self.root.after(round(1000 / CAN_SEND_FREQ_HZ), self.push_tx_loop)

    def on_closing(self):
        print("Shutting down...")
        try: self.tx_queue.put_nowait((CMD_CAN_ID, struct.pack("<hhBBBB", 0, 0, 0x01, 0x00, 0x00, 0x05)))
        except queue.Full: pass
            
        self.stop_event.set()
        self.tx_worker.join(timeout=1.0)
        self.rx_worker.join(timeout=1.0)
        
        rclpy.shutdown()
        self.ros_thread.join(timeout=1.0)

        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = VehicleControl(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()