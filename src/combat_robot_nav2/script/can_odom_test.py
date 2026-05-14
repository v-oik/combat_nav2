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
except ImportError:
    can = None

# ==========================================
# 🌟 TinS-17 정밀 파라미터 (회전 오차 조절 포인트)
# ==========================================
WHEEL_RADIUS_M = 0.07      # 실제 바퀴(스프로킷) 반지름: 7cm
TRACK_WIDTH_M = 0.90       # 궤도 중심간 거리 (회전 부족 시 이 값을 낮추세요)
TRACK_SLIP_FACTOR = 1.2    # 회전 슬립 계수 (회전 부족 시 0.75에서 1.0 이상으로 올리세요)

# 1m 주행 시 0.06m(R=0.15기준) 오차 보정치
CALIBRATION_SCALING = 35.714  

ENCODER_PPR = 1024         
GEAR_RATIO = 30.0          

TICKS_PER_WHEEL_REV = ENCODER_PPR * GEAR_RATIO
RAD_PER_TICK = ((2.0 * math.pi) / TICKS_PER_WHEEL_REV) * CALIBRATION_SCALING

# CAN 설정
CMD_CAN_ID = 0x201
FEEDBACK_CAN_ID = 0x181
MAX_STEER = 2000
MAX_SPEED = 5600

# ==========================================
# ROS 2 노드
# ==========================================
class VehicleROSNode(Node):
    def __init__(self, ui_app):
        super().__init__('tins17_ctrl_node')
        self.ui_app = ui_app
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

    def cmd_vel_callback(self, msg):
        v_x, v_yaw = msg.linear.x, msg.angular.z 
        left_v = v_x - (v_yaw * TRACK_WIDTH_M / 2.0)
        right_v = v_x + (v_yaw * TRACK_WIDTH_M / 2.0)
        conv = RAD_PER_TICK * WHEEL_RADIUS_M
        self.ui_app.current_speed.set(int(((left_v + right_v) / 2.0) / conv))
        self.ui_app.current_steer.set(int(((right_v - left_v) / 2.0) / conv))

    def publish_odom(self, left_line_s, right_line_s):
        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        self.last_time = curr_time

        l_v = left_line_s * RAD_PER_TICK * WHEEL_RADIUS_M
        r_v = right_line_s * RAD_PER_TICK * WHEEL_RADIUS_M
        v_x = (l_v + r_v) / 2.0
        v_yaw = ((r_v - l_v) / TRACK_WIDTH_M) * TRACK_SLIP_FACTOR

        self.x += (v_x * math.cos(self.th)) * dt
        self.y += (v_x * math.sin(self.th)) * dt
        self.th += v_yaw * dt

        q_z, q_w = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = curr_time.to_msg(), 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q_z, q_w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp, t.header.frame_id = curr_time.to_msg(), 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = q_z, q_w
        self.tf_broadcaster.sendTransform(t)

# ==========================================
# 메인 GUI 앱 (슬라이더 포함)
# ==========================================
class VehicleControl:
    def __init__(self, root):
        self.root = root
        self.root.title("TinS-17 Full Control & Odom")
        self.root.geometry("600x450")

        self.current_steer = tk.IntVar(value=0)
        self.current_speed = tk.IntVar(value=0)
        self.control_enabled = tk.BooleanVar(value=False)
        
        self.can_bus = None
        if can:
            try: self.can_bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000)
            except: print("CAN 연결 실패")

        self.ui_queue = queue.Queue()
        self.stop_event = threading.Event()
        
        self.init_ui()
        rclpy.init()
        self.ros_node = VehicleROSNode(self)
        threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True).start()
        threading.Thread(target=self.rx_loop, daemon=True).start()
        
        # 키보드 바인딩
        self.root.bind('<Up>', lambda e: self.current_speed.set(min(self.current_speed.get() + 200, MAX_SPEED)))
        self.root.bind('<Down>', lambda e: self.current_speed.set(max(self.current_speed.get() - 200, -MAX_SPEED)))
        self.root.bind('<Left>', lambda e: self.current_steer.set(min(self.current_steer.get() + 100, MAX_STEER)))
        self.root.bind('<Right>', lambda e: self.current_steer.set(max(self.current_steer.get() - 100, -MAX_STEER)))
        self.root.bind('<space>', lambda e: (self.current_speed.set(0), self.current_steer.set(0)))

        self.process_ui_queue()
        self.tx_loop()

    def init_ui(self):
        f = ttk.Frame(self.root, padding="20"); f.pack(fill=tk.BOTH, expand=True)
        
        ttk.Checkbutton(f, text="✅ Enable CAN Control", variable=self.control_enabled).pack(pady=5)

        # 조향 슬라이더 (복구됨)
        ttk.Label(f, text="Steering (Right - Left)").pack(pady=(10, 0))
        self.steer_scale = ttk.Scale(f, from_=MAX_STEER, to=-MAX_STEER, variable=self.current_steer, orient=tk.HORIZONTAL)
        self.steer_scale.pack(fill=tk.X, padx=20)
        self.steer_lbl = ttk.Label(f, text="0"); self.steer_lbl.pack()

        # 속도 슬라이더 (복구됨)
        ttk.Label(f, text="Speed (Forward - Backward)").pack(pady=(10, 0))
        self.speed_scale = ttk.Scale(f, from_=MAX_SPEED, to=-MAX_SPEED, variable=self.current_speed, orient=tk.VERTICAL)
        self.speed_scale.pack(fill=tk.Y, expand=True, pady=5)
        self.speed_lbl = ttk.Label(f, text="0"); self.speed_lbl.pack()

        self.status_label = ttk.Label(f, text="Odometry: Ready", font=("Arial", 11, "bold"), foreground="blue")
        self.status_label.pack(side=tk.BOTTOM, pady=10)

        # 값 변경 감지 업데이트
        self.current_steer.trace_add('write', lambda *a: self.steer_lbl.config(text=str(self.current_steer.get())))
        self.current_speed.trace_add('write', lambda *a: self.speed_lbl.config(text=str(self.current_speed.get())))

    def rx_loop(self):
        while not self.stop_event.is_set():
            if self.can_bus:
                msg = self.can_bus.recv(timeout=0.1)
                if msg and msg.arbitration_id == FEEDBACK_CAN_ID:
                    l, r = struct.unpack("<hh", msg.data[0:4])
                    self.ui_queue.put((l, r))

    def process_ui_queue(self):
        while not self.ui_queue.empty():
            l, r = self.ui_queue.get()
            self.ros_node.publish_odom(l, r)
            self.status_label.config(text=f"X: {self.ros_node.x:.2f}m | Y: {self.ros_node.y:.2f}m | Yaw: {math.degrees(self.ros_node.th):.1f}°")
        self.root.after(33, self.process_ui_queue)

    def tx_loop(self):
        if self.control_enabled.get() and self.can_bus:
            speed, steer = self.current_speed.get(), self.current_steer.get()
            l, r = speed - steer, speed + steer
            payload = struct.pack("<hhBBBB", r, l, 0x02, 0, 0x01, 0x05)
            self.can_bus.send(can.Message(arbitration_id=CMD_CAN_ID, data=payload, is_extended_id=False))
        self.root.after(33, self.tx_loop)

    def on_closing(self):
        self.stop_event.set(); rclpy.shutdown(); self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk(); app = VehicleControl(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing); root.mainloop()