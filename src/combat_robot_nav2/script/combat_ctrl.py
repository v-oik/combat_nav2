import sys
import struct
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk

# python-can import
try:
    import can
except ImportError as e:
    can = None
    print("python-can not available:", e)

# Joystick (pygame) import
try:
    import pygame
except ImportError as e:
    pygame = None
    print("pygame not available for joystick:", e)

# --------------------
# Config (TinS-13 CANopen Protocol 적용)
# --------------------
# Node ID: 1, CAN Baud Rate: 250k
CMD_CAN_ID = 0x201          # RPDO0 (Master -> Chassis 제어)
FEEDBACK_CAN_ID = 0x181     # TPDO0 (Chassis -> Master 피드백)

CAN_SEND_FREQ_HZ = 30       # 전송 주기 (TinS-13은 <1s 연속 입력 필수)
INTER_MSG_GAP_SEC = 0.0005  

DEAD_ZONE_STEER = 50        
DEAD_ZONE_SPEED = 50        

# Joystick axis mapping & Scaler (Max speed: 5600 line/s)
JOY_AXIS_STEER = 0   # Left stick X
JOY_AXIS_SPEED = 4   # Left stick Y
JOY_SCALER_STEER = 2000  # 회전 시 최대 속도 차이
JOY_SCALER_SPEED = 5600  # 전/후진 시 최대 속도

# --------------------
# CAN Bus Wrapper
# --------------------
class CanBusWrapper:
    def __init__(self):
        self.bus = None
        self.init_bus()

    def init_bus(self):
        if can is None:
            print("python-can module not installed; CAN disabled")
            return
        try:
            # PCAN 전용으로 고정 설정
            self.bus = can.interface.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=250000)
            print("PCAN Bus Initialized (PCAN_USBBUS1, 250kbps)")
        except Exception as e:
            print("CAN init error:", e)
            self.bus = None

    def send(self, msg):
        if self.bus is None:
            return
        try:
            self.bus.send(msg)
        except Exception as e:
            print("CAN send error:", e)

    def recv(self, timeout=1.0):
        if self.bus is None:
            time.sleep(timeout)
            return None
        try:
            return self.bus.recv(timeout=timeout)
        except Exception as e:
            print("CAN recv error:", e)
            return None

# ==========================
# Joystick Worker Thread
# ==========================
class JoystickWorker(threading.Thread):
    def __init__(self, ui_queue, stop_event):
        super().__init__(daemon=True)
        self.ui_queue = ui_queue
        self.stop_event = stop_event

        if pygame is not None:
            pygame.init()
            pygame.joystick.init()
            self.joy = None
            if pygame.joystick.get_count() > 0:
                self.joy = pygame.joystick.Joystick(0)
                self.joy.init()
                print("Joystick connected:", self.joy.get_name())
            else:
                print("No joystick found.")
        else:
            self.joy = None

    def run(self):
        if self.joy is None:
            return

        while not self.stop_event.is_set():
            pygame.event.pump()
            
            # 조이스틱 스틱을 왼쪽으로 밀 때 양수(+)가 나오도록 유지
            steer_raw = -self.joy.get_axis(JOY_AXIS_STEER) 
            speed_raw = self.joy.get_axis(JOY_AXIS_SPEED)

            # Invert speed (stick up = negative on many pads)
            speed_raw = -speed_raw

            steer = int(steer_raw * JOY_SCALER_STEER)
            speed = int(speed_raw * JOY_SCALER_SPEED)

            self.ui_queue.put(("joy", (steer, speed)))
            time.sleep(0.02)   # 50Hz

# --------------------
# Threads / Workers
# --------------------
class TxWorker(threading.Thread):
    def __init__(self, can_wrapper: CanBusWrapper, tx_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.can = can_wrapper
        self.tx_queue = tx_queue
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            try:
                item = self.tx_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                arb_id, payload = item
                msg = can.Message(arbitration_id=arb_id, data=payload, is_extended_id=False)
                self.can.send(msg)
            except Exception as e:
                print("TxWorker send error:", e)

            time.sleep(INTER_MSG_GAP_SEC)

class RxWorker(threading.Thread):
    def __init__(self, can_wrapper: CanBusWrapper, ui_queue: queue.Queue, stop_event: threading.Event):
        super().__init__(daemon=True)
        self.can = can_wrapper
        self.ui_queue = ui_queue  
        self.stop_event = stop_event

    def run(self):
        while not self.stop_event.is_set():
            msg = self.can.recv(timeout=0.5)
            if msg is None:
                continue
            try:
                self.ui_queue.put(("rx", msg))
            except Exception as e:
                print("Rx callback error:", e)

# --------------------
# Main UI controller (Tkinter)
# --------------------
class VehicleControl:
    def __init__(self, root):
        self.root = root
        self.root.title("TinS-13 Combat Control")
        self.root.geometry("500x400")

        # Variables
        self.current_steer = tk.IntVar(value=0)
        self.current_speed = tk.IntVar(value=0)
        self.control_enabled = tk.BooleanVar(value=False)
        self.laser_enabled = tk.BooleanVar(value=False)
        self.joystick_enabled = tk.BooleanVar(value=False)

        self.filterd_steer = 0.0
        self.filterd_speed = 0.0

        # Hardware setup
        self.can = CanBusWrapper()
        self.tx_queue = queue.Queue()
        self.ui_queue = queue.Queue()
        self.stop_event = threading.Event()

        self.tx_worker = TxWorker(self.can, self.tx_queue, self.stop_event)
        self.rx_worker = RxWorker(self.can, self.ui_queue, self.stop_event)
        
        self.joy_thread = None
        self.joy_stop_event = None

        self.init_ui()

        # --- 키보드 방향키 바인딩 ---
        self.root.bind('<Up>', self.increase_speed)
        self.root.bind('<Down>', self.decrease_speed)
        
        # 왼쪽 화살표 = 값 증가(+) / 오른쪽 화살표 = 값 감소(-)
        self.root.bind('<Left>', self.increase_steer)
        self.root.bind('<Right>', self.decrease_steer)
        
        self.root.bind('<space>', self.reset_controls)

        self.tx_worker.start()
        self.rx_worker.start()

        if self.can.bus is None:
            self.status_label.config(text="CAN: (simulated) - interface not up.")
        else:
            self.status_label.config(text="CAN: connected (PCAN 250k)")

        self.process_ui_queue()
        self.push_tx_loop()

    def init_ui(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill=tk.X, pady=10)
        
        ttk.Checkbutton(ctrl_frame, text="Enable Control (Start/Stop)", variable=self.control_enabled).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(ctrl_frame, text="Headlights (Laser)", variable=self.laser_enabled).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(ctrl_frame, text="Enable Joystick", variable=self.joystick_enabled, command=self.toggle_joystick).pack(side=tk.LEFT, padx=5)

        slider_frame = ttk.LabelFrame(frame, text="Manual Control (Use Arrow Keys / Spacebar to Stop)")
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # UI 슬라이더 시각적 일치 (왼쪽이 +, 오른쪽이 -)
        ttk.Label(slider_frame, text="Steering:").grid(row=0, column=0, padx=5, pady=10, sticky="w")
        self.steer_slider = ttk.Scale(slider_frame, from_=JOY_SCALER_STEER, to=-JOY_SCALER_STEER, orient=tk.HORIZONTAL, variable=self.current_steer)
        self.steer_slider.grid(row=0, column=1, padx=5, sticky="ew")
        self.steer_val_lbl = ttk.Label(slider_frame, text="0", width=5)
        self.steer_val_lbl.grid(row=0, column=2, padx=5)

        ttk.Label(slider_frame, text="Speed:").grid(row=1, column=0, padx=5, pady=10, sticky="w")
        self.speed_slider = ttk.Scale(slider_frame, from_=JOY_SCALER_SPEED, to=-JOY_SCALER_SPEED, orient=tk.HORIZONTAL, variable=self.current_speed)
        self.speed_slider.grid(row=1, column=1, padx=5, sticky="ew")
        self.speed_val_lbl = ttk.Label(slider_frame, text="0", width=5)
        self.speed_val_lbl.grid(row=1, column=2, padx=5)

        slider_frame.columnconfigure(1, weight=1)

        self.status_label = ttk.Label(frame, text="Status: Ready", font=("Arial", 10, "bold"))
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        self.current_steer.trace_add('write', lambda *args: self.steer_val_lbl.config(text=str(self.current_steer.get())))
        self.current_speed.trace_add('write', lambda *args: self.speed_val_lbl.config(text=str(self.current_speed.get())))

    def increase_speed(self, event=None):
        step = 200  
        new_val = min(self.current_speed.get() + step, JOY_SCALER_SPEED)
        self.current_speed.set(new_val)

    def decrease_speed(self, event=None):
        step = 200
        new_val = max(self.current_speed.get() - step, -JOY_SCALER_SPEED)
        self.current_speed.set(new_val)

    def increase_steer(self, event=None):
        step = 100  
        new_val = min(self.current_steer.get() + step, JOY_SCALER_STEER)
        self.current_steer.set(new_val)

    def decrease_steer(self, event=None):
        step = 100
        new_val = max(self.current_steer.get() - step, -JOY_SCALER_STEER)
        self.current_steer.set(new_val)

    def reset_controls(self, event=None):
        self.current_speed.set(0)
        self.current_steer.set(0)

    def toggle_joystick(self):
        state = self.joystick_enabled.get()
        if state:  
            self.status_label.config(text="Joystick enabled")
            self.joy_stop_event = threading.Event()
            self.joy_thread = JoystickWorker(self.ui_queue, self.joy_stop_event)
            self.joy_thread.start()
        else:
            self.status_label.config(text="Joystick disabled")
            if self.joy_thread:
                self.joy_stop_event.set()
                self.joy_thread.join()
                self.joy_thread = None

    def process_ui_queue(self):
        while not self.ui_queue.empty():
            msg_type, data = self.ui_queue.get()
            
            if msg_type == "rx":
                msg = data
                if msg.arbitration_id == FEEDBACK_CAN_ID and len(msg.data) >= 4:
                    left_actual, right_actual = struct.unpack("<hh", msg.data[0:4])
                    txt = f"Actual Speed -> L: {left_actual}, R: {right_actual}"
                    self.status_label.config(text=txt)
                    
            elif msg_type == "joy":
                steer, speed = data
                self.current_steer.set(steer)
                self.current_speed.set(speed)

        self.root.after(33, self.process_ui_queue)

    def apply_deadzone(self, value, dz):
        if abs(value) <= dz:
            return 0
        return value

    def push_tx_loop(self):
        steer = self.apply_deadzone(self.current_steer.get(), DEAD_ZONE_STEER)
        speed = self.apply_deadzone(self.current_speed.get(), DEAD_ZONE_SPEED)

        # LPF
        alpha = 0.3
        self.filterd_steer = (1 - alpha) * self.filterd_steer + alpha * steer
        self.filterd_speed = (1 - alpha) * self.filterd_speed + alpha * speed
        
        # 🌟 [물리적 하드웨어 방향에 맞게 모터 부호 최종 수정]
        # 왼쪽 키 누름 -> steer 양수(+) -> 하드웨어에서 왼쪽으로 돌도록 + / - 배치 변경
        left_wheel = int(self.filterd_speed + self.filterd_steer)
        right_wheel = int(self.filterd_speed - self.filterd_steer)

        # 섀시 최대 한계 속도 클램핑
        left_wheel = max(-JOY_SCALER_SPEED, min(JOY_SCALER_SPEED, left_wheel))
        right_wheel = max(-JOY_SCALER_SPEED, min(JOY_SCALER_SPEED, right_wheel))

        # 8-Byte Payload 구성
        light_ctrl = 0x03 if self.laser_enabled.get() else 0x02
        radar = 0x00
        start_stop = 0x01 if self.control_enabled.get() else 0x00
        mode = 0x05

        payload = struct.pack("<hhBBBB", right_wheel, left_wheel, light_ctrl, radar, start_stop, mode)

        try:
            self.tx_queue.put_nowait((CMD_CAN_ID, payload))
        except queue.Full:
            pass

        self.root.after(round(1000 / CAN_SEND_FREQ_HZ), self.push_tx_loop)

    def on_closing(self):
        print("Shutting down threads...")
        payload = struct.pack("<hhBBBB", 0, 0, 0x01, 0x00, 0x00, 0x05)
        try:
            self.tx_queue.put_nowait((CMD_CAN_ID, payload))
        except queue.Full:
            pass
            
        self.stop_event.set()
        if self.joy_thread:
            self.joy_stop_event.set()
            self.joy_thread.join(timeout=1.0)

        self.tx_worker.join(timeout=1.0)
        self.rx_worker.join(timeout=1.0)
        
        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = VehicleControl(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()