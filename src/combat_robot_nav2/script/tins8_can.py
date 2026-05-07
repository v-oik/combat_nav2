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
# Config (T8 CANopen Protocol 적용)
# --------------------
# Node ID: 1, CAN Baud Rate: 250k
CMD_CAN_ID = 0x201          # RPDO0 (Master -> Chassis 제어)
FEEDBACK_CAN_ID = 0x181     # TPDO0 (Chassis -> Master 피드백)

CAN_SEND_FREQ_HZ = 30       # 전송 주기 (< 1S 연속 입력 필수)
INTER_MSG_GAP_SEC = 0.0005  

DEAD_ZONE_STEER = 10        # 스케일러가 작아졌으므로 데드존도 줄임
DEAD_ZONE_SPEED = 10        

# Joystick axis mapping & Scaler
JOY_AXIS_STEER = 0   # Left stick X
JOY_AXIS_SPEED = 4   # Left stick Y

# --- 속도 스케일러 설정 (TINS-8) ---
# TINS-8 권장 속도 범위: 23 ~ 230
JOY_SCALER_STEER = 100   # 회전 시 최대 속도 차이 (최대 속도에 비례하여 하향 조정)
JOY_SCALER_SPEED = 230   # 전/후진 시 최대 속도 (TINS-8 기준)

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
            # PCAN 전용으로 고정 설정 (Baudrate 250k)
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
            steer_raw = self.joy.get_axis(JOY_AXIS_STEER)   # -1 ~ 1
            speed_raw = self.joy.get_axis(JOY_AXIS_SPEED)   # -1 ~ 1

            # Invert speed (stick up = negative on many pads)
            speed_raw = -speed_raw

            steer = int(steer_raw * JOY_SCALER_STEER)
            speed = int(speed_raw * JOY_SCALER_SPEED)

            # 큐를 통해 UI 스레드에 조이스틱 데이터 전달
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
                # 큐를 통해 UI 스레드에 CAN 수신 데이터 전달
                self.ui_queue.put(("rx", msg))
            except Exception as e:
                print("Rx callback error:", e)

# --------------------
# Main UI controller (Tkinter)
# --------------------
class VehicleControl:
    def __init__(self, root):
        self.root = root
        self.root.title("TINS-8 Combat Control")
        self.root.geometry("500x400")

        # Variables
        self.current_steer = tk.IntVar(value=0)
        self.current_speed = tk.IntVar(value=0)
        self.control_enabled = tk.BooleanVar(value=False)
        self.laser_enabled = tk.BooleanVar(value=False) # Headlights
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
        self.root.bind('<Left>', self.decrease_steer)
        self.root.bind('<Right>', self.increase_steer)
        self.root.bind('<space>', self.reset_controls)

        # Start background tasks
        self.tx_worker.start()
        self.rx_worker.start()

        if self.can.bus is None:
            self.status_label.config(text="CAN: (simulated) - interface not up.")
        else:
            self.status_label.config(text="CAN: connected (PCAN 250k)")

        # Main loops
        self.process_ui_queue()
        self.push_tx_loop()

    def init_ui(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        # Control Checkboxes
        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill=tk.X, pady=10)
        
        ttk.Checkbutton(ctrl_frame, text="Enable Control (Start/Stop)", variable=self.control_enabled).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(ctrl_frame, text="Headlights", variable=self.laser_enabled).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(ctrl_frame, text="Enable Joystick", variable=self.joystick_enabled, command=self.toggle_joystick).pack(side=tk.LEFT, padx=5)

        # Sliders
        slider_frame = ttk.LabelFrame(frame, text="Manual Control (Use Arrow Keys / Spacebar to Stop)")
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # Steer
        ttk.Label(slider_frame, text="Steering:").grid(row=0, column=0, padx=5, pady=10, sticky="w")
        self.steer_slider = ttk.Scale(slider_frame, from_=-JOY_SCALER_STEER, to=JOY_SCALER_STEER, orient=tk.HORIZONTAL, variable=self.current_steer)
        self.steer_slider.grid(row=0, column=1, padx=5, sticky="ew")
        self.steer_val_lbl = ttk.Label(slider_frame, text="0", width=5)
        self.steer_val_lbl.grid(row=0, column=2, padx=5)

        # Speed
        ttk.Label(slider_frame, text="Speed:").grid(row=1, column=0, padx=5, pady=10, sticky="w")
        self.speed_slider = ttk.Scale(slider_frame, from_=-JOY_SCALER_SPEED, to=JOY_SCALER_SPEED, orient=tk.HORIZONTAL, variable=self.current_speed)
        self.speed_slider.grid(row=1, column=1, padx=5, sticky="ew")
        self.speed_val_lbl = ttk.Label(slider_frame, text="0", width=5)
        self.speed_val_lbl.grid(row=1, column=2, padx=5)

        slider_frame.columnconfigure(1, weight=1)

        # Status
        self.status_label = ttk.Label(frame, text="Status: Ready", font=("Arial", 10, "bold"))
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        self.current_steer.trace_add('write', lambda *args: self.steer_val_lbl.config(text=str(self.current_steer.get())))
        self.current_speed.trace_add('write', lambda *args: self.speed_val_lbl.config(text=str(self.current_speed.get())))

    # -----------------------------------
    # 키보드 이벤트 처리 함수들 (TINS-8 스케일에 맞춰 감소)
    # -----------------------------------
    def increase_speed(self, event=None):
        step = 20 # 230 범위에 맞춰 부드러운 가속을 위해 20으로 설정
        new_val = min(self.current_speed.get() + step, JOY_SCALER_SPEED)
        self.current_speed.set(new_val)

    def decrease_speed(self, event=None):
        step = 20
        new_val = max(self.current_speed.get() - step, -JOY_SCALER_SPEED)
        self.current_speed.set(new_val)

    def increase_steer(self, event=None):
        step = 10 # 조향 단위 축소
        new_val = min(self.current_steer.get() + step, JOY_SCALER_STEER)
        self.current_steer.set(new_val)

    def decrease_steer(self, event=None):
        step = 10
        new_val = max(self.current_steer.get() - step, -JOY_SCALER_STEER)
        self.current_steer.set(new_val)

    def reset_controls(self, event=None):
        self.current_speed.set(0)
        self.current_steer.set(0)

    # -----------------------------------
    # 시스템 컨트롤 함수들
    # -----------------------------------
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
                # TPDO 피드백: Left Wheel (Byte 0-1), Right Wheel (Byte 2-3)
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

        alpha = 0.3
        self.filterd_steer = (1 - alpha) * self.filterd_steer + alpha * steer
        self.filterd_speed = (1 - alpha) * self.filterd_speed + alpha * speed
        
        left_wheel = int(self.filterd_speed + self.filterd_steer)
        right_wheel = int(self.filterd_speed - self.filterd_steer)

        left_wheel = max(-JOY_SCALER_SPEED, min(JOY_SCALER_SPEED, left_wheel))
        right_wheel = max(-JOY_SCALER_SPEED, min(JOY_SCALER_SPEED, right_wheel))

        # 데이터 구조 매핑 (RPDO 0x201)
        # Byte 0-1: 우측 바퀴 속도, Byte 2-3: 좌측 바퀴 속도
        # Byte 4: Light Control (01: Off, 03: High Beam)
        light_ctrl = 0x03 if self.laser_enabled.get() else 0x01
        
        # Byte 5: Radar Enable (00: Enable)
        radar = 0x00 
        
        # Byte 6: Start/Stop (01: Start, 00: Stop)
        start_stop = 0x01 if self.control_enabled.get() else 0x00
        
        # Byte 7: Speed Loop Control Mode (05)
        mode = 0x05 

        payload = struct.pack("<hhBBBB", right_wheel, left_wheel, light_ctrl, radar, start_stop, mode)

        try:
            self.tx_queue.put_nowait((CMD_CAN_ID, payload))
        except queue.Full:
            pass

        self.root.after(round(1000 / CAN_SEND_FREQ_HZ), self.push_tx_loop)

    def on_closing(self):
        print("Shutting down threads...")
        # 안전한 정지를 위해 속도 0, 제어 중지, 조명 꺼짐 신호 전송
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