import sys
import time
import threading
import queue
import struct
import tkinter as tk
from tkinter import ttk

# Joystick (pygame) import
try:
    import pygame
except ImportError as e:
    pygame = None
    print("pygame not available for joystick:", e)

# Modbus import
try:
    from pymodbus.client import ModbusSerialClient
except ImportError as e:
    ModbusSerialClient = None
    print("pymodbus not available. Please install: pip install pymodbus pyserial")

# --------------------
# Config (TINS-8 Modbus RTU Protocol 적용)
# --------------------
# 시리얼 포트 파라미터 (매뉴얼 기준) 
MODBUS_PORT = 'COM3'         # ★ 사용하시는 PC의 COM 포트로 변경하세요 (Linux의 경우 '/dev/ttyUSB0')
MODBUS_BAUD = 38400          # [cite: 163]
SLAVE_ID = 1                 # 기본 Slave ID [cite: 167]

# 레지스터 주소 맵 (Hex -> Decimal 변환) [cite: 174, 176]
REG_LEFT_SPEED_CMD  = 0x0C   # 12 [cite: 174]
REG_RIGHT_SPEED_CMD = 0x0D   # 13 [cite: 174]
REG_CTRL_MODE       = 0x50   # 80 (1: 속도 폐루프 모드) [cite: 174]
REG_START_STOP      = 0x51   # 81 (0: 정지, 1: 시작) [cite: 174]
REG_ACTUAL_LEFT     = 0x36   # 54 [cite: 176]
REG_ACTUAL_RIGHT    = 0x37   # 55 [cite: 176]

# 제어 주기 및 데드존
MODBUS_LOOP_DELAY = 0.05     # 50ms 권장 [cite: 189]
DEAD_ZONE_STEER = 100
DEAD_ZONE_SPEED = 100

# Joystick axis mapping & Scaler (TINS-8 Modbus 스케일)
JOY_AXIS_STEER = 0   
JOY_AXIS_SPEED = 4   

# Modbus 단위는 line/second. 매뉴얼 권장 범위 600~3000 
JOY_SCALER_STEER = 1000  # 회전 시 최대 속도 차이
JOY_SCALER_SPEED = 3000  # 전/후진 시 최대 속도 

# --------------------
# Helper Functions
# --------------------
def to_uint16(val):
    """ Signed 16-bit int를 Modbus용 Unsigned 16-bit로 변환 """
    return int(val) & 0xFFFF

def to_int16(val):
    """ Modbus에서 읽은 Unsigned 16-bit를 Signed 16-bit로 변환 """
    if val > 32767:
        return val - 65536
    return val

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
            steer_raw = self.joy.get_axis(JOY_AXIS_STEER)
            speed_raw = self.joy.get_axis(JOY_AXIS_SPEED)

            # Invert speed (stick up = negative on many pads)
            speed_raw = -speed_raw

            steer = int(steer_raw * JOY_SCALER_STEER)
            speed = int(speed_raw * JOY_SCALER_SPEED)

            self.ui_queue.put(("joy", (steer, speed)))
            time.sleep(0.02)

# ==========================
# Modbus RTU Worker Thread (Tx/Rx 통합)
# ==========================
class ModbusWorker(threading.Thread):
    def __init__(self, port, ui_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.ui_queue = ui_queue
        self.stop_event = stop_event
        self.client = None
        self.connected = False
        
        # 외부에서 변경 가능한 상태 변수 (스레드 안전성 위해 단순 복사 허용)
        self.target_left = 0
        self.target_right = 0
        self.is_started = False
        self.prev_is_started = None

    def run(self):
        if ModbusSerialClient is None:
            return
            
        try:
            self.client = ModbusSerialClient(
                port=self.port, 
                baudrate=MODBUS_BAUD, 
                bytesize=8, 
                parity='N', 
                stopbits=1, 
                timeout=0.1
            )
            self.connected = self.client.connect()
            if self.connected:
                print(f"Modbus connected to {self.port} at {MODBUS_BAUD}bps")
                
                # 초기 설정: 속도 폐루프 모드(1) 설정 [cite: 174]
                self.client.write_register(REG_CTRL_MODE, 1, slave=SLAVE_ID)
            else:
                print(f"Failed to connect Modbus on {self.port}")
        except Exception as e:
            print(f"Modbus init error: {e}")
            self.connected = False

        while not self.stop_event.is_set():
            if not self.connected:
                time.sleep(1)
                continue
                
            try:
                # 1. Start/Stop 상태가 변경되었으면 업데이트 (0x51 레지스터) [cite: 174]
                if self.is_started != self.prev_is_started:
                    cmd_val = 1 if self.is_started else 0
                    self.client.write_register(REG_START_STOP, cmd_val, slave=SLAVE_ID) [cite: 174]
                    self.prev_is_started = self.is_started

                # 2. 목표 속도 전송 (0x0C, 0x0D 레지스터) [cite: 174, 187]
                if self.is_started:
                    self.client.write_register(REG_LEFT_SPEED_CMD, to_uint16(self.target_left), slave=SLAVE_ID) [cite: 174]
                    self.client.write_register(REG_RIGHT_SPEED_CMD, to_uint16(self.target_right), slave=SLAVE_ID) [cite: 174]

                # 3. 실제 속도 피드백 읽기 (0x36번부터 2개 연속 읽기) [cite: 176]
                result = self.client.read_holding_registers(REG_ACTUAL_LEFT, 2, slave=SLAVE_ID) [cite: 176]
                if not result.isError():
                    act_left = to_int16(result.registers[0])
                    act_right = to_int16(result.registers[1])
                    self.ui_queue.put(("rx", (act_left, act_right)))
                
            except Exception as e:
                print(f"Modbus loop error: {e}")

            # 매뉴얼 권장 통신 주기 (50ms) [cite: 189]
            time.sleep(MODBUS_LOOP_DELAY)
            
        # 스레드 종료 시 안전 정지
        if self.connected:
            self.client.write_register(REG_LEFT_SPEED_CMD, 0, slave=SLAVE_ID) [cite: 174]
            self.client.write_register(REG_RIGHT_SPEED_CMD, 0, slave=SLAVE_ID) [cite: 174]
            self.client.write_register(REG_START_STOP, 0, slave=SLAVE_ID) [cite: 174]
            self.client.close()

# --------------------
# Main UI controller
# --------------------
class VehicleControl:
    def __init__(self, root):
        self.root = root
        self.root.title("TINS-8 RS485 Modbus Control")
        self.root.geometry("500x400")

        self.current_steer = tk.IntVar(value=0)
        self.current_speed = tk.IntVar(value=0)
        self.control_enabled = tk.BooleanVar(value=False)
        self.joystick_enabled = tk.BooleanVar(value=False)

        self.filterd_steer = 0.0
        self.filterd_speed = 0.0

        self.ui_queue = queue.Queue()
        self.stop_event = threading.Event()

        # Modbus Worker 시작
        self.modbus_worker = ModbusWorker(MODBUS_PORT, self.ui_queue, self.stop_event)
        self.modbus_worker.start()
        
        self.joy_thread = None
        self.joy_stop_event = None

        self.init_ui()

        # 방향키 바인딩
        self.root.bind('<Up>', self.increase_speed)
        self.root.bind('<Down>', self.decrease_speed)
        self.root.bind('<Left>', self.decrease_steer)
        self.root.bind('<Right>', self.increase_steer)
        self.root.bind('<space>', self.reset_controls)

        # UI 업데이트 루프 시작
        self.process_ui_queue()
        self.push_tx_loop()

    def init_ui(self):
        frame = ttk.Frame(self.root, padding="10")
        frame.pack(fill=tk.BOTH, expand=True)

        ctrl_frame = ttk.Frame(frame)
        ctrl_frame.pack(fill=tk.X, pady=10)
        
        ttk.Checkbutton(ctrl_frame, text="Enable Control (Start)", variable=self.control_enabled).pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(ctrl_frame, text="Enable Joystick", variable=self.joystick_enabled, command=self.toggle_joystick).pack(side=tk.LEFT, padx=5)

        slider_frame = ttk.LabelFrame(frame, text="Manual Control (Arrow Keys / Spacebar to Stop)")
        slider_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        ttk.Label(slider_frame, text="Steering:").grid(row=0, column=0, padx=5, pady=10, sticky="w")
        self.steer_slider = ttk.Scale(slider_frame, from_=-JOY_SCALER_STEER, to=JOY_SCALER_STEER, orient=tk.HORIZONTAL, variable=self.current_steer)
        self.steer_slider.grid(row=0, column=1, padx=5, sticky="ew")
        self.steer_val_lbl = ttk.Label(slider_frame, text="0", width=5)
        self.steer_val_lbl.grid(row=0, column=2, padx=5)

        ttk.Label(slider_frame, text="Speed:").grid(row=1, column=0, padx=5, pady=10, sticky="w")
        self.speed_slider = ttk.Scale(slider_frame, from_=-JOY_SCALER_SPEED, to=JOY_SCALER_SPEED, orient=tk.HORIZONTAL, variable=self.current_speed)
        self.speed_slider.grid(row=1, column=1, padx=5, sticky="ew")
        self.speed_val_lbl = ttk.Label(slider_frame, text="0", width=5)
        self.speed_val_lbl.grid(row=1, column=2, padx=5)

        slider_frame.columnconfigure(1, weight=1)

        self.status_label = ttk.Label(frame, text="Status: Ready", font=("Arial", 10, "bold"))
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

        self.current_steer.trace_add('write', lambda *args: self.steer_val_lbl.config(text=str(self.current_steer.get())))
        self.current_speed.trace_add('write', lambda *args: self.speed_val_lbl.config(text=str(self.current_speed.get())))

        # 초기 연결 상태 표시
        if not ModbusSerialClient:
            self.status_label.config(text="Modbus: pymodbus library missing.")
        else:
            self.root.after(1000, self.check_connection_status)

    def check_connection_status(self):
        if self.modbus_worker.connected:
            self.status_label.config(text=f"Modbus: Connected to {MODBUS_PORT}")
        else:
            self.status_label.config(text=f"Modbus: Connection Failed on {MODBUS_PORT}")

    def increase_speed(self, event=None):
        step = 300 
        new_val = min(self.current_speed.get() + step, JOY_SCALER_SPEED)
        self.current_speed.set(new_val)

    def decrease_speed(self, event=None):
        step = 300
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
            self.joy_stop_event = threading.Event()
            self.joy_thread = JoystickWorker(self.ui_queue, self.joy_stop_event)
            self.joy_thread.start()
        else:
            if self.joy_thread:
                self.joy_stop_event.set()
                self.joy_thread.join()
                self.joy_thread = None

    def process_ui_queue(self):
        while not self.ui_queue.empty():
            msg_type, data = self.ui_queue.get()
            
            if msg_type == "rx":
                # 실제 속도 피드백 업데이트
                act_left, act_right = data
                txt = f"Actual Speed (line/s) -> L: {act_left}, R: {act_right}"
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

        # Modbus Worker 스레드에 목표값 전달
        self.modbus_worker.target_left = left_wheel
        self.modbus_worker.target_right = right_wheel
        self.modbus_worker.is_started = self.control_enabled.get()

        # 30Hz 주기로 상태 갱신
        self.root.after(33, self.push_tx_loop)

    def on_closing(self):
        print("Shutting down threads...")
        self.stop_event.set()
        
        if self.joy_thread:
            self.joy_stop_event.set()
            self.joy_thread.join(timeout=1.0)

        self.modbus_worker.join(timeout=1.5)
        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = VehicleControl(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()