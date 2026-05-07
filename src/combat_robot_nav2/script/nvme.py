import serial
import time

# 설정 (현재 사용 중인 환경과 동일하게 세팅)
PORT = '/dev/ttyUSB0'
BAUD = 921600

def run_test():
    try:
        # 시리얼 포트 열기
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"--- {PORT} 연결 성공 (Baud: {BAUD}) ---")
        print("데이터를 모니터링합니다. (종료: Ctrl+C)")

        while True:
            if ser.in_waiting > 0:
                # 한 줄 읽기
                line = ser.readline().decode('ascii', errors='ignore').strip()
                
                # $GNTHS 또는 $GPTHS 문장만 필터링
                if 'THS' in line:
                    parts = line.split(',')
                    if len(parts) >= 2 and parts[1]:
                        heading = parts[1]
                        status = parts[2].split('*')[0] if len(parts) >= 3 else "N/A"
                        print(f"[{time.strftime('%H:%M:%S')}] 헤딩 각도: {heading}° (상태: {status})")
    
    except serial.SerialException as e:
        print(f"포트 연결 에러: {e}")
    except KeyboardInterrupt:
        print("\\n테스트를 종료합니다.")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    run_test()