# combat_nav2




```
cd existing_repo
git remote add origin https://gitlab.com/tmd2637/birdro_nav2.git
git branch -M main
git push -uf origin main
```


## Prerequisites & Installation

`skyautonet_birdro_nav2` 워크스페이스를 빌드하고 자율주행 노드를 구동하기 위해 필요한 ROS 2 Humble 핵심 패키지들을 설치합니다.

```bash
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-mppi-controller \ 
    ros-humble-robot-localization \
    ros-humble-rviz2\
    ros-humble-nmea-navsat-driver\
    ros-humble-nmea-msgs
```






#   RoboSense LiDAR (rslidar) ROS 2 연동 및 데이터 수신 가이드

본 문서는 **Ubuntu 22.04** 및 **ROS 2 (Humble)** 환경에서 **RoboSense LiDAR**만을 단독으로 연동하고 데이터를 수집하는 전 과정을 다루는 매뉴얼입니다.

---

##  1단계: 라이다 유선 네트워크 설정 (고정 IP)

라이다는 기본적으로 `192.168.1.200` IP를 가지며 `192.168.1.xxx` 대역의 PC로 데이터를 전송합니다. 와이파이(인터넷)는 그대로 쓰면서, 라이다가 연결된 랜 포트에 전용 IP를 추가합니다.

```bash
# 1. 랜 포트 이름 확인 (예: eth0, eno1 등)
ip a

# 2. 고정 IP 추가 (기존 IP를 유지하며 라이다용 IP 1.102를 추가함)
# ※ 'eth0' 부분은 ip a 에서 확인한 본인의 포트 이름으로 수정하세요.
sudo ip addr add 192.168.1.102/24 dev eth0
sudo ip link set eth0 up

# 3. 통신 확인 (64 bytes from... 응답이 오면 성공)
ping 192.168.1.200
```

---

##  2단계: 드라이버 다운로드 및 의존성 설치

ROS 2 워크스페이스(`~/ros2_ws`)에서 진행합니다.

### 2.1 시스템 의존성 설치 (필수)
네트워크 패킷 캡처 및 설정 파일 파싱을 위한 시스템 라이브러리를 설치합니다.
```bash
sudo apt update
sudo apt install -y libpcap-dev libyaml-cpp-dev
```

### 2.2 소스코드 다운로드 및 서브모듈 초기화
```bash
cd ~/ros2_ws/src

# 1. LiDAR 메시지 및 SDK 클론
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git

# 2. LiDAR SDK 내부 핵심 드라이버(서브모듈) 가져오기
cd rslidar_sdk
git submodule init
git submodule update
```

---

## ⚙️ 3단계: LiDAR 소스코드 및 설정 파일 세부 수정

`rslidar_sdk`는 ROS 1/2 공용이므로 ROS 2에 맞게 설정을 변경해야 합니다.

### 3.1 빌드 방식 변경 (ROS 2 전용)
```bash
cd ~/ros2_ws/src/rslidar_sdk
cp package_ros2.xml package.xml

# CMakeLists.txt의 빌드 옵션을 ORIGINAL에서 COLCON으로 변경
sed -i 's/set(COMPILE_METHOD ORIGINAL)/set(COMPILE_METHOD COLCON)/g' CMakeLists.txt
```

### 3.2 LiDAR 설정 파일 (`config.yaml`) 수정
`nano ~/ros2_ws/src/rslidar_sdk/config/config.yaml` 명령으로 파일을 열어 아래 **3가지 핵심 파라미터**를 반드시 확인하고 수정합니다.

```yaml
common:
  msg_source: 1                  # 1: 실 실시간 센서 수신 (필수)
  send_packet_ros: false
  send_point_cloud_ros: true     # ROS PointCloud2 토픽 발행 (필수)

lidar:
  - driver:
      lidar_type: RSE1           # [중요] 사용 중인 모델명 (예: RSE1, RSM1 등. 틀리면 MSOP 에러 발생)
      msop_port: 6699
      difop_port: 7788
      host_address: 0.0.0.0      # 전체 수신 허용
```

### 3.3 Headless 실행 설정 (SSH 환경 RViz2 에러 방지)
SSH 원격 접속 시 디스플레이(GUI)가 없어 RViz2 렌더링 에러로 드라이버가 종료되는 것을 막기 위해 런치 파일을 수정합니다.
`nano ~/ros2_ws/src/rslidar_sdk/launch/start.py`

* 파일 하단 `return LaunchDescription([...])` 내부의 **`rviz2` 관련 `Node` 부분을 삭제하거나 주석 처리**합니다.

```python
    return LaunchDescription([
        Node(
            namespace='rslidar_sdk',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            output='screen'
        )
        # --- 아래 부분은 삭제 또는 주석 처리 ---
        # , Node(
        #     namespace='rviz2',
        #     ...
        # )
    ])
```

---

##  4단계: 컴파일 및 개별 드라이버 실행

### 4.1 워크스페이스 빌드
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select rslidar_msg rslidar_sdk
source install/setup.bash
```

### 4.2 센서 가동
드라이버를 실행하여 라이다 패킷 수신을 시작합니다.
```bash
ros2 launch rslidar_sdk start.py
```

---

## 📼 5단계: 센서 데이터 검증 및 녹화 (Rosbag)

라이다가 정상적으로 토픽을 발행하는지 확인 후, Bag 파일로 저장합니다. 새로운 터미널을 열고 진행합니다.

```bash
source ~/ros2_ws/install/setup.bash

# 데이터 수신 상태 확인 (E1 모델 기준 약 10Hz가 정상)
ros2 topic hz /rslidar_points

# 정상 확인 후 녹화 시작
cd ~/ros2_ws
ros2 bag record -o rslidar_dataset /rslidar_points

# 녹화 종료: Ctrl + C
# 저장 검증: ros2 bag info rslidar_dataset
```

---

## 🖥️ 6단계: PC 시각화 및 재생 트러블슈팅

저장된 Bag 파일을 윈도우/맥/리눅스 메인 PC로 옮겨서 재생할 때의 필수 설정입니다.

### 6.1 시간 동기화 옵션 재생
과거의 데이터를 현재 시스템 시간으로 재생해야 시스템이 데이터를 버리지 않습니다.
```bash
ros2 bag play rslidar_dataset --clock -l
```

### 6.2 RViz2 필수 세팅 (데이터가 안 보일 때)
RViz2를 켜고 `PointCloud2` 토픽(`/rslidar_points`)을 추가한 뒤, 아래 두 곳을 무조건 수정해야 점군이 나타납니다.

1. **Fixed Frame 변경:** 왼쪽 `Global Options > Fixed Frame`의 `map`을 지우고 **`rslidar`** 로 직접 타이핑하여 입력합니다.
2. **QoS 변경 [매우 중요]:** 추가한 `PointCloud2` 세부 메뉴에서 `Topic > QoS > Reliability Policy`를 `Reliable`에서 **`Best Effort`** 로 변경합니다.


#  [Master Guide] Xsens MTi IMU (bluespace-ai) ROS 2 연동 및 데이터 수신 가이드

본 문서는 **Ubuntu 20.04/22.04** 및 **ROS 2 (Foxy/Humble)** 환경에서 **bluespace-ai 버전의 Xsens MTi 시리즈 IMU**를 연동하고 `/imu/data` 토픽을 생성하는 전 과정을 다룹니다.

---

## 🛠️ 1단계: 하드웨어 연결 및 시리얼 권한 설정

IMU는 USB-to-Serial 방식으로 연결되므로, 리눅스 시스템에서 해당 USB 포트에 접근할 수 있는 권한을 먼저 설정해야 합니다.

```bash
# 1. IMU가 연결된 USB 포트 이름 확인 (일반적으로 /dev/ttyUSB0 로 잡힙니다)
ls -l /dev/ttyUSB*

# 2. 시리얼 권한 영구 부여 (사용자를 dialout 그룹에 추가, 재부팅 후 적용)
sudo usermod -aG dialout $USER

# 3. 즉시 권한 부여 (지금 바로 테스트하고 싶을 때)
sudo chmod 777 /dev/ttyUSB0
```

---

## 📥 2단계: 드라이버 다운로드 (bluespace-ai 버전)

기존에 성공적으로 사용하셨던 `bluespace-ai` 레포지토리를 워크스페이스에 클론합니다.

```bash
cd ~/ros2_ws/src
git clone https://github.com/bluespace-ai/bluespace_ai_xsens_ros_mti_driver.git

---

## ⚙️ 3단계: 핵심 파라미터 설정 (포트 및 통신 속도)

장비가 연결된 포트와 통신 속도(Baudrate)가 드라이버 설정과 일치해야 정상적으로 데이터를 읽어올 수 있습니다.

`nano ~/ros2_ws/src/bluespace_ai_xsens_ros_mti_driver/param/xsens_mti_node.yaml` 명령으로 파일을 열어 아래 부분을 확인하고 수정합니다.

```yaml
xsens_mti_node:
  ros__parameters:
    # 1.1에서 확인한 포트 이름으로 정확히 기입
    port: "/dev/ttyUSB0"
    
    # 통신 속도 (장비 세팅에 따라 115200 또는 921600 입력)
    baudrate: 921600
    
    frame_id: "imu_link"
```
*(수정 후 저장: `Ctrl+O` → `Enter` → `Ctrl+X`)*

---

## 🔨 4단계: 워크스페이스 빌드 및 드라이버 가동

### 4.1 패키지 단독 빌드
레포지토리 이름(`bluespace_ai_xsens_ros_mti_driver`)과 실제 빌드되는 패키지 이름(`bluespace_ai_xsens_mti_driver`)이 살짝 다르니 아래 명령어를 그대로 복사해서 사용하세요.

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select bluespace_ai_xsens_mti_driver
source install/setup.bash
```

### 4.2 드라이버 실행
```bash
ros2 launch bluespace_ai_xsens_mti_driver xsens_mti_node.launch.py
```

---

## 📼 5단계: 토픽 수신 확인 및 데이터 녹화 (Rosbag)

드라이버가 켜진 상태에서 새로운 터미널을 열고 데이터가 정상적으로 쏟아지는지 검증합니다.

```bash
source ~/ros2_ws/install/setup.bash

# 1. 토픽 수신 주기 확인 (MTi 모델에 따라 보통 100Hz로 들어옵니다)
ros2 topic hz /imu/data

# 2. 실제 방향/가속도 데이터 눈으로 직접 확인해보기
ros2 topic echo /imu/data

# 3. 100Hz로 깔끔하게 들어온다면 녹화 시작!
cd ~/ros2_ws
ros2 bag record -o imu_dataset /imu/data
```
