# ROS 2 기반 자율 주차 시스템

**2391007 인공지능학과 김민정 · 2391017 인공지능학과 이윤지**

주차 슬롯 유형(일반/장애인 전용) 분류와 사용자 자격 기반 필터링을 자율 주행·주차 파이프라인에 통합한 시뮬레이션입니다.

---

## 환경

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 22.04 (WSL2) |
| ROS | ROS 2 Humble |
| 시뮬레이터 | Gazebo Classic 11 |
| Python | 3.10 |
| 비전 (선택) | YOLOv8 (ultralytics) |

---

## 패키지 구조

```
IL/
├── autonomous_parking/          # ROS 2 패키지 (소스)
│   ├── config/
│   │   └── slot_metadata.yaml   # 슬롯 좌표·유형·점유 메타데이터
│   ├── launch/
│   │   ├── parking_sim.launch.py    # 메인 런치 (전체 파이프라인)
│   │   ├── world_only.launch.py     # Gazebo 월드만 띄울 때
│   │   └── parking_world.launch.py  # (구버전) TurtleBot3용 — 미사용
│   ├── models/
│   │   ├── ego_hatchback/       # 자율주차 차량 모델 (카메라·LiDAR·IMU 장착)
│   │   ├── hatchback/           # 주차된 일반 차량
│   │   ├── hatchback_red/
│   │   ├── hatchback_blue/
│   │   └── wheel_stopper/
│   ├── scripts/
│   │   ├── parking_node.py      # 주차 컨트롤러 (상태 기계)
│   │   ├── vision_node.py       # 슬롯 인식 (YOLOv8 or YAML fallback)
│   │   └── decision_node.py     # 사용자 자격 기반 슬롯 선택
│   ├── worlds/
│   │   └── parking_lot.world    # Gazebo 주차장 월드
│   ├── media/                   # ISA 마커 텍스처
│   ├── CMakeLists.txt
│   └── package.xml
├── build/                       # colcon 빌드 산출물 (git 제외)
├── install/                     # colcon 설치 산출물 (git 제외)
├── log/                         # 빌드 로그 (git 제외)
└── .gitignore
```

---

## 빌드

```bash
cd ~/IL
colcon build --packages-select autonomous_parking
```

> 전체 패키지를 새로 빌드하려면 `--packages-select` 옵션을 제거합니다.

`.bashrc`에 아래 줄이 있으면 새 터미널마다 자동으로 소싱됩니다:

```bash
# ~/.bashrc 마지막 줄 확인
source ~/IL/install/setup.bash
```

수동으로 소싱할 경우:

```bash
source ~/IL/install/setup.bash
```

---

## 런치 파일

### 1. `parking_sim.launch.py` — **메인 (전체 파이프라인)**

Gazebo + 세 개의 ROS 노드(vision / decision / parking)를 한 번에 실행합니다.

```bash
# 일반 사용자 (장애인 슬롯 배정 제외)
ros2 launch autonomous_parking parking_sim.launch.py user_credential:=general

# 장애인 사용자 (장애인 슬롯 우선 배정)
ros2 launch autonomous_parking parking_sim.launch.py user_credential:=handicapped

# YOLOv8 모델이 있는 경우 (없으면 YAML ground truth 사용)
ros2 launch autonomous_parking parking_sim.launch.py \
    user_credential:=general \
    yolo_model:=/path/to/best.pt
```

**런치 인자**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `user_credential` | `general` | 주차 자격: `general` 또는 `handicapped` |
| `yolo_model` | `""` (비어있음) | YOLOv8 `.pt` 파일 경로. 비어있으면 YAML 사용 |

---

### 2. `world_only.launch.py` — **Gazebo 월드만 실행**

노드 없이 주차장 시각화만 확인할 때 사용합니다.

```bash
ros2 launch autonomous_parking world_only.launch.py
```

---

### 3. `parking_world.launch.py` — **구버전 / 미사용**

초기 개발 당시 TurtleBot3를 사용하던 런치 파일입니다. 현재 ego_hatchback 기반으로 전환되어 사용하지 않습니다.

---

## 노드 설명

### `parking_node.py` — 주차 컨트롤러

**상태 기계 흐름:**

```
INIT → ENTER_LOT → OBS_DRIVE → OBS_WAIT → LANE_DRIVE → TURN_TO_SLOT → X_ALIGN → PARK_FORWARD → DONE
```

| 상태 | 설명 |
|------|------|
| `INIT` | 첫 odom 수신 대기 |
| `ENTER_LOT` | 주차장 입구(-10.5, 0.0)까지 이동 |
| `OBS_DRIVE` | 관찰 포인트(obs_1, obs_2)까지 이동 |
| `OBS_WAIT` | vision_node 스캔 후 decision_node의 슬롯 선택 대기 (타임아웃 10초) |
| `LANE_DRIVE` | y=0 레인을 따라 목표 슬롯 x좌표까지 이동 |
| `TURN_TO_SLOT` | 슬롯 반대 방향으로 회전 (후방주차 준비) |
| `X_ALIGN` | 회전 후 슬롯 중심 x에 정렬 |
| `PARK_FORWARD` | **후진**으로 슬롯 진입; 목표 깊이 도달 + ROI 내부 진입 시 완료 |
| `DONE` | 정지 유지 |

**토픽**

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| Subscribe | `/ego/odom` | `nav_msgs/Odometry` | 차량 위치·자세 |
| Subscribe | `/ego/scan` | `sensor_msgs/LaserScan` | LiDAR 스캔 |
| Subscribe | `/parking/target_slot` | `std_msgs/String` | decision_node가 선택한 슬롯 ID |
| Subscribe | `/parking/no_slot` | `std_msgs/Bool` | 가능한 슬롯 없음 알림 |
| Publish | `/ego/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 |
| Publish | `/parking/obs_trigger` | `std_msgs/Bool` | 관찰 포인트 도달 → vision 스캔 요청 |

---

### `vision_node.py` — 슬롯 인식

관찰 트리거를 받으면 카메라 이미지를 처리해 슬롯 상태를 퍼블리시합니다.

- YOLOv8 모델(`.pt`)이 있으면 실행, 없으면 `slot_metadata.yaml` 그대로 사용
- 검출 클래스: `0=isa_mark` (장애인 마크), `1=vehicle` (차량)

**토픽**

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| Subscribe | `/ego/camera/image_raw` | `sensor_msgs/Image` | 카메라 이미지 |
| Subscribe | `/ego/camera/camera_info` | `sensor_msgs/CameraInfo` | 카메라 내부 파라미터 |
| Subscribe | `/parking/obs_trigger` | `std_msgs/Bool` | 스캔 트리거 |
| Publish | `/parking/slot_states` | `std_msgs/String` (JSON) | 전체 슬롯 상태 목록 |

**파라미터**

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `yolo_model` | `""` | YOLOv8 `.pt` 경로 (비어있으면 YAML fallback) |

---

### `decision_node.py` — 슬롯 선택 (의사결정)

슬롯 상태를 받아 사용자 자격에 맞는 최적 슬롯을 선택합니다.

**의사결정 순서:**
1. 빈 슬롯 필터링
2. 사용자 자격 적용 (`general` → 장애인 슬롯 제외, `handicapped` → 전체 허용)
3. 장애인 사용자는 장애인 슬롯 우선 배정
4. 주차장 입구와의 거리 기준 정렬 → 가장 가까운 슬롯 선택

**토픽**

| 방향 | 토픽 | 타입 | 설명 |
|------|------|------|------|
| Subscribe | `/parking/slot_states` | `std_msgs/String` (JSON) | vision_node 슬롯 상태 |
| Publish | `/parking/target_slot` | `std_msgs/String` | 선택된 슬롯 ID |
| Publish | `/parking/no_slot` | `std_msgs/Bool` | 유효한 슬롯 없음 |

**파라미터**

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `user_credential` | `general` | 주차 자격: `general` 또는 `handicapped` |

---

## 센서 구성 (ego_hatchback)

| 센서 | 위치 (chassis 기준) | 토픽 |
|------|-------------------|------|
| 2D LiDAR | 지붕 (z=1.5m), 360° | `/ego/scan` |
| 전방 카메라 | 앞범퍼 (x=2.0, z=1.0m), 20° 하향 틸트 | `/ego/camera/image_raw` |
| IMU | 차체 중앙 (z=0.5m) | `/ego/imu` |

---

## 슬롯 구성 (`slot_metadata.yaml`)

- 총 16개 슬롯: **A1–A8** (북쪽), **B1–B8** (남쪽)
- **장애인 전용**: A1, A2, B1, B2 (ISA 마크 포함)
- **일반**: A3–A8, B3–B8
- **사전 점유**: A4, A7, B3, B6

```
     A1   A2   A3   A4   A7   A8
     HC   HC   GN  [GN] GN  [GN] GN   GN
 ←━━━━━━━━━━━━━━━ 레인 (y≈0) ━━━━━━━━━━━━━━━→ 입구(-12,0)
     HC   HC   GN   GN  [GN] GN  [GN] GN
     B1   B2   B3   B4   B5   B6   B7   B8
                   HC = 장애인 전용, GN = 일반, [...] = 점유
```

---

## YOLO 학습 (김민정 담당)

```bash
# 1. 의존성 설치
pip install ultralytics opencv-python-headless

# 2. Gazebo에서 이미지 수집
ros2 bag record /ego/camera/image_raw

# 3. 학습 (클래스: 0=isa_mark, 1=vehicle)
yolo train model=yolov8n.pt data=dataset.yaml epochs=50 imgsz=640

# 4. 모델 연결
ros2 launch autonomous_parking parking_sim.launch.py \
    user_credential:=general \
    yolo_model:=runs/detect/train/weights/best.pt
```

---

## 자주 쓰는 명령

```bash
# Gazebo가 이미 떠 있을 때 강제 종료
killall gzserver gzclient

# 빌드 후 소싱
cd ~/IL && colcon build --packages-select autonomous_parking

# 토픽 실시간 확인
ros2 topic echo /parking/target_slot
ros2 topic echo /parking/slot_states
ros2 topic echo /ego/cmd_vel
```
