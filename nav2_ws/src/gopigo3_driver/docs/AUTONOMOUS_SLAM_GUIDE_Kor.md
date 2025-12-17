# Autonomous SLAM 가이드 - 자율 주행 기반 맵 생성

GoPiGo3 로봇이 스스로 환경을 탐색하면서 맵을 생성하는 절차입니다.

---

## 개요

**목적**: 로봇이 자율적으로 미탐색 영역(frontier)을 찾아 이동하며 맵을 생성합니다.

**동작 방식**:
1. **Frontier Exploration**: explore_lite가 미탐색 영역 경계를 찾아 이동
2. **Coverage Pattern**: 탐색이 멈추면 지그재그 패턴으로 추가 커버리지
3. **자동 장애물 회피**: Nav2가 경로 계획 및 회피 담당

**소요 시간**: 방 크기에 따라 10-30분 (자동)

**필요한 것**:
- GoPiGo3 로봇 (전원 ON)
- SSH 연결
- 맵핑할 공간 (장애물이 없는 바닥)

---

## 사전 준비

### 공간 준비
- [ ] 바닥에 전선, 신발 등 작은 장애물 치우기
- [ ] 문 열어두기 (탐색할 공간 확보)
- [ ] 로봇이 빠질 수 있는 틈새 막기

### 로봇 배치
- 방 중앙 또는 열린 공간에 배치
- 로봇 전방 1m 이상 공간 확보
- **이 위치가 맵의 원점(0,0)이자 복귀 지점**

---

## 단계별 절차

### Step 0: 로봇 전원 켜기

1. GoPiGo3 배터리 연결 확인
2. Raspberry Pi 전원 ON
3. 부팅 완료까지 약 1분 대기
4. SSH 접속:
   ```bash
   ssh ubuntu@<로봇IP주소>
   ```

---

### Step 1: 자율 SLAM 실행 (터미널 1)

```bash
# ROS2 환경 설정 (사용하는 쉘에 따라 선택)
source ~/nav2_ws/install/setup.zsh   # zsh 사용시
source ~/nav2_ws/install/setup.bash  # bash 사용시

# 자율 탐색 + SLAM + Nav2 한번에 실행
ros2 launch gopigo3_driver autonomous_slam.launch.py
```

**시작 순서** (자동으로 진행됨):
```
0초:  robot_state_publisher, bno055_imu 시작
2초:  gopigo3_driver 시작 (모터/인코더)
즉시: slam_toolbox, Nav2 서버들 시작
10초: explore_lite 시작 (frontier 탐색)
15초: coverage_explorer 시작 (백업 탐색)
```

**정상 출력 확인**:
```
[INFO] [robot_state_publisher]: got segment base_footprint
[INFO] [gopigo3_driver]: GoPiGo3 driver started
[INFO] [bno055_imu]: BNO055 IMU initialized
[INFO] [rplidar_node]: RPLIDAR running
[INFO] [slam_toolbox]: Waiting for scan...
[INFO] [controller_server]: Creating controller server
[INFO] [planner_server]: Creating planner server
[INFO] [lifecycle_manager]: All nodes have been activated
[INFO] [explore_node]: Exploring...
[INFO] [coverage_explorer]: Coverage Explorer ready!
```

---

### Step 2: 탐색 모니터링

로봇이 자동으로 탐색을 시작합니다. 로그를 관찰하세요:

**정상 탐색 중 로그**:
```
[INFO] [explore_node]: Sending goal to frontier at (x, y)
[INFO] [controller_server]: Received new goal
[INFO] [bt_navigator]: Begin navigating...
```

**커버리지 모드 전환 로그**:
```
[INFO] [coverage_explorer]: No progress for 30.0s, switching to coverage mode
[INFO] [coverage_explorer]: Navigating to start position...
[INFO] [coverage_explorer]: Returned to start, generating coverage pattern
[INFO] [coverage_explorer]: Coverage waypoint 1/24: (0.50, 0.30)
```

---

### Step 3: 실시간 확인 (선택사항)

**방법 A: Foxglove Studio** (PC에서)
1. Foxglove Studio 실행
2. `ws://<로봇IP>:8765` 연결
3. 추가할 패널:
   - Map (`/map`)
   - LaserScan (`/scan`)
   - Path (`/plan`)
   - RobotModel

**방법 B: 토픽 모니터링** (터미널 2)
```bash
ssh ubuntu@<로봇IP주소>

# ROS2 환경 설정 (사용하는 쉘에 따라 선택)
source ~/nav2_ws/install/setup.zsh   # zsh 사용시
source ~/nav2_ws/install/setup.bash  # bash 사용시

# 현재 탐색 상태 확인
ros2 topic echo /explore/frontiers --once

# 로봇 위치 확인
ros2 topic echo /odometry/filtered --once | grep -A3 position
```

---

### Step 4: 탐색 완료 대기

탐색이 완료되면 다음 메시지가 나타납니다:

**Frontier 탐색 완료**:
```
[INFO] [explore_node]: All frontiers traversed/tried out, stopping exploration
```

**Coverage 탐색 완료**:
```
[INFO] [coverage_explorer]: Coverage pass 1 completed
[INFO] [coverage_explorer]: Coverage pass 2 completed
[INFO] [coverage_explorer]: All coverage passes completed!
```

---

### Step 5: 맵 저장

탐색이 완료되면 맵을 저장합니다.

**터미널 2에서 실행**:
```bash
ssh ubuntu@<로봇IP주소>

# ROS2 환경 설정 (사용하는 쉘에 따라 선택)
source ~/nav2_ws/install/setup.zsh   # zsh 사용시
source ~/nav2_ws/install/setup.bash  # bash 사용시

# 저장 디렉토리 생성
mkdir -p ~/maps

# 방법 A: slam_toolbox 서비스 (권장)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/living_room'}"

# 방법 B: nav2_map_server (표준 형식)
ros2 run nav2_map_server map_saver_cli -f ~/maps/living_room
```

---

### Step 6: 종료

터미널 1에서 `Ctrl+C`를 눌러 모든 노드를 한번에 종료합니다.

---

## 탐색 동작 설명

### Phase 1: Frontier Exploration (explore_lite)

```
┌─────────────────────────────────┐
│  ███░░░░░░░░░░░░░░░███          │
│  ███       ?        ███          │
│  ███    ┌───┐       ███          │
│         │ R │──→ Frontier        │
│         └───┘                    │
│  ███                ███          │
│  ███████████████████             │
└─────────────────────────────────┘
███ = 벽 (장애물)
░░░ = 미탐색 영역 (Unknown)
?   = Frontier (탐색 경계)
R   = 로봇
```

- 알려진 영역과 미지 영역의 **경계(Frontier)**를 찾음
- 가장 가까운/유망한 Frontier로 이동
- 이동하면서 새로운 영역 스캔
- 모든 Frontier가 탐색되면 종료

### Phase 2: Coverage Pattern (coverage_explorer)

```
┌─────────────────────────────────┐
│  ███████████████████████████    │
│  ███  ←───────────────────      │
│  ███  ─────────────────→        │
│  ███  ←───────────────────      │
│  ███  ─────────────────→ R      │
│  ███████████████████████████    │
└─────────────────────────────────┘
```

- 30초간 진전이 없으면 활성화
- 시작 위치로 복귀
- 맵 경계 내에서 지그재그 패턴 생성
- 2회 반복하여 놓친 영역 커버

---

## 파라미터 조정

필요시 launch 파일에서 조정 가능:

```python
# autonomous_slam.launch.py 내 coverage_explorer 파라미터
'coverage_spacing': 0.5,    # 지그재그 간격 (m) - 작으면 더 촘촘
'coverage_margin': 0.3,     # 벽에서 거리 (m)
'coverage_passes': 2,       # 커버리지 반복 횟수
'frontier_timeout': 30.0,   # 전환 대기 시간 (초)
'goal_tolerance': 0.3       # 목표 도달 허용 오차 (m)
```

---

## 문제 해결

### 문제: 로봇이 움직이지 않음
```
[WARN] [controller_server]: No valid path found
```
**해결**:
- 로봇 주변에 공간이 충분한지 확인
- LiDAR가 장애물을 너무 가깝게 감지하는지 확인
- 로봇을 더 열린 공간으로 옮기고 재시작

### 문제: 탐색이 너무 일찍 멈춤
```
[INFO] [explore_node]: All frontiers traversed
```
(하지만 맵에 회색 영역이 남아있음)

**해결**:
- coverage_explorer가 자동으로 지그재그 패턴 시작
- 30초 기다리면 커버리지 모드로 전환됨

### 문제: 로봇이 장애물에 부딪힘
**해결**:
```bash
# nav2_params.yaml에서 inflation_radius 증가
inflation_radius: 0.35  # 0.4 이상으로 조정
```

### 문제: "Behavior Tree tick rate exceeded"
**해결**: Pi4 CPU 과부하 - 이미 최적화되어 있으나 심하면:
```bash
# 불필요한 프로세스 종료
sudo systemctl stop bluetooth
```

### 문제: SPI 오류
```
[ERROR] No SPI response
```
**해결**:
```bash
# 재시도 - 이미 2초 딜레이가 적용됨
# 계속 실패하면:
sudo pigpiod
# 재시작
```

---

## 빠른 참조 명령어

> **참고**: 먼저 환경 설정 필요
> - zsh: `source ~/nav2_ws/install/setup.zsh`
> - bash: `source ~/nav2_ws/install/setup.bash`

```bash
# 모든 것을 한번에 실행
ros2 launch gopigo3_driver autonomous_slam.launch.py

# 맵 저장 (탐색 완료 후)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

# 또는 slam_toolbox 방식
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/my_map'}"
```

---

## 탐색 완료 체크리스트

- [ ] 로봇이 자동으로 이동하기 시작함
- [ ] Foxglove에서 맵이 점점 커지는 것 확인
- [ ] "All frontiers traversed" 또는 "All coverage passes completed" 메시지 확인
- [ ] 맵 저장 완료
- [ ] 저장된 맵 파일 존재 확인 (`ls ~/maps/`)
