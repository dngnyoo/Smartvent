# Teleop SLAM 가이드 - 수동 조종 기반 맵 생성

GoPiGo3 로봇을 키보드로 조종하면서 SLAM으로 맵을 생성하고 저장하는 절차입니다.

---

## 개요

**목적**: 키보드로 로봇을 직접 조종하면서 환경의 맵을 생성합니다.

**소요 시간**: 방 크기에 따라 5-15분

**필요한 것**:
- GoPiGo3 로봇 (전원 ON)
- SSH 연결 또는 모니터/키보드
- 맵핑할 공간 (방, 복도 등)

---

## 단계별 절차

### Step 0: 로봇 전원 켜기

1. GoPiGo3 배터리 연결 확인
2. Raspberry Pi 전원 ON
3. 부팅 완료까지 약 1분 대기
4. SSH 접속:
   ```bash
   ssh ubuntu@<로봇IP주소>
   # 예: ssh ubuntu@192.168.1.100
   ```

---

### Step 1: 로봇 시작 위치 설정

- 로봇을 방의 중앙 또는 시작하기 좋은 위치에 배치
- 로봇 전방에 최소 30cm 이상 공간 확보
- **이 위치가 맵의 원점(0,0)이 됩니다**

---

### Step 2: 기본 드라이버 실행 (터미널 1)

```bash
# ROS2 환경 설정 (사용하는 쉘에 따라 선택)
source ~/nav2_ws/install/setup.zsh   # zsh 사용시
source ~/nav2_ws/install/setup.bash  # bash 사용시

# 로봇 드라이버 + 센서 실행
ros2 launch gopigo3_driver gopigo3_bringup.launch.py
```

**정상 출력 확인**:
```
[INFO] [robot_state_publisher]: got segment base_footprint
[INFO] [gopigo3_driver]: GoPiGo3 driver started
[INFO] [bno055_imu]: BNO055 IMU initialized
[INFO] [rplidar_node]: RPLIDAR running
[INFO] [ekf_filter_node]: Starting...
```

**2-3초 대기** 후 다음 단계로

---

### Step 3: SLAM 실행 (터미널 2)

새 SSH 세션 열기:
```bash
ssh ubuntu@<로봇IP주소>
```

SLAM 실행:
```bash
# ROS2 환경 설정 (사용하는 쉘에 따라 선택)
source ~/nav2_ws/install/setup.zsh   # zsh 사용시
source ~/nav2_ws/install/setup.bash  # bash 사용시

ros2 launch gopigo3_driver slam.launch.py
```

**정상 출력 확인**:
```
[INFO] [slam_toolbox]: Waiting for scan...
[INFO] [slam_toolbox]: Got scan, starting SLAM
```

---

### Step 4: Teleop (키보드 조종) 실행 (터미널 3)

새 SSH 세션 열기:
```bash
ssh ubuntu@<로봇IP주소>
```

키보드 조종 시작:
```bash
# ROS2 환경 설정 (사용하는 쉘에 따라 선택)
source ~/nav2_ws/install/setup.zsh   # zsh 사용시
source ~/nav2_ws/install/setup.bash  # bash 사용시

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**조종 키**:
```
   u    i    o
   j    k    l
   m    ,    .

i : 전진
, : 후진
j : 좌회전
l : 우회전
k : 정지
u : 전진 + 좌회전
o : 전진 + 우회전
q/z : 속도 증가/감소
```

**권장 속도 설정**:
- `q` 또는 `z`로 속도 조절
- 선속도(linear): 0.1 ~ 0.15 m/s
- 각속도(angular): 0.3 ~ 0.5 rad/s

---

### Step 5: 맵핑 수행

**맵핑 팁**:

1. **천천히 이동**: 빠르게 움직이면 SLAM이 흐릿해짐
2. **벽을 따라 이동**: 벽에서 30-50cm 거리 유지
3. **모든 영역 방문**: 방의 모든 구석을 돌아다님
4. **Loop Closure**: 시작점으로 돌아오기 (정확도 향상)
5. **회전은 제자리에서**: 돌 때는 멈추고 천천히 회전

**맵핑 순서 예시**:
```
1. 시작점에서 오른쪽 벽으로
2. 시계방향으로 벽을 따라 한 바퀴
3. 방 중앙의 가구 주변
4. 시작점으로 복귀
```

---

### Step 6: 맵 실시간 확인 (선택사항)

**방법 A: Foxglove Studio** (권장)
1. PC에서 Foxglove Studio 실행
2. `ws://<로봇IP>:8765` 연결
3. Map 패널 추가하여 `/map` 토픽 시각화

**방법 B: 터미널에서 토픽 확인**
```bash
# 터미널 4
ros2 topic echo /map --once | head -20
```

---

### Step 7: 맵 저장

맵핑이 완료되면 맵을 저장합니다.

**방법 A: slam_toolbox 서비스 (권장)**
```bash
# 터미널 4
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/ubuntu/maps/my_room'}"
```

이 명령은 두 파일을 생성합니다:
- `my_room.posegraph` - 포즈 그래프
- `my_room.data` - 맵 데이터

**방법 B: nav2_map_server (표준 맵 형식)**
```bash
# 저장 디렉토리 생성
mkdir -p ~/maps

# 맵 저장
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```

이 명령은 두 파일을 생성합니다:
- `my_room.pgm` - 맵 이미지
- `my_room.yaml` - 맵 메타데이터

---

### Step 8: 종료

각 터미널에서 `Ctrl+C`로 종료:

1. 터미널 3: teleop 종료
2. 터미널 2: SLAM 종료
3. 터미널 1: 드라이버 종료

---

## 저장된 맵 확인

```bash
# 맵 파일 확인
ls -la ~/maps/

# 맵 YAML 내용 확인
cat ~/maps/my_room.yaml
```

예상 출력:
```yaml
image: my_room.pgm
resolution: 0.050000
origin: [-5.000000, -5.000000, 0.000000]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

## 맵 품질 확인 체크리스트

저장된 맵이 좋은지 확인:

- [ ] 벽이 얇고 직선으로 표시됨 (두껍거나 이중선 = 나쁨)
- [ ] 모든 방/구역이 맵에 포함됨
- [ ] 시작점과 끝점이 일치 (Loop Closure 성공)
- [ ] 가구, 장애물이 맵에 표시됨
- [ ] 회색(unknown) 영역이 벽 바깥에만 있음

---

## 문제 해결

### 문제: 맵이 뒤틀림 (drift)
- **원인**: 너무 빠르게 이동 또는 회전
- **해결**: 더 천천히 이동, Loop Closure 수행

### 문제: 벽이 두 줄로 보임
- **원인**: 오도메트리 드리프트
- **해결**: 시작점으로 돌아가서 Loop Closure

### 문제: 맵에 빈 공간이 있음
- **원인**: 해당 영역을 방문하지 않음
- **해결**: 빈 영역 근처로 이동하여 스캔

### 문제: SLAM이 시작되지 않음
- **원인**: LiDAR 데이터 없음
- **해결**:
  ```bash
  ros2 topic echo /scan --once
  # 데이터가 없으면 LiDAR 연결 확인
  ```

---

## 빠른 참조 명령어

> **참고**: 각 터미널에서 먼저 환경 설정 필요
> - zsh: `source ~/nav2_ws/install/setup.zsh`
> - bash: `source ~/nav2_ws/install/setup.bash`

```bash
# 터미널 1: 로봇 드라이버
ros2 launch gopigo3_driver gopigo3_bringup.launch.py

# 터미널 2: SLAM
ros2 launch gopigo3_driver slam.launch.py

# 터미널 3: 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 터미널 4: 맵 저장 (방법 B)
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_room
```
