# FAKE — COSHOW 시나리오를 실기 없이 돌려보기

실제 Crazyflie·카메라 없이 COSHOW 임무 시나리오를 재현한다. 리모만 진짜(또는 별도 가짜)를
붙이면 전체 흐름이 끝까지 돈다.

```
fake_limo.py      ── 리모 2대인 척 (navigate_to_pose 액션 + amcl_pose)
fake_crazyflie.py ── 드론 4대인 척 (서비스 받고 pose 를 움직인다)
fake_vision.py    ── 카메라인 척 (마커 검출 + 영상·fps·stream_ok)
central_pc.py     ── BT 대신 시나리오를 순서대로 진행
config.yaml       ── 넷이 같이 읽는 설정
setup_env.sh      ── 터미널마다 source (워크스페이스 자동 탐색)
build_interfaces.sh ── 메시지 패키지 빌드 (처음 한 번)
interfaces/src/   ── coshow_interfaces · crazyflie_interfaces
```

실기 리모를 쓸 때는 `fake_limo.py` 대신 그 패키지를 띄우면 된다. 나머지는 그대로다.

## 먼저 필요한 것

ROS 2 Humble, 파이썬 패키지(`pyyaml`, `opencv-python`, `numpy`), 그리고 메시지 패키지 두 개.

| 패키지 | 쓰는 곳 | 어디 있나 |
| --- | --- | --- |
| `coshow_interfaces` | 마커 검출(`MarkerDetections`) | **이 폴더 안** (`interfaces/src/`) |
| `crazyflie_interfaces` | 드론 서비스(`Takeoff`/`GoTo`/`Land`/`Arm`)·상태(`Status`) | 이 폴더 안. crazyswarm2 를 이미 쓰면 그쪽 것 |
| `nav2_msgs` | 리모 액션 | ROS 2 기본 설치 |

### 처음 한 번 — 메시지 빌드

```bash
~/FAKE/build_interfaces.sh
```

`crazyswarm2` 가 이미 환경에 있으면 `crazyflie_interfaces` 는 건너뛴다 (같은 이름의
패키지가 둘이 되면 어느 쪽이 쓰이는지 헷갈리기 때문).

가상환경이 켜져 있으면 `colcon` 이 그 python 을 쓰다 `catkin_pkg` 를 못 찾고 죽는데,
스크립트가 빌드하는 동안만 벗겨낸다.

**터미널마다** 한 번 실행한다. 빼먹으면 `ModuleNotFoundError: No module named
'coshow_interfaces'` 가 난다.

```bash
source ~/FAKE/setup_env.sh
```

워크스페이스를 자동으로 찾아 source 하고, 모듈이 다 있는지 확인해 준다. 경로가 다르면
미리 알려 준다.

```bash
export COSHOW_WS=~/my_vision_ws
export CRAZYFLIE_WS=~/my_crazyswarm2_ws
source ~/FAKE/setup_env.sh
```

직접 하려면:

```bash
source /opt/ros/humble/setup.bash
source <coshow_interfaces 가 있는 워크스페이스>/install/setup.bash
source <crazyflie_interfaces 가 있는 워크스페이스>/install/setup.bash
export ROS_DOMAIN_ID=33        # 네 터미널과 대시보드가 같은 값이어야 한다
```

## 실행 — 터미널 4개

```bash
# 터미널마다 먼저: source ~/FAKE/setup_env.sh

# 1) 가짜 리모 (실기 리모 패키지를 쓸 거면 이것 대신 그 패키지를 띄운다)
/usr/bin/python3 fake_limo.py
/usr/bin/python3 fake_limo.py --ros-args -p publish_when_idle:=false  # 실기 AMCL 과 같은 조건

# 2) 가짜 드론 4대
/usr/bin/python3 fake_crazyflie.py

# 3) 가짜 카메라
/usr/bin/python3 fake_vision.py                          # 타겟 위치가 매번 랜덤
/usr/bin/python3 fake_vision.py --ros-args -p seed:=7    # 같은 자리에 두고 재현

# 4) 중앙 제어기 (맨 마지막)
/usr/bin/python3 central_pc.py
```

`python3` 가 가상환경을 가리키면 `yaml`·`rclpy` 를 못 찾는다. **항상 `/usr/bin/python3`**
로 실행한다 (`setup_env.sh` 가 `fpy` 별칭도 만들어 둔다).

## 시나리오

| | 단계 | 다음으로 넘어가는 조건 |
| --- | --- | --- |
| 1 | `limo_a` → 관측점 (0,0) | Nav2 액션 `SUCCEEDED` |
| 2 | 미션 드론 이륙 → 관측점 상공 | pose 가 반경 0.15 m 에 1 초 유지 |
| 3 | 미션 마커 11 인식 | `marker_detections` 수신 → `target_id = 1` |
| 4 | 미션 드론 복귀·착륙 ∥ `limo_a` 기지 복귀 | 기지 반경 + z ≤ 0.08 |
| 5 | 탐색 드론 3대 순차 출격 (3 초 간격) | 전부 이륙 완료 |
| 6 | 각자 구역을 x 로 왕복 | 타겟 마커 발견 → `finder` 확정 |
| 7 | `finder` 외 2대 복귀·착륙 | 둘 다 착륙 |
| 8 | `limo_b` → 마커 접근점 (x+0.1) | Nav2 `SUCCEEDED` |
| 9 | 도착 후 10 초 대기 | 구조 완료 |
| 10 | `finder` 복귀·착륙 ∥ `limo_b` 기지 복귀 | 완료 |

## 주고받는 인터페이스

### `fake_crazyflie.py`
| 방향 | 이름 | 타입 |
| --- | --- | --- |
| 받음 | `/{드론}/takeoff` `/go_to` `/land` `/arm` | `crazyflie_interfaces/srv/*` |
| 냄 | `/{드론}/pose` (10 Hz) | `geometry_msgs/msg/PoseStamped` |
| 냄 | `/{드론}/status` (10 Hz) | `crazyflie_interfaces/msg/Status` |
| 냄 | `/{드론}/kalman_variance` | `crazyflie_interfaces/msg/LogDataGeneric` |

### `fake_vision.py`
| 방향 | 이름 | 타입 |
| --- | --- | --- |
| 받음 | `/{드론}/pose` | `geometry_msgs/msg/PoseStamped` |
| 냄 | `/{드론}/marker_detections` (5 Hz) | `coshow_interfaces/msg/MarkerDetections` |
| 냄 | `/aideck/{드론}/image_annotated/compressed` | `sensor_msgs/msg/CompressedImage` |
| 냄 | `/aideck/{드론}/fps` (1 Hz) | `std_msgs/msg/Float32` |
| 냄 | `/aideck/{드론}/stream_ok` (1 Hz) | `std_msgs/msg/Bool` |

이름이 둘로 갈린 것은 실기와 같다. 검출 결과는 BT 가 쓰는 전역 이름(`/cf231/...`)으로,
영상·진단은 노드 네임스페이스(`/aideck/cf231/...`) 아래로 나간다.

### `central_pc.py`
| 방향 | 이름 | 타입 |
| --- | --- | --- |
| 보냄 | `/{드론}/takeoff` `/go_to` `/land` | `crazyflie_interfaces/srv/*` |
| 보냄 | `/{리모}/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| 받음 | `/{드론}/pose`, `/{드론}/marker_detections` | — |
| 냄 | `/coshow/mission_state` (latched) | `std_msgs/msg/String` (JSON) |
| 냄 | `/{로봇}/led` | `std_msgs/msg/String` |

LED 는 실기 BT 의 `SetLed` 와 같은 순서로 바뀐다 — 색이 바뀔 때만 발행한다.

| 시점 | 대상 | 색 |
| --- | --- | --- |
| 미션 마커 확정 | 전 기체 | `blue` |
| 타겟 마커 발견 | `finder` | `red` |
| 구조 리모 도착 | `finder` | `green` |
| 구조 완료 | 전 기체 | `off` |

`/coshow/mission_state` 의 `phase` 는 실기 BT 의 `_derive_phase` 와 같은 규칙으로 계산한다.

```
observe -> handover -> search -> capture -> rescue_dispatch -> rescue -> return -> done
```

## 설정 — `config.yaml`

세 프로그램이 같은 파일을 읽는다. 좌표는 `map`, 단위는 m·초.

- `drones.base` — 이륙·착륙 자리
- `drones.altitude` — 단계별 고도 (`takeoff` / `search` / `capture` / `return` / `observe`)
- `field.observe_point` — 미션 마커 자리
- `field.zones` — 탐색 구역 (x 구간을 왕복, 레인 중앙 y 고정)
- `markers.detect_radius` — 드론이 이 안에 들어오면 타겟 마커가 보인다
- `markers.mission_delay_max` — 미션 드론 도착 후 인식까지의 최대 지연 (랜덤)
- `limos.base` — 리모 기지. `fake_limo.py` 의 시작 위치이기도 하다
- `timing.*` — 이동 시간, 도착 판정 기준, 구조 대기 시간

다른 파일을 쓰려면:
```bash
/usr/bin/python3 central_pc.py --ros-args -p config:=/path/to/other.yaml
```

## 실기와 다른 점 (의도적으로 단순화했다)

- **비행 조건은 무조건 통과.** `status` 의 무장·`CAN_FLY` 비트와 칼만 분산을 정상값으로 고정한다.
  배터리 소모·추력 부족·통신 두절·기체 고장은 재현하지 않는다.
- **드론 이동은 선형 보간.** 실제 궤적·가속도·바람이 없다. 거리와 무관하게 명령한
  `duration` 만큼 걸린다.
- **마커 검출에 오차가 없다.** 실기는 크기 기반 역투영이라 거리 오차가 섞인다.
  `--ros-args -p noise:=0.05` 로 오차를 줄 수 있다.
- **`land` 는 고도만 0 으로 내린다.** 실기처럼 모터가 꺼지지는 않는다.
- **사전점검(PreflightReady)이 없다.** 실기 BT 는 칼만 수렴·초기위치·슈퍼바이저·카메라를
  확인한 뒤에야 첫 명령을 낸다. 여기서는 비행 가능으로 가정하므로 그 관문이 없다.
- **일부 토픽은 내지 않는다.** 대시보드도 BT 도 쓰지 않는 것들이다 —
  crazyswarm2 의 `robot_description`·`/tf`, 영상 노드의 `image_raw`·`image_annotated`
  (비압축)·`markers_text`.
- **`/coshow/estop` 을 구독하지 않는다.** 현재 실기 BT(`feat/BT`)도 구독하지 않으므로
  같게 맞춘 것이다. 대시보드의 비상 정지 버튼은 양쪽 모두에서 BT 에 닿지 않는다.
- **서비스 응답은 실기와 똑같이 비어 있다.** 명령 성공 여부를 응답으로 알 수 없고
  `pose` 로 판정해야 하는 것까지 같게 맞췄다.

## 확인용 명령

```bash
ros2 topic hz /cf230/pose                       # 10 Hz 가 정상
ros2 topic echo /cf230/marker_detections --once # 검출 내용
ros2 topic echo /coshow/mission_state --once    # 진행 상황 (phase·finder·led)
ros2 topic echo /cf230/led --once               # LED 색
ros2 topic hz /aideck/cf230/fps                 # 1 Hz 가 정상
ros2 action list | grep limo                    # 리모 액션 서버 생존
ros2 service list | grep cf230                  # 드론 서비스 목록
```

진행이 멈추면 `central_pc` 콘솔의 `=== A -> B ===` 로그에서 어느 단계에 머물러 있는지
먼저 확인한다. `limo_observe` 에서 멈추면 리모 액션 서버가 없다는 뜻이다.

## 대시보드로 볼 때

```bash
cd ~/coshow_feat_dashboard_ui/dashboard && ./run.sh
#   참관자 화면 → http://localhost:8080/visitor.html
```

`ROS_DOMAIN_ID` 가 네 노드와 대시보드에서 모두 같아야 한다.

**참관자 화면은 전부 정상으로 나온다** — 지도·드론 위치·탐색 레인·카메라 영상·국면·LED.

운영자 화면(`admin.html`)은 대상이 아니다. 실제 하드웨어 핑과 사전점검 토픽을 요구하는
체크리스트가 있어 가짜로는 채울 수 없다.

## 포함된 외부 코드

`interfaces/src/crazyflie_interfaces` 는 [crazyswarm2](https://github.com/IMRCLab/crazyswarm2)
의 메시지 패키지를 그대로 가져온 것이다 (MIT, Wolfgang Hönig · Kimberly N. McGuire).
crazyswarm2 를 설치하지 않고도 이 데모를 돌릴 수 있게 하려고 넣었다. 이미 crazyswarm2 를
쓰고 있다면 `build_interfaces.sh` 가 이 패키지를 건너뛰고 그쪽을 쓴다.
