# FAKE 실행 환경 — 터미널마다 한 번 source 한다.
#   source ~/FAKE/setup_env.sh
#
# 필요한 것: ROS 2 Humble + coshow_interfaces + crazyflie_interfaces
# 워크스페이스 위치가 다르면 아래 두 변수를 바꾸거나, 미리 환경변수로 넣어 둔다.
#   export COSHOW_WS=~/my_vision_ws
#   export CRAZYFLIE_WS=~/my_crazyswarm2_ws

set +u    # ROS setup.bash 는 미설정 변수를 건드린다. set -u 면 여기서 죽는다.

source /opt/ros/humble/setup.bash

# FAKE 자체 워크스페이스를 가장 먼저 본다 (build_interfaces.sh 로 빌드한 것).
_FAKE_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/interfaces"

# coshow_interfaces (MarkerDetections)
for ws in "${COSHOW_WS:-}" "$_FAKE_WS" \
          ~/aideck_realtime_v42_ws/vision_ws \
          ~/aideck_realtime_v43_ws/vision_ws \
          ~/aideck_realtime_v44_ws/vision_ws \
          ~/coshow_feat_dashboard_ui/ros2_ws; do
    if [ -n "$ws" ] && [ -f "$ws/install/setup.bash" ]; then
        source "$ws/install/setup.bash"
        echo "  coshow_interfaces  <- $ws"
        break
    fi
done

# crazyflie_interfaces (Takeoff/GoTo/Land/Arm, Status, LogDataGeneric)
for ws in "${CRAZYFLIE_WS:-}" \
          ~/crazyswarm2_ws \
          "$_FAKE_WS" \
          ~/coshow_feat_dashboard_ui/ros2_ws \
          ~/COSHOW_Sim/ros2_ws; do
    if [ -n "$ws" ] && [ -f "$ws/install/setup.bash" ]; then
        source "$ws/install/setup.bash"
        echo "  crazyflie_interfaces <- $ws"
        break
    fi
done

# 네 터미널과 대시보드가 모두 같은 값이어야 서로 보인다.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77}
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 가상환경 python 은 yaml·rclpy 를 못 찾는다. 시스템 python 을 쓴다.
alias fpy='/usr/bin/python3'

# 임포트가 되는지 바로 확인한다 — 안 되면 여기서 알 수 있다.
/usr/bin/python3 - <<'PY'
import importlib, sys
bad = []
for m in ('rclpy', 'yaml', 'cv2', 'numpy',
          'coshow_interfaces.msg', 'crazyflie_interfaces.srv', 'nav2_msgs.action'):
    try:
        importlib.import_module(m)
    except Exception:
        bad.append(m)
if bad:
    print('  [실패] 못 찾는 모듈:', ', '.join(bad))
    if any('interfaces' in m for m in bad):
        print('         메시지 패키지를 먼저 빌드하세요:  ~/FAKE/build_interfaces.sh')
    else:
        print('         pip 로 설치하세요 (cv2 -> opencv-python, yaml -> pyyaml)')
else:
    print('  [준비 완료] 모듈 전부 확인됨')
PY
