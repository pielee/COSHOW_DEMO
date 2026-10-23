#!/bin/bash
# 메시지 패키지 2개를 빌드한다. 처음 한 번만 하면 된다.
#   ./build_interfaces.sh
#
#   coshow_interfaces     MarkerDetections — 우리가 만든 것. 다른 데 없다.
#   crazyflie_interfaces  Takeoff/GoTo/Land/Arm, Status, LogDataGeneric
#                         crazyswarm2 에 들어 있는 것과 같다. 이미 crazyswarm2 를
#                         쓰고 있으면 빌드하지 말고 그쪽을 source 하는 편이 낫다
#                         (같은 이름의 패키지가 둘이 되면 어느 쪽이 쓰이는지 헷갈린다).
set -eo pipefail
set +u
cd "$(dirname "$0")/interfaces"

# 가상환경이 켜져 있으면 colcon 이 그 python 을 쓰다가 catkin_pkg 를 못 찾고 죽는다.
# 빌드하는 동안만 벗겨낸다.
if [ -n "$VIRTUAL_ENV" ]; then
    echo "가상환경($VIRTUAL_ENV)을 빌드에서 제외합니다."
    PATH=$(echo "$PATH" | tr ':' '\n' | grep -v "^$VIRTUAL_ENV/bin$" | paste -sd:)
    unset VIRTUAL_ENV PYTHONHOME
    export PATH
fi

source /opt/ros/humble/setup.bash

SKIP=""
if /usr/bin/python3 -c 'import crazyflie_interfaces.srv' 2>/dev/null; then
    echo "crazyflie_interfaces 가 이미 환경에 있습니다 — 빌드에서 제외합니다."
    echo "  (그래도 빌드하려면: colcon build  를 직접 실행)"
    SKIP="--packages-skip crazyflie_interfaces"
fi

colcon build $SKIP --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
echo
echo "완료. 이제 터미널마다:  source ~/FAKE/setup_env.sh"
