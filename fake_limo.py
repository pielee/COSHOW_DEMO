#!/usr/bin/env python3
"""가짜 LIMO — 실기체 리허설용.

실기체 리모가 없는 자리에서 BT 가 요구하는 인터페이스만 그대로 제공한다.
BT 입장에서 리모는 딱 두 가지다.

  1. /{limo}/amcl_pose         (geometry_msgs/PoseWithCovarianceStamped)  대시보드가 구독
  2. /{limo}/navigate_to_pose  (nav2_msgs/NavigateToPose) LimoNavigateTo 가 호출

1번을 odom 이 아니라 amcl_pose 로 내는 이유: 실기 리모의 map 좌표 출처가 AMCL 이기
때문이다. /odom 은 "전원을 켠 자리" 기준이라 map 과 다르고 누적 오차로 흘러간다.
가짜가 실기와 다른 토픽을 내면, 가짜로 맞춰 둔 설정이 실기에서 그대로 깨진다.

⚠️ 실기 AMCL 과 다른 점: 진짜 AMCL 은 로봇이 멈춰 있으면 amcl_pose 를 새로 내지 않는다
(필터 갱신이 주행으로 촉발되므로). 이 가짜는 기본값(publish_when_idle=true)으로 정지
중에도 10 Hz 로 계속 낸다 — 화면이 비지 않아 디버깅이 편하기 때문이다. 토픽 나이로
리모 생존을 판정하는 설정을 시험하려면 publish_when_idle:=false 로 두고 돌려야
실기와 같은 조건이 된다.

특히 2번은 "액션"이라 `ros2 topic pub` 으로는 흉내낼 수 없다. LimoNavigateTo 는
wait_for_server() → send_goal → accepted → get_result → STATUS_SUCCEEDED 순서를
요구하고, 서버가 없으면 그 자리에서 영원히 RUNNING 으로 멈춘다.

동작: 목표 수신 → 수락 → travel_sec(기본 8초) 동안 odom 을 목표까지 선형 보간 → SUCCEEDED.
실제 주행이 아니라 "명령을 받아 이동을 마쳤다" 는 사실만 재현한다. 거리와 무관하게 같은 시간이 걸린다.

리모 이름과 시작 위치는 config.yaml 의 limos.base 에서 읽는다 (FAKE 의 다른 노드와 같은 파일).

실행:
  /usr/bin/python3 fake_limo.py
  /usr/bin/python3 fake_limo.py --ros-args -p travel_sec:=5.0
  /usr/bin/python3 fake_limo.py --ros-args -p publish_when_idle:=false  # 실기 AMCL 과 같은 조건
"""
import math
import threading
from pathlib import Path

import yaml

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose


class FakeLimo(Node):
    def __init__(self):
        super().__init__('fake_limo')
        self.declare_parameter('config', str(Path(__file__).resolve().parent / 'config.yaml'))
        # 목표까지 이동하는 데 걸리는 시간(초). 거리와 무관하게 고정.
        # 실제 리모의 주행 속도에 가깝게 두는 편이 BT 타이밍 검증에 유리하다.
        self.declare_parameter('travel_sec', 8.0)
        # 시작 위치. 비워 두면 config.yaml 의 limos.base 를 쓴다 (x1,y1,x2,y2,... 순서).
        self.declare_parameter('starts', [])
        # 정지 중에도 amcl_pose 를 계속 낼지. 실기 AMCL 과 같은 조건으로 시험하려면 false.
        self.declare_parameter('publish_when_idle', True)

        cfg = yaml.safe_load(Path(self.get_parameter('config').value).read_text(encoding='utf-8'))
        base = cfg['limos']['base']
        names = list(base)                      # config 에 적힌 순서 그대로
        self.travel_sec = float(self.get_parameter('travel_sec').value)
        starts = [float(v) for v in self.get_parameter('starts').value]
        if not starts:                          # 기본값: 각 리모의 기지에서 시작한다
            starts = [float(v) for nm in names for v in base[nm]]
        self.publish_when_idle = bool(self.get_parameter('publish_when_idle').value)
        self._moving = set()            # 지금 주행 중인 리모 (publish_when_idle=false 일 때 쓴다)

        cb = ReentrantCallbackGroup()   # odom 타이머와 액션 실행이 서로를 막지 않게
        self._lock = threading.Lock()
        self.state = {}                 # limo -> {'x','y','yaw'}
        self.pubs = {}
        self.servers = {}

        for i, n in enumerate(names):
            sx = float(starts[2 * i]) if 2 * i + 1 < len(starts) else 0.0
            sy = float(starts[2 * i + 1]) if 2 * i + 1 < len(starts) else 0.0
            self.state[n] = {'x': sx, 'y': sy, 'yaw': 0.0}
            self.pubs[n] = self.create_publisher(PoseWithCovarianceStamped, f'/{n}/amcl_pose', 10)
            self.servers[n] = ActionServer(
                self, NavigateToPose, f'/{n}/navigate_to_pose',
                execute_callback=self._make_execute(n),
                goal_callback=lambda _req: GoalResponse.ACCEPT,
                callback_group=cb)
            self.get_logger().info(f'[{n}] amcl_pose + navigate_to_pose 준비. 시작 ({sx:+.2f}, {sy:+.2f})')

        self.create_timer(0.1, self._pub_pose, callback_group=cb)   # 10 Hz

    # ---- amcl_pose 발행 ----
    def _pub_pose(self):
        now = self.get_clock().now().to_msg()
        with self._lock:
            snapshot = {n: dict(s) for n, s in self.state.items()}
            moving = set(self._moving)
        for n, s in snapshot.items():
            if not self.publish_when_idle and n not in moving:
                continue            # 실기 AMCL 처럼 정지 중에는 내지 않는다
            m = PoseWithCovarianceStamped()
            m.header.stamp = now
            m.header.frame_id = 'map'
            m.pose.pose.position.x = s['x']
            m.pose.pose.position.y = s['y']
            m.pose.pose.orientation.z = math.sin(s['yaw'] * 0.5)
            m.pose.pose.orientation.w = math.cos(s['yaw'] * 0.5)
            # AMCL 이 내는 것과 비슷한 수준의 분산. 전부 0 이면 "완벽히 안다" 는 뜻이 되어
            # 공분산을 보는 쪽(rviz 타원 등)이 오해한다.
            m.pose.covariance[0] = 0.05     # x
            m.pose.covariance[7] = 0.05     # y
            m.pose.covariance[35] = 0.02    # yaw
            self.pubs[n].publish(m)

    # ---- 액션 실행: travel_sec 동안 목표로 보간 ----
    def _make_execute(self, name):
        def execute(goal_handle):
            p = goal_handle.request.pose.pose.position
            q = goal_handle.request.pose.pose.orientation
            gx, gy = float(p.x), float(p.y)
            gyaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                s0 = dict(self.state[name])
            self.get_logger().info(
                f'[{name}] goal ({gx:+.2f}, {gy:+.2f}) <- ({s0["x"]:+.2f}, {s0["y"]:+.2f})')

            with self._lock:
                self._moving.add(name)
            steps = max(1, int(self.travel_sec * 10))
            rate = self.create_rate(10.0)
            fb = NavigateToPose.Feedback()
            for i in range(1, steps + 1):
                if not goal_handle.is_active or goal_handle.is_cancel_requested:
                    with self._lock:
                        self._moving.discard(name)
                    goal_handle.canceled()
                    self.get_logger().info(f'[{name}] 취소됨')
                    return NavigateToPose.Result()
                a = i / steps
                with self._lock:
                    self.state[name] = {
                        'x': s0['x'] + (gx - s0['x']) * a,
                        'y': s0['y'] + (gy - s0['y']) * a,
                        'yaw': s0['yaw'] + (gyaw - s0['yaw']) * a,
                    }
                    cur = self.state[name]
                fb.distance_remaining = float(math.hypot(gx - cur['x'], gy - cur['y']))
                goal_handle.publish_feedback(fb)
                rate.sleep()

            with self._lock:
                self._moving.discard(name)
            goal_handle.succeed()       # -> STATUS_SUCCEEDED. IsLimoAt 이 이걸 보고 통과한다
            self.get_logger().info(f'[{name}] 도착 ({gx:+.2f}, {gy:+.2f}) SUCCEEDED')
            return NavigateToPose.Result()
        return execute


def main():
    rclpy.init()
    node = FakeLimo()
    ex = MultiThreadedExecutor()        # rate.sleep() 이 executor 를 막지 않게
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
