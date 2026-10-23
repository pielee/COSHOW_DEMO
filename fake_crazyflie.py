#!/usr/bin/env python3
"""가짜 Crazyflie — 드론 4대를 한 노드가 흉내낸다.

실기 crazyswarm2 가 내주는 것 중, 중앙 제어기(central_pc.py)가 실제로 쓰는 것만 제공한다.

  서비스 (받는다)            /{드론}/takeoff   crazyflie_interfaces/srv/Takeoff
                             /{드론}/go_to     crazyflie_interfaces/srv/GoTo
                             /{드론}/land      crazyflie_interfaces/srv/Land
                             /{드론}/arm       crazyflie_interfaces/srv/Arm
  토픽 (낸다)                /{드론}/pose             geometry_msgs/PoseStamped
                             /{드론}/status           crazyflie_interfaces/msg/Status
                             /{드론}/kalman_variance  crazyflie_interfaces/msg/LogDataGeneric

동작: 명령을 받으면 request.duration 동안 현재 위치에서 목표까지 선형 보간해 pose 를 움직인다.
실제 비행이 아니라 "명령을 받아 그만큼 이동했다" 는 사실만 재현한다.

비행 가능 조건은 전부 통과로 고정한다 (status 의 무장·CAN_FLY 비트, 수렴한 칼만 분산).
배터리 소모·추력 부족·통신 두절 같은 실패는 흉내내지 않는다.

실기와 다른 점
  - 서비스 응답은 실기와 똑같이 비어 있다. 명령이 성공했는지는 응답으로 알 수 없고,
    pose 를 보고 판정해야 한다 (실기 crazyswarm2 도 마찬가지다).
  - land 는 고도만 0 으로 내린다. 실기처럼 모터가 꺼지지는 않는다.

실행:
  python3 fake_crazyflie.py
  python3 fake_crazyflie.py --ros-args -p config:=/path/to/config.yaml
"""
import math
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg  # noqa: F401  (요청 필드 읽기용)
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import LogDataGeneric, Status
from crazyflie_interfaces.srv import Arm, GoTo, Land, Takeoff

# status 비트 (crazyflie_interfaces/msg/Status.msg 의 상수와 같다)
CAN_BE_ARMED, IS_ARMED, AUTO_ARM, CAN_FLY, IS_FLYING = 1, 2, 4, 8, 16
PM_STATE_BATTERY = 0
GROUND_Z = 0.0          # 지상 고도
AIRBORNE_Z = 0.25       # 이 위면 IS_FLYING 을 세운다


def _secs(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) / 1e9


class FakeCrazyflie(Node):
    def __init__(self):
        super().__init__('fake_crazyflie')
        self.declare_parameter('config', str(Path(__file__).resolve().parent / 'config.yaml'))
        cfg = yaml.safe_load(Path(self.get_parameter('config').value).read_text(encoding='utf-8'))
        d = cfg['drones']
        self.names = [d['mission']] + list(d['searchers'])
        self.base = {n: [float(v) for v in d['base'][n]] for n in self.names}

        self._lock = threading.Lock()
        # 기체별 상태. 명령이 없으면 target 이 None 이고 pose 는 그 자리에 멈춘다.
        self.state = {n: {'x': self.base[n][0], 'y': self.base[n][1], 'z': GROUND_Z,
                          'from': None, 'to': None, 't0': 0.0, 'dur': 0.0,
                          'armed': True, 'yaw': 0.0} for n in self.names}

        cb = ReentrantCallbackGroup()
        self.pose_pub, self.status_pub, self.kal_pub = {}, {}, {}
        for n in self.names:
            self.pose_pub[n] = self.create_publisher(PoseStamped, f'/{n}/pose', 10)
            self.status_pub[n] = self.create_publisher(Status, f'/{n}/status', 10)
            self.kal_pub[n] = self.create_publisher(LogDataGeneric, f'/{n}/kalman_variance', 10)
            self.create_service(Takeoff, f'/{n}/takeoff', self._takeoff(n), callback_group=cb)
            self.create_service(GoTo, f'/{n}/go_to', self._goto(n), callback_group=cb)
            self.create_service(Land, f'/{n}/land', self._land(n), callback_group=cb)
            self.create_service(Arm, f'/{n}/arm', self._arm(n), callback_group=cb)
            self.get_logger().info(f'[{n}] 준비. 기지 ({self.base[n][0]:+.2f}, {self.base[n][1]:+.2f})')

        self.create_timer(0.1, self._tick, callback_group=cb)   # 10 Hz

    # ---- 이동 명령 공통: 현재 위치에서 목표까지 dur 초 동안 보간 ----
    def _start(self, name, gx, gy, gz, dur, kind):
        with self._lock:
            s = self.state[name]
            s['from'] = (s['x'], s['y'], s['z'])
            s['to'] = (float(gx), float(gy), float(gz))
            s['t0'] = time.monotonic()
            s['dur'] = max(0.05, float(dur))
        self.get_logger().info(
            f'[{name}] {kind} -> ({gx:+.2f}, {gy:+.2f}, {gz:.2f}) {dur:.1f}s')

    def _takeoff(self, name):
        def cb(request, response):
            with self._lock:
                x, y = self.state[name]['x'], self.state[name]['y']
            self._start(name, x, y, request.height, _secs(request.duration), 'takeoff')
            return response                      # 실기와 같이 빈 응답
        return cb

    def _goto(self, name):
        def cb(request, response):
            g = request.goal
            if request.relative:
                with self._lock:
                    s = self.state[name]
                    g_x, g_y, g_z = s['x'] + g.x, s['y'] + g.y, s['z'] + g.z
            else:
                g_x, g_y, g_z = g.x, g.y, g.z
            with self._lock:
                self.state[name]['yaw'] = float(request.yaw)
            self._start(name, g_x, g_y, g_z, _secs(request.duration), 'go_to')
            return response
        return cb

    def _land(self, name):
        def cb(request, response):
            with self._lock:
                x, y = self.state[name]['x'], self.state[name]['y']
            self._start(name, x, y, GROUND_Z, _secs(request.duration), 'land')
            return response
        return cb

    def _arm(self, name):
        def cb(request, response):
            with self._lock:
                self.state[name]['armed'] = bool(request.arm)
            self.get_logger().info(f'[{name}] arm={request.arm}')
            return response
        return cb

    # ---- 10 Hz: 위치 갱신 + 토픽 발행 ----
    def _tick(self):
        now_t = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        with self._lock:
            for n, s in self.state.items():
                if s['to'] is not None:
                    a = min(1.0, (now_t - s['t0']) / s['dur'])
                    fx, fy, fz = s['from']
                    tx, ty, tz = s['to']
                    s['x'], s['y'], s['z'] = fx + (tx - fx) * a, fy + (ty - fy) * a, fz + (tz - fz) * a
                    if a >= 1.0:
                        s['to'] = None           # 도착: 그 자리에 호버
            snapshot = {n: dict(s) for n, s in self.state.items()}

        for n, s in snapshot.items():
            p = PoseStamped()
            p.header.stamp = stamp
            p.header.frame_id = 'map'
            p.pose.position.x, p.pose.position.y, p.pose.position.z = s['x'], s['y'], s['z']
            p.pose.orientation.z = math.sin(s['yaw'] * 0.5)
            p.pose.orientation.w = math.cos(s['yaw'] * 0.5)
            self.pose_pub[n].publish(p)

            st = Status()
            st.header.stamp = stamp
            st.header.frame_id = 'map'
            # 비행 조건은 무조건 통과로 가정한다 (TUMBLED/LOCKED/CRASHED 없음).
            info = CAN_BE_ARMED | CAN_FLY | (IS_ARMED if s['armed'] else 0)
            if s['z'] >= AIRBORNE_Z:
                info |= IS_FLYING
            st.supervisor_info = info
            st.battery_voltage = 4.0
            st.pm_state = PM_STATE_BATTERY
            st.rssi = 60
            st.latency_unicast = 5
            self.status_pub[n].publish(st)

            kal = LogDataGeneric()
            kal.header.stamp = stamp
            kal.values = [0.0001, 0.0001, 0.0001]     # 수렴한 값 — 사전점검이 바로 통과
            self.kal_pub[n].publish(kal)


def main():
    rclpy.init()
    node = FakeCrazyflie()
    ex = MultiThreadedExecutor()
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
