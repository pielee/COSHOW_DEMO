#!/usr/bin/env python3
"""가짜 영상 검출 — 카메라 없이 마커 검출만 흉내낸다.

실기 영상 노드(aideck_aruco_node)가 내는 것 중 중앙 제어기가 쓰는 토픽 하나만 낸다.

  /{드론}/marker_detections                 coshow_interfaces/msg/MarkerDetections
  /aideck/{드론}/image_annotated/compressed  sensor_msgs/msg/CompressedImage
  /aideck/{드론}/fps                         std_msgs/msg/Float32
  /aideck/{드론}/stream_ok                   std_msgs/msg/Bool

이름이 둘로 갈린 이유는 실기와 같다. 검출 결과는 crazyswarm2·BT 가 쓰는 전역 이름(/cf231/...)
으로, 영상·진단은 노드 네임스페이스(/aideck/cf231/...) 아래로 나간다.

마커 두 종류를 다르게 다룬다.

  [미션 마커]  관측점(0,0)에 있다. 미션 드론이 그 위에 도착하면 mission_delay_max 안의
               랜덤한 시점에 인식한다. "카메라가 읽는 데 걸리는 시간" 을 흉내낸 것이라
               위치 반경은 보지 않고 도착 여부만 본다.

  [타겟 마커]  시작할 때 탐색 경로 위의 랜덤한 지점에 놓는다. 드론이 그 근처
               (detect_radius) 를 지나가면 발견한다. 그래서 누가 먼저 그 구역을
               지나느냐에 따라 finder 가 매번 달라진다.
               --seed 를 주면 같은 자리에 놓여 재현된다.

world_x/world_y 에는 마커의 실제 좌표를 그대로 넣는다. 실기에서는 역투영 오차가 섞이는데,
여기서는 오차 없는 이상적인 검출로 둔다 (--noise 로 오차를 줄 수 있다).

실행:
  python3 vision/vision.py
  python3 vision/vision.py --ros-args -p seed:=42 -p noise:=0.05
"""
import math
import random
import threading
import time
import pathlib
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32
from coshow_interfaces.msg import MarkerDetection, MarkerDetections

def _default_config():
    """config/config.yaml 의 경로. 노드는 <루트>/<폴더>/x.py, 설정은 <루트>/config/ 에 있다."""
    here = pathlib.Path(__file__).resolve().parent
    for cand in (here.parent / 'config' / 'config.yaml', here / 'config.yaml'):
        if cand.is_file():
            return str(cand)
    return str(here.parent / 'config' / 'config.yaml')   # 없으면 이 경로로 명확히 실패하게 둔다


AIRBORNE_Z = 0.25


class FakeVision(Node):
    def __init__(self):
        super().__init__('fake_vision')
        self.declare_parameter('config', _default_config())
        self.declare_parameter('seed', -1)          # -1 = 매번 다르게
        self.declare_parameter('noise', 0.0)        # 검출 좌표에 섞을 오차 (m)
        cfg = yaml.safe_load(Path(self.get_parameter('config').value).read_text(encoding='utf-8'))
        seed = int(self.get_parameter('seed').value)
        self.rng = random.Random(None if seed < 0 else seed)
        self.noise = float(self.get_parameter('noise').value)

        d, f, m = cfg['drones'], cfg['field'], cfg['markers']
        self.mission_drone = d['mission']
        self.searchers = list(d['searchers'])
        self.names = [self.mission_drone] + self.searchers
        self.observe = [float(v) for v in f['observe_point']]
        self.mission_id = int(m['mission_id'])
        self.target_id = self.mission_id - int(m['id_offset'])
        self.radius = float(m['detect_radius'])
        self.delay_max = float(m['mission_delay_max'])
        self.arrive_tol = float(cfg['timing']['arrive_tol'])

        # 타겟 마커를 탐색 경로 위 랜덤 지점에 놓는다.
        # 구역 하나를 고르고, 그 구역의 x 구간 안에서 한 점을 뽑는다 (레인 중앙 y 는 고정).
        who = self.rng.choice(self.searchers)
        zone = f['zones'][who]
        x0, x1 = (float(v) for v in zone['x'])
        self.target_xy = (self.rng.uniform(x0, x1), float(zone['y']))
        self.get_logger().info(
            f'타겟 마커 {self.target_id} 를 ({self.target_xy[0]:+.2f}, {self.target_xy[1]:+.2f}) 에 둔다 '
            f'— {who} 구역. seed={"랜덤" if seed < 0 else seed}')
        self.get_logger().info(
            f'미션 마커 {self.mission_id} 는 ({self.observe[0]:+.2f}, {self.observe[1]:+.2f}) '
            f'— {self.mission_drone} 도착 후 0~{self.delay_max:.0f}s 안에 인식')

        self._lock = threading.Lock()
        self.pose = {}
        self._arrived_t = None        # 미션 드론이 관측점 반경에 처음 들어온 시각
        self._mission_at = None       # 그때 뽑은 "인식되는 시각"
        for n in self.names:
            self.create_subscription(PoseStamped, f'/{n}/pose',
                                     lambda msg, r=n: self._on_pose(r, msg), 10)
        # 실기 영상 노드와 같은 QoS (depth 1, KEEP_LAST)
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.pubs = {n: self.create_publisher(MarkerDetections, f'/{n}/marker_detections', qos)
                     for n in self.names}
        # 대시보드가 보는 영상·진단 토픽. 실기에서는 노드 네임스페이스가 /aideck 다.
        self.pub_img = {n: self.create_publisher(
            CompressedImage, f'/aideck/{n}/image_annotated/compressed', qos) for n in self.names}
        self.pub_fps = {n: self.create_publisher(Float32, f'/aideck/{n}/fps', qos) for n in self.names}
        self.pub_ok = {n: self.create_publisher(Bool, f'/aideck/{n}/stream_ok', qos) for n in self.names}

        self.fps = float(m['frame_rate'])
        self.create_timer(1.0 / self.fps, self._frame)
        self.create_timer(1.0, self._diag)           # fps·stream_ok 는 1 Hz

    def _on_pose(self, robot, msg):
        p = msg.pose.position
        with self._lock:
            self.pose[robot] = (p.x, p.y, p.z)

    def _jitter(self, x, y):
        if self.noise <= 0:
            return x, y
        return x + self.rng.gauss(0, self.noise), y + self.rng.gauss(0, self.noise)

    def _frame(self):
        """카메라 한 프레임. 기체마다 지금 보이는 마커를 담아 발행한다."""
        now_t = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        with self._lock:
            poses = dict(self.pose)

        # --- 미션 마커: 도착 후 랜덤 지연 ---
        mission_visible = False
        p = poses.get(self.mission_drone)
        if p is not None and p[2] >= AIRBORNE_Z and \
                math.hypot(p[0] - self.observe[0], p[1] - self.observe[1]) <= self.arrive_tol * 2:
            if self._arrived_t is None:
                self._arrived_t = now_t
                self._mission_at = now_t + self.rng.uniform(0.0, self.delay_max)
                self.get_logger().info(
                    f'{self.mission_drone} 관측점 도착 — {self._mission_at - now_t:.1f}s 뒤 인식 예정')
            mission_visible = now_t >= self._mission_at
        else:
            self._arrived_t = self._mission_at = None   # 벗어나면 다시 센다

        for n in self.names:
            p = poses.get(n)
            if p is None or p[2] < AIRBORNE_Z:
                continue                                 # 지상이면 카메라가 바닥만 본다
            msg = MarkerDetections()
            msg.header.stamp = stamp
            msg.header.frame_id = 'map'
            msg.drone = n
            dp = PoseStamped()
            dp.header.stamp = stamp
            dp.header.frame_id = 'map'
            dp.pose.position.x, dp.pose.position.y, dp.pose.position.z = p
            msg.drone_pose = dp

            if n == self.mission_drone and mission_visible:
                msg.markers.append(self._det(self.mission_id, *self.observe))
            # 타겟 마커: 누구든 반경 안에 들어오면 보인다 (미션기가 탐색에 투입돼도 동작)
            if math.hypot(p[0] - self.target_xy[0], p[1] - self.target_xy[1]) <= self.radius:
                msg.markers.append(self._det(self.target_id, *self.target_xy))

            # 마커가 0개여도 발행한다 — 실기 영상 노드와 같다 (발행 = 영상이 흐른다는 신호)
            self.pubs[n].publish(msg)
            self._publish_image(n, p, msg.markers, stamp)

    # ---- 영상 토픽 (대시보드 카메라 타일용) ----
    def _publish_image(self, name, pose, markers, stamp):
        """324x244 흑백 합성 영상. 실기 AI-deck 과 같은 해상도로, 검출된 마커를 네모로 그린다."""
        img = np.full((244, 324, 3), 40, np.uint8)
        cv2.putText(img, f'{name}  FAKE', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(img, f'x{pose[0]:+.2f} y{pose[1]:+.2f} z{pose[2]:.2f}', (8, 232),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1)
        for mk in markers:
            cx, cy = int(mk.cx), int(mk.cy)
            h = int(mk.size_px / 2)
            cv2.rectangle(img, (cx - h, cy - h), (cx + h, cy + h), (0, 220, 0), 2)
            cv2.putText(img, str(mk.id), (cx - h, cy - h - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1)
        ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return
        out = CompressedImage()
        out.header.stamp = stamp
        out.header.frame_id = name
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub_img[name].publish(out)

    def _diag(self):
        """fps·stream_ok. 실기에서는 실제 수신 프레임으로 계산한 값이 나간다."""
        with self._lock:
            poses = dict(self.pose)
        for n in self.names:
            alive = n in poses                       # pose 가 오면 그 기체는 살아 있다고 본다
            self.pub_fps[n].publish(Float32(data=self.fps if alive else 0.0))
            self.pub_ok[n].publish(Bool(data=bool(alive)))

    def _det(self, marker_id, wx, wy):
        d = MarkerDetection()
        d.id = int(marker_id)
        d.cx, d.cy = 162.0, 162.0       # 화면 중심이라고 둔다 (중앙 제어기는 안 쓴다)
        d.size_px = 45.0
        jx, jy = self._jitter(wx, wy)
        d.world_x, d.world_y = float(jx), float(jy)
        return d


def main():
    rclpy.init()
    node = FakeVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
