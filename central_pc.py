#!/usr/bin/env python3
"""중앙 제어기 — BT 대신 시나리오를 순서대로 진행시킨다.

실기 BT(행동 트리)를 단순한 순차 상태기계로 바꾼 것이다. 로봇에 내는 명령과 다음 단계로
넘어가는 판정 기준은 BT 와 같게 맞췄다.

  드론에 보내는 것   /{드론}/takeoff · /go_to · /land   (서비스)
  리모에 보내는 것   /{리모}/navigate_to_pose           (액션)
  받아서 판정하는 것 /{드론}/pose                       (도착 판정)
                     /{드론}/marker_detections          (마커 발견)
  내는 것            /coshow/mission_state              (참관자 대시보드용 JSON, latched)
                     /{로봇}/led                        (LED 색. 실기 BT 의 SetLed 와 같다)

판정 규칙 — 실기와 같은 이유로 이렇게 한다.
  · 드론 도착: pose 가 목표 반경(arrive_tol) 안에 arrive_hold 초 머물면 도착.
    드론 명령은 서비스라 "완료" 응답이 없어서 위치를 보고 역추적할 수밖에 없다.
  · 리모 도착: Nav2 액션이 SUCCEEDED 를 돌려준 것만 인정. odom 은 보지 않는다.
  · 같은 목표는 서비스를 1회만 부른다. 매 tick 재호출하면 궤적이 t=0 으로 리셋된다.

진행 순서
   1 limo_a → 관측점            2 미션 드론 이륙 → 관측점 상공
   3 미션 마커 인식             4 미션 드론 복귀·착륙 ∥ limo_a 기지 복귀
   5 탐색 드론 3대 순차 출격 → 구역 왕복
   6 타겟 마커 발견(finder 확정) 7 finder 외 드론 복귀·착륙
   8 limo_b → 마커 접근점       9 도착 후 rescue_sec 대기
  10 finder 복귀·착륙 ∥ limo_b 기지 복귀 → 완료

실행:
  python3 central_pc.py
  python3 central_pc.py --ros-args -p config:=/path/to/config.yaml
"""
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from coshow_interfaces.msg import MarkerDetections
from crazyflie_interfaces.srv import GoTo, Land, Takeoff

AIRBORNE_Z, LANDED_Z = 0.25, 0.08


def _dur(sec):
    d = DurationMsg()
    d.sec = int(sec)
    d.nanosec = int((sec - int(sec)) * 1e9)
    return d


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


class CentralPC(Node):
    def __init__(self):
        super().__init__('central_pc')
        self.declare_parameter('config', str(Path(__file__).resolve().parent / 'config.yaml'))
        cfg = yaml.safe_load(Path(self.get_parameter('config').value).read_text(encoding='utf-8'))
        self.cfg = cfg
        d, l, f, m, t = cfg['drones'], cfg['limos'], cfg['field'], cfg['markers'], cfg['timing']
        self.mission_drone = d['mission']
        self.searchers = list(d['searchers'])
        self.drones = [self.mission_drone] + self.searchers
        self.base = {n: [float(v) for v in d['base'][n]] for n in self.drones}
        self.alt = d['altitude']
        self.limo_obs, self.limo_rescue = l['observer'], l['rescuer']
        self.limo_base = {k: [float(v) for v in v2] for k, v2 in l['base'].items()}
        self.observe = [float(v) for v in f['observe_point']]
        self.zones = f['zones']
        self.mission_id = int(m['mission_id'])
        self.target_id = self.mission_id - int(m['id_offset'])
        self.T = t

        cb = ReentrantCallbackGroup()
        self.pose, self.arrived, self.cmd = {}, {}, {}
        self._in_since = {}                       # 로봇 -> 목표 반경에 들어온 시각
        self._sent = {}                           # 로봇 -> 마지막으로 보낸 명령 서명
        self._landing = set()                     # land 를 보낸 기체 (다시 올리지 않는다)
        self.seen = {}                            # 드론 -> {마커id: (x, y, 시각)}

        self.cli = {}
        for n in self.drones:
            self.create_subscription(PoseStamped, f'/{n}/pose',
                                     lambda msg, r=n: self._on_pose(r, msg), 10, callback_group=cb)
            self.create_subscription(MarkerDetections, f'/{n}/marker_detections',
                                     lambda msg, r=n: self._on_det(r, msg), 10, callback_group=cb)
            self.cli[n] = {
                'takeoff': self.create_client(Takeoff, f'/{n}/takeoff', callback_group=cb),
                'go_to': self.create_client(GoTo, f'/{n}/go_to', callback_group=cb),
                'land': self.create_client(Land, f'/{n}/land', callback_group=cb)}
        self.nav = {k: ActionClient(self, NavigateToPose, f'/{k}/navigate_to_pose', callback_group=cb)
                    for k in (self.limo_obs, self.limo_rescue)}
        self.nav_state = {k: {'phase': 'idle', 'goal': None, 'future': None, 'handle': None}
                          for k in self.nav}
        self.limo_arrived = {}

        # 참관자 대시보드가 이 토픽 하나만 구독한다. latched 라 늦게 붙어도 최신 상태를 받는다.
        # LED. 실기 BT 의 SetLed 와 같은 토픽·타입이고, 색이 바뀔 때만 보낸다.
        self.led_pub = {r: self.create_publisher(String, f'/{r}/led', 10)
                        for r in self.drones + [self.limo_obs, self.limo_rescue]}
        self.led = {}

        self.mission_pub = self.create_publisher(
            String, '/coshow/mission_state',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # 시나리오 상태
        self.phase = 'wait_nodes'
        self.t0 = time.monotonic()
        self.mission_found = False
        self.finder = None
        self.P_N = None
        self.rescue_done_t = 0.0
        self.search_idx = {n: 0 for n in self.searchers}      # 구역 왕복 웨이포인트 번호
        self.launch_t = {}
        self._phase_t = time.monotonic()
        self._last_log = ''

        self.create_timer(0.1, self._tick, callback_group=cb)
        self.get_logger().info('중앙 제어기 시작 — 노드가 다 뜨기를 기다린다')

    def _set_led(self, robots, color):
        """색이 바뀔 때만 발행한다 (실기 SetLed 와 같다 — 매 tick 재발행하지 않는다)."""
        for r in robots:
            if self.led.get(r) == color:
                continue
            self.led[r] = color
            self.led_pub[r].publish(String(data=color))
            self.get_logger().info(f'[{r}] LED {color}')

    # ---------------------------------------------------------------- 수신
    def _on_pose(self, robot, msg):
        p = msg.pose.position
        self.pose[robot] = (p.x, p.y, p.z)

    def _on_det(self, robot, msg):
        now_t = time.monotonic()
        store = self.seen.setdefault(robot, {})
        for mk in msg.markers:
            store[int(mk.id)] = (float(mk.world_x), float(mk.world_y), now_t)

    # ---------------------------------------------------------------- 명령
    def _send(self, robot, kind, request, sig):
        """같은 목표는 한 번만 보낸다. 보냈으면 True."""
        sig = (kind,) + tuple(sig)
        if self._sent.get(robot) == sig:
            return False
        c = self.cli[robot][kind]
        if not c.service_is_ready():
            return False
        c.call_async(request)                 # 실기와 같이 응답은 보지 않는다
        self._sent[robot] = sig
        self.cmd[robot] = {'kind': kind, 'goal': [round(v, 3) for v in sig[1:]], 't': time.monotonic()}
        self.get_logger().info(f'[{robot}] {kind} {sig[1:]}')
        return True

    def _takeoff(self, robot, key='takeoff'):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = float(self.alt[robot][key])
        req.duration = _dur(self.T['takeoff_sec'])
        return self._send(robot, 'takeoff', req, (req.height,))

    def _goto(self, robot, gx, gy, alt_key):
        gz = float(self.alt[robot][alt_key])
        p = self.pose.get(robot)
        far = p is None or _dist(p[0], p[1], gx, gy) > float(self.T['short_dist'])
        dur = float(self.T['goto_long_sec'] if far else self.T['goto_short_sec'])
        req = GoTo.Request()
        req.group_mask = 0
        req.relative = False
        req.goal.x, req.goal.y, req.goal.z = float(gx), float(gy), gz
        req.yaw = 0.0
        req.duration = _dur(dur)
        return self._send(robot, 'go_to', req, (round(gx, 3), round(gy, 3), round(gz, 3)))

    def _land(self, robot):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = _dur(self.T['land_sec'])
        return self._send(robot, 'land', req, ())

    # ---------------------------------------------------------------- 판정
    def _at(self, robot, gx, gy, gz=None):
        """목표 반경에 arrive_hold 초 머물렀나. 벗어나면 타이머를 0 부터 다시 센다."""
        p = self.pose.get(robot)
        if p is None:
            self._in_since.pop(robot, None)
            return False
        tol = float(self.T['arrive_tol'])
        inside = _dist(p[0], p[1], gx, gy) <= tol and (gz is None or abs(p[2] - gz) <= tol)
        if not inside:
            self._in_since.pop(robot, None)
            return False
        goal = (round(gx, 3), round(gy, 3))
        prev = self._in_since.get(robot)
        if prev is None or prev[0] != goal:       # 목표가 바뀌면 처음부터 센다
            prev = (goal, time.monotonic())
            self._in_since[robot] = prev
        if time.monotonic() - prev[1] >= float(self.T['arrive_hold']):
            self.arrived[robot] = (gx, gy, prev[1])
            return True
        return False

    def _return_home(self, robot):
        """기지로 보내 착륙시킨다. 착륙까지 끝났으면 True.

        land 를 한 번 보낸 기체는 다시 go_to 로 올리지 않는다(_landing 래치).
        래치가 없으면 land 로 고도가 내려가면서 복귀 고도 기준 도착 판정이 거짓이 되고,
        그것을 "아직 안 왔다" 로 읽어 go_to 를 재전송해 다시 올라가는 일이 무한 반복된다.
        """
        if self._landed(robot):
            self._landing.discard(robot)
            return True
        if robot in self._landing:
            return False                          # 내려가는 중. 건드리지 않는다
        bx, by = self.base[robot]
        if self._at(robot, bx, by, float(self.alt[robot]['return'])):
            self._landing.add(robot)
            self._land(robot)
        else:
            self._goto(robot, bx, by, 'return')
        return False

    def _landed(self, robot):
        p = self.pose.get(robot)
        bx, by = self.base[robot]
        return (p is not None and p[2] <= LANDED_Z
                and _dist(p[0], p[1], bx, by) <= float(self.T['arrive_tol']) * 2)

    def _marker(self, marker_id, who=None):
        """마커를 본 (드론, x, y) 를 돌려준다. who 를 주면 그 드론 것만 본다."""
        for robot, store in self.seen.items():
            if who is not None and robot != who:
                continue
            hit = store.get(marker_id)
            if hit is not None:
                return robot, hit[0], hit[1]
        return None

    # ---------------------------------------------------------------- 리모 (Nav2 액션)
    def _nav(self, limo, gx, gy):
        """목표로 보낸다. 도착했으면 True. 같은 목표는 다시 보내지 않는다."""
        s = self.nav_state[limo]
        goal = (round(float(gx), 3), round(float(gy), 3))
        arr = self.limo_arrived.get(limo)
        if arr is not None and _dist(arr[0], arr[1], gx, gy) <= float(self.T['limo_tol']):
            return True
        if s['phase'] == 'idle' and s['goal'] != goal:
            if not self.nav[limo].server_is_ready():
                return False
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(gx), float(gy)
            ps.pose.orientation.w = 1.0
            g = NavigateToPose.Goal()
            g.pose = ps
            s['goal'], s['phase'] = goal, 'sending'
            self.cmd[limo] = {'kind': 'nav', 'goal': list(goal), 't': time.monotonic()}
            self.get_logger().info(f'[{limo}] navigate_to_pose -> {goal}')
            self.nav[limo].send_goal_async(g).add_done_callback(
                lambda fut, k=limo: self._nav_accepted(k, fut))
        elif s['phase'] == 'running' and s['future'] is not None and s['future'].done():
            res = s['future'].result()
            s['phase'], s['future'], s['handle'] = 'idle', None, None
            if res.status == GoalStatus.STATUS_SUCCEEDED:
                self.limo_arrived[limo] = (s['goal'][0], s['goal'][1], time.monotonic())
                self.get_logger().info(f'[{limo}] SUCCEEDED {s["goal"]}')
                return True
            self.get_logger().warn(f'[{limo}] status={res.status} — 다시 보낸다')
            s['goal'] = None                    # 재전송 허용
        return False

    def _nav_accepted(self, limo, future):
        s = self.nav_state[limo]
        try:
            handle = future.result()
        except Exception as exc:                 # 서버가 죽은 경우
            self.get_logger().warn(f'[{limo}] goal 전송 실패: {exc}')
            s['phase'], s['goal'] = 'idle', None
            return
        if not handle.accepted:
            self.get_logger().warn(f'[{limo}] goal 거절됨')
            s['phase'], s['goal'] = 'idle', None
            return
        s['handle'], s['future'], s['phase'] = handle, handle.get_result_async(), 'running'

    # ---------------------------------------------------------------- 탐색 경로
    def _waypoint(self, robot):
        """구역을 x 로 왕복한다. 레인 중앙 y 는 고정 (실기 lawnmower 와 같은 모양)."""
        z = self.zones[robot]
        x0, x1 = (float(v) for v in z['x'])
        pts = [(x0, float(z['y'])), (x1, float(z['y']))]
        return pts[self.search_idx[robot] % len(pts)]

    # ---------------------------------------------------------------- 상태기계
    def _go(self, phase):
        if self.phase != phase:
            self.get_logger().info(f'=== {self.phase} -> {phase} ===')
            self.phase, self._phase_t = phase, time.monotonic()

    def _tick(self):
        self._publish_state()
        p = self.phase

        if p == 'wait_nodes':
            ready = all(n in self.pose for n in self.drones)
            if ready and all(self.cli[n]['takeoff'].service_is_ready() for n in self.drones):
                self._go('limo_observe')
            return

        # 1. limo_a 를 관측점으로
        if p == 'limo_observe':
            if self._nav(self.limo_obs, *self.observe):
                self._go('mission_takeoff')
            return

        # 2. 미션 드론 이륙 -> 관측점 상공
        if p == 'mission_takeoff':
            m = self.mission_drone
            pz = self.pose.get(m, (0, 0, 0))[2]
            if pz < AIRBORNE_Z:
                self._takeoff(m)
                return
            self._go('mission_observe')
            return

        if p == 'mission_observe':
            m = self.mission_drone
            self._goto(m, self.observe[0], self.observe[1], 'observe')
            if self._at(m, self.observe[0], self.observe[1], float(self.alt[m]['observe'])):
                self._go('mission_detect')
            return

        # 3. 미션 마커 인식
        if p == 'mission_detect':
            hit = self._marker(self.mission_id, who=self.mission_drone)
            if hit is not None:
                self.mission_found = True
                self.get_logger().info(
                    f'미션 마커 {self.mission_id} 인식 — 찾을 타겟은 {self.target_id} 번')
                self._set_led(self.drones, 'blue')      # BT: SetLed robot="all" color="blue"
                self._go('mission_return')
            return

        # 4. 미션 드론 복귀·착륙  ∥  limo_a 기지 복귀
        if p == 'mission_return':
            m = self.mission_drone
            self._nav(self.limo_obs, *self.limo_base[self.limo_obs])
            if self._return_home(m):
                self._go('search_launch')
            return

        # 5. 탐색 드론 순차 출격 -> 구역 왕복
        if p == 'search_launch':
            now_t = time.monotonic()
            prev = None
            for n in self.searchers:
                if n not in self.launch_t:
                    if prev is not None and now_t - prev < float(self.T['stagger_sec']):
                        break                       # 앞 기체 출격 후 stagger 만큼 기다린다
                    self.launch_t[n] = now_t
                prev = self.launch_t[n]
            for n in list(self.launch_t):
                if self.pose.get(n, (0, 0, 0))[2] < AIRBORNE_Z:
                    self._takeoff(n)
            if len(self.launch_t) == len(self.searchers) and \
                    all(self.pose.get(n, (0, 0, 0))[2] >= AIRBORNE_Z for n in self.searchers):
                self._go('search')
            return

        # 6. 구역 왕복하며 타겟 마커를 찾는다
        if p == 'search':
            for n in self.searchers:
                gx, gy = self._waypoint(n)
                self._goto(n, gx, gy, 'search')
                if self._at(n, gx, gy, float(self.alt[n]['search'])):
                    self.search_idx[n] += 1         # 다음 웨이포인트 (끝까지 가면 되돌아온다)
            hit = self._marker(self.target_id)
            if hit is not None:
                self.finder = hit[0]
                self.P_N = {'x': hit[1], 'y': hit[2], 'z': float(self.alt[hit[0]]['capture'])}
                self.get_logger().info(
                    f'타겟 마커 {self.target_id} 발견 — finder={self.finder} '
                    f'P_N=({hit[1]:+.2f}, {hit[2]:+.2f})')
                self._set_led([self.finder], 'red')     # BT: SetLed robot_key="finder" color="red"
                self._go('others_return')
            return

        # 7. finder 외 드론 복귀·착륙 (finder 는 마커 위에서 호버)
        if p == 'others_return':
            self._goto(self.finder, self.P_N['x'], self.P_N['y'], 'capture')
            others = [n for n in self.searchers if n != self.finder]
            done = [self._return_home(n) for n in others]   # 전부 평가해야 모두에게 명령이 간다
            if all(done):
                self._go('rescue_dispatch')
            return

        # 8·9. limo_b 를 마커 접근점으로 -> 도착 후 rescue_sec 대기
        if p == 'rescue_dispatch':
            gx = self.P_N['x'] + float(self.T['limo_approach_dx'])
            gy = self.P_N['y']
            if self._nav(self.limo_rescue, gx, gy):
                self._set_led([self.finder], 'green')   # BT: 리모 도착 뒤 SetLed green
                arr = self.limo_arrived[self.limo_rescue]
                if time.monotonic() - arr[2] >= float(self.T['rescue_sec']):
                    self.rescue_done_t = time.monotonic()
                    self.get_logger().info('구조 완료')
                    self._set_led(self.drones, 'off')   # BT: SetLed robot="all" color="off"
                    self._go('final_return')
            return

        # 10. finder 복귀·착륙  ∥  limo_b 기지 복귀
        if p == 'final_return':
            self._nav(self.limo_rescue, *self.limo_base[self.limo_rescue])
            f_home = self._return_home(self.finder)
            limo_home = self.limo_arrived.get(self.limo_rescue)
            home_ok = limo_home is not None and _dist(
                limo_home[0], limo_home[1], *self.limo_base[self.limo_rescue]) <= float(self.T['limo_tol'])
            if f_home and home_ok:
                self._go('done')
            return

        if p == 'done' and self._last_log != 'done':
            self._last_log = 'done'
            self.get_logger().info('시나리오 완료 — 모든 기체 복귀')

    # ---------------------------------------------------------------- 대시보드 발행
    def _derive_phase(self):
        """실기 BT 의 _derive_phase 와 같은 규칙으로 국면을 도출한다.

        내부 단계 이름(limo_observe 등)을 그대로 쓰지 않고 상태 조합으로 계산하는 이유는,
        대시보드가 기대하는 문자열이 BT 쪽에서 그렇게 정의돼 있기 때문이다.
        """
        if any(r not in self.pose for r in self.drones):
            return 'waiting_poses'
        air = lambda r: self.pose.get(r, (0, 0, 0))[2] >= AIRBORNE_Z
        if self.rescue_done_t > 0:
            return 'return' if any(air(d) for d in self.drones) else 'done'
        if self.finder is not None:
            arr = self.limo_arrived.get(self.limo_rescue)
            if arr is not None and self.P_N is not None:
                gx = self.P_N['x'] + float(self.T['limo_approach_dx'])
                if _dist(arr[0], arr[1], gx, self.P_N['y']) <= float(self.T['limo_tol']) + 0.1:
                    return 'rescue'                  # 구조 리모가 접근점 도착
            c = self.cmd.get(self.limo_rescue)
            if c is not None and c['kind'] == 'nav':
                return 'rescue_dispatch'             # 구조 리모 출동 명령 나감
            return 'capture'                         # 타겟 발견, 위치 확인 중
        if self.mission_found:
            return 'search' if any(air(d) for d in self.searchers) else 'handover'
        return 'observe'                             # 미션 마커 판독 전

    def _publish_state(self):
        payload = {
            't': time.monotonic(),
            'phase': self._derive_phase(),
            'led': dict(self.led),
            'mission_marker_id': self.mission_id if self.mission_found else None,
            'target_id': self.target_id if self.mission_found else None,
            'finder': self.finder,
            'target_confirmed': self.finder is not None,
            'preflight_required': False,
            'preflight_ready': True,
            'rescue_done_t': float(self.rescue_done_t),
            'missing_pose': [n for n in self.drones if n not in self.pose],
        }
        if self.P_N is not None:
            payload['P_N'] = {k: float(v) for k, v in self.P_N.items()}
        if self.cmd:
            payload['cmd'] = {r: {'kind': c['kind'], 'goal': [float(g) for g in c['goal']],
                                  't': float(c['t'])} for r, c in self.cmd.items()}
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        self.mission_pub.publish(msg)


def main():
    rclpy.init()
    node = CentralPC()
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
