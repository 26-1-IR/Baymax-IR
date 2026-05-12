#!/usr/bin/env python3
"""Autonomous parking node for ego_hatchback.

Full pipeline (§5 of project proposal):
  INIT         – wait for first odom
  ENTER_LOT    – drive to lane entry (-10.5, 0.0)
  OBS_DRIVE    – drive to next observation point
  OBS_YAW      – rotate to face observation direction (separate from position)
  OBS_WAIT     – publish trigger → wait for decision_node to publish target slot
  LANE_DRIVE   – drive along y=0 to slot x (tight tolerance)
  TURN_TO_SLOT – rotate in place to face slot direction (north/south)
  X_ALIGN      – re-centre on slot x after rotation
  PARK_FORWARD – reverse into slot; stop when depth+ROI condition met or obstacle
  DONE         – hold position

Topics published:
  /parking/obs_trigger  (Bool)  – fires vision_node scan at each obs point

Topics subscribed:
  /parking/target_slot  (String) – slot id from decision_node
  /parking/no_slot      (Bool)   – decision_node found no valid slot
"""

import math
import os
import yaml
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from ament_index_python.packages import get_package_share_directory

try:
    from gazebo_msgs.srv import SpawnEntity, DeleteEntity
    GAZEBO_SRV_OK = True
except ImportError:
    GAZEBO_SRV_OK = False


def angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def quat_to_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# Observation points (from slot_metadata.yaml)
OBS_POINTS = [
    {'id': 'obs_1', 'x': -5.0, 'y': 0.0, 'yaw': 0.0},  # 진행방향 유지
    {'id': 'obs_2', 'x':  5.0, 'y': 0.0, 'yaw': 0.0},
]


class ParkingNode(Node):
    # Target y for car centre when fully inside slot (rear near stopper at ±8.0m)
    PARK_Y_A =  5.7   # Row A — north side
    PARK_Y_B = -5.7   # Row B — south side

    PARK_SPEED = 0.35   # m/s while reversing into slot
    LANE_SPEED = 1.5    # m/s along lane

    # LiDAR 기반 방지턱 감지 멈춤 거리
    # 방지턱 y=±8.0m, 차 중심에서 방지턱까지 LiDAR 거리가 이 값 이하이면 정지
    # car_rear = center + 2.1m → 방지턱까지 0.3m 여유 → 2.1+0.3 = 2.4m
    STOPPER_LIDAR_DIST = 2.4   # m — rear LiDAR to wheel stopper
    REAR_EMRG_DIST     = 0.3   # m — emergency rear guard (non-stopper obstacles)
    SIDE_STOP_DIST     = 0.7   # m — side LiDAR guard (adjacent cars)

    OBS_WAIT_TIMEOUT = 10.0   # s  — give up waiting for decision_node

    def __init__(self):
        super().__init__('parking_node')

        pkg = get_package_share_directory('autonomous_parking')
        with open(os.path.join(pkg, 'config', 'slot_metadata.yaml')) as f:
            data = yaml.safe_load(f)
        self.slots = {s['id']: s for s in data['slots']}

        self.x = -12.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_ok = False

        self.ranges = None
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / 360

        # State
        self.state = 'INIT'
        self.target_slot = None
        self.slot_target_yaw = 0.0
        self.obs_idx = 0          # index into OBS_POINTS
        self.obs_wait_start = None
        self.received_target = False
        self._last_cmd = (0.0, 0.0)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/ego/cmd_vel', 10)
        self.obs_trigger_pub = self.create_publisher(Bool, '/parking/obs_trigger', 10)

        # Subscribers
        self.create_subscription(Odometry, '/ego/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/ego/scan', self._scan_cb, 10)
        self.create_subscription(String, '/parking/target_slot', self._target_cb, 10)
        self.create_subscription(Bool, '/parking/no_slot', self._no_slot_cb, 10)

        # Gazebo 슬롯 하이라이트
        self._highlight_name = None
        if GAZEBO_SRV_OK:
            self._spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
            self._delete_client = self.create_client(DeleteEntity, '/delete_entity')
        else:
            self._spawn_client  = None
            self._delete_client = None
            self.get_logger().warn('gazebo_msgs not found — slot highlight disabled')

        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._log_status)   # periodic status at 2 Hz
        self.get_logger().info('Parking node started')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.odom_ok = True

    def _scan_cb(self, msg):
        self.ranges = list(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    # States where a new target from decision_node is accepted.
    # During active parking (LANE_DRIVE onward) a stale/re-published message
    # must not overwrite the slot we are already driving toward.
    _RECEPTIVE_STATES = frozenset({
        'INIT', 'ENTER_LOT', 'OBS_DRIVE', 'OBS_YAW', 'OBS_WAIT'
    })

    def _target_cb(self, msg):
        if self.state not in self._RECEPTIVE_STATES:
            self.get_logger().debug(
                f'_target_cb: ignoring late target in state {self.state}'
            )
            return
        slot_id = msg.data.strip()
        if slot_id not in self.slots:
            self.get_logger().error(f'Unknown slot id from decision_node: {slot_id}')
            return
        self.get_logger().info(f'decision_node selected slot: {slot_id}')
        self._set_target(self.slots[slot_id])
        self.received_target = True

    def _no_slot_cb(self, msg):
        if msg.data:
            self.get_logger().error('No valid slot available — going to DONE')
            self._pub(0.0, 0.0)
            self.state = 'DONE'

    # ── helpers ────────────────────────────────────────────────────────────

    def _lidar_range(self, robot_angle):
        if not self.ranges:
            return float('inf')
        idx = round((robot_angle - self.angle_min) / self.angle_inc)
        idx = max(0, min(len(self.ranges) - 1, idx))
        r = self.ranges[idx]
        return float('inf') if (math.isnan(r) or math.isinf(r)) else r

    def _stopper_reached(self):
        """True when the rear LiDAR detects the wheel stopper within STOPPER_LIDAR_DIST.

        방지턱 LiDAR 반사체(z=1.45m)가 수평 스캔에 보이면 거리가 줄어듦.
        STOPPER_LIDAR_DIST(2.4m) 이하이면 차 뒷부분이 방지턱 0.3m 앞에 있는 것.
        """
        for deg in range(-6, 7, 2):
            r = self._lidar_range(math.pi + math.radians(deg))
            if r < self.STOPPER_LIDAR_DIST:
                return True
        return False

    def _rear_emergency(self):
        """True when something unexpectedly close behind (< REAR_EMRG_DIST)."""
        for deg in range(-6, 7, 2):
            if self._lidar_range(math.pi + math.radians(deg)) < self.REAR_EMRG_DIST:
                return True
        return False

    def _sides_clear(self):
        """True when neither flank has an obstacle within SIDE_STOP_DIST.

        후진 주차 중 좌우 감지: 차가 남향(-π/2)일 때 슬롯 좌우는 동/서 방향.
        동/서 = 로봇 프레임 ±90°. ±80-100° 범위로 실제 옆 방향을 감지.
        """
        for deg in range(80, 101, 5):
            if self._lidar_range(math.radians(deg)) < self.SIDE_STOP_DIST:
                return False
            if self._lidar_range(math.radians(-deg)) < self.SIDE_STOP_DIST:
                return False
        return True

    def _in_slot_boundary(self):
        """True when car centre is fully within the target slot's ROI."""
        roi = self.target_slot.get('roi')
        if not roi:
            return False
        return (roi['x_min'] < self.x < roi['x_max'] and
                roi['y_min'] < self.y < roi['y_max'])

    # ── Gazebo 슬롯 하이라이트 ─────────────────────────────────────────────

    def _highlight_slot(self, slot, color='green'):
        """목표 슬롯 위에 반투명 색상 박스를 Gazebo에 스폰."""
        if not self._spawn_client:
            return
        self._delete_highlight()

        r, g, b = (0.0, 1.0, 0.0) if color == 'green' else (0.2, 0.5, 1.0)
        name = 'slot_highlight'
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="floor">
        <pose>0 0 0.005 0 0 0</pose>
        <geometry><box><size>2.6 4.3 0.008</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 0.6</ambient>
          <diffuse>{r} {g} {b} 0.6</diffuse>
          <emissive>{r*0.4:.2f} {g*0.4:.2f} {b*0.4:.2f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(slot['center_x'])
        req.initial_pose.position.y = float(slot['center_y'])
        req.initial_pose.position.z = 0.0
        self._spawn_client.call_async(req)
        self._highlight_name = name
        self.get_logger().info(
            f"Slot highlight spawned at ({slot['center_x']}, {slot['center_y']}) [{color}]"
        )

    def _delete_highlight(self):
        if not self._delete_client or not self._highlight_name:
            return
        req = DeleteEntity.Request()
        req.name = self._highlight_name
        self._delete_client.call_async(req)
        self._highlight_name = None

    def _goto(self, tx, ty, tol=0.35, speed=None):
        if speed is None:
            speed = self.LANE_SPEED
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < tol:
            return 0.0, 0.0, True
        target_yaw = math.atan2(dy, dx)
        err = angle_diff(target_yaw, self.yaw)
        lin = min(speed, 1.5 * dist)
        ang = max(-1.5, min(1.5, 2.0 * err))
        if abs(err) > 0.5:
            lin *= 0.15
        elif abs(err) > 0.2:
            lin *= 0.5
        return lin, ang, False

    def _pub(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)
        self._last_cmd = (lin, ang)

    def _log_status(self):
        if not self.odom_ok:
            return
        fwd  = self._lidar_range(0.0)
        left = self._lidar_range(math.pi / 2)
        right= self._lidar_range(-math.pi / 2)
        rear = self._lidar_range(math.pi)
        lin, ang = getattr(self, '_last_cmd', (0.0, 0.0))

        target_info = ''
        if self.target_slot:
            sx = self.target_slot['center_x']
            sy = self.target_slot['center_y']
            dist = math.hypot(sx - self.x, sy - self.y)
            target_info = (
                f" | target={self.target_slot['id']}"
                f" ({sx:.1f},{sy:.1f}) dist={dist:.2f}m"
            )

        self.get_logger().info(
            f"[{self.state}] "
            f"pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}° "
            f"cmd=lin:{lin:.2f} ang:{ang:.2f} "
            f"lidar F:{fwd:.2f} L:{left:.2f} R:{right:.2f} B:{rear:.2f}"
            + target_info
        )

    def _fire_obs_trigger(self):
        msg = Bool()
        msg.data = True
        self.obs_trigger_pub.publish(msg)

    # ── state machine ──────────────────────────────────────────────────────

    def _tick(self):
        if not self.odom_ok:
            return

        if self.state not in ('INIT', 'OBS_WAIT', 'DONE'):
            if math.hypot(self.x - self._stuck_last_pos[0],
                        self.y - self._stuck_last_pos[1]) > 0.05:
                self._stuck_last_pos = (self.x, self.y)
                self._stuck_since = self.get_clock().now()
            elif (self.get_clock().now() - self._stuck_since).nanoseconds / 1e9 > 4.0:
                self.get_logger().error(
                    f'Stuck at ({self.x:.2f},{self.y:.2f}) in {self.state} — DONE'
                )
                self._pub(0.0, 0.0)
                self.state = 'DONE'
                return

        # ── INIT ──────────────────────────────────────────────────────────
        if self.state == 'INIT':
            self.get_logger().info('Entering parking lot...')
            self.state = 'ENTER_LOT'
            self._stuck_last_pos = (self.x, self.y)
            self._stuck_since = self.get_clock().now()  

        # ── ENTER_LOT ─────────────────────────────────────────────────────
        elif self.state == 'ENTER_LOT':
            lin, ang, done = self._goto(-10.5, 0.0)
            if done:
                self.get_logger().info(
                    f'ENTER_LOT done — pos=({self.x:.2f},{self.y:.2f})'
                )
                self._pub(0.0, 0.0)
                self.obs_idx = 0
                self.state = 'OBS_DRIVE'
            else:
                self._pub(lin, ang)

        # ── OBS_DRIVE ─────────────────────────────────────────────────────
        # Navigate to obs point position only (tol=0.6m).
        # Yaw alignment is handled in OBS_YAW so drift during rotation
        # never re-triggers navigation (which caused south-spiral bug).
        elif self.state == 'OBS_DRIVE':
            obs = OBS_POINTS[self.obs_idx]
            lin, ang, done = self._goto(obs['x'], obs['y'], tol=0.6)
            if done:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"OBS_DRIVE done — pos=({self.x:.2f},{self.y:.2f}) → OBS_YAW"
                )
                self.state = 'OBS_YAW'
            else:
                self._pub(lin, ang)

        # ── OBS_YAW ───────────────────────────────────────────────────────
        # Pure in-place rotation to face the observation direction.
        # Position drift during rotation is accepted (no re-navigation).
        elif self.state == 'OBS_YAW':
            obs = OBS_POINTS[self.obs_idx]
            yaw_err = angle_diff(obs['yaw'], self.yaw)
            if abs(yaw_err) < 0.08:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"OBS_YAW done — pos=({self.x:.2f},{self.y:.2f}) "
                    f"yaw={math.degrees(self.yaw):.1f}° — triggering vision scan"
                )
                self._fire_obs_trigger()
                self.obs_wait_start = self.get_clock().now()
                self.received_target = False
                self.state = 'OBS_WAIT'
            else:
                ang = max(-1.2, min(1.2, 2.5 * yaw_err))
                self.get_logger().debug(
                    f"OBS_YAW: err={math.degrees(yaw_err):.1f}° ang={ang:.2f}"
                )
                self._pub(0.0, ang)

        # ── OBS_WAIT ──────────────────────────────────────────────────────
        # Wait for decision_node to publish a target slot.
        elif self.state == 'OBS_WAIT':
            self._pub(0.0, 0.0)

            if self.received_target:
                self.state = 'LANE_DRIVE'
                return

            elapsed = (self.get_clock().now() - self.obs_wait_start).nanoseconds / 1e9
            if elapsed > self.OBS_WAIT_TIMEOUT:
                self.get_logger().warn(
                    f'Timeout at {OBS_POINTS[self.obs_idx]["id"]} — '
                    f'moving to next obs point'
                )
                self.obs_idx += 1
                if self.obs_idx >= len(OBS_POINTS):
                    self.get_logger().error('All obs points exhausted — DONE')
                    self.state = 'DONE'
                else:
                    self.state = 'OBS_DRIVE'

        # ── LANE_DRIVE ────────────────────────────────────────────────────
        # Two-phase: (A) rotate in place to face (sx, 0), (B) drive with steering.
        # `_goto` alone spirals when start yaw is >90° off target bearing.
        elif self.state == 'LANE_DRIVE':
            sx = self.target_slot['center_x']
            dx = sx - self.x
            dy = 0.0 - self.y
            dist = math.hypot(dx, dy)

            if dist < 0.15:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"LANE_DRIVE done — pos=({self.x:.2f},{self.y:.2f}) "
                    f"target_x={sx:.2f} x_err={dx:.3f}m"
                )
                self.state = 'TURN_TO_SLOT'
                return

            target_bearing = math.atan2(dy, dx)
            yaw_err = angle_diff(target_bearing, self.yaw)

            # Phase A: in-place rotation until roughly aligned
            if abs(yaw_err) > 0.15:
                ang = max(-1.2, min(1.2, 2.0 * yaw_err))
                self._pub(0.0, ang)
                self.get_logger().debug(
                    f"LANE_YAW: bearing={math.degrees(target_bearing):.1f}° "
                    f"err={math.degrees(yaw_err):.1f}°"
                )
                return

            # Phase B: drive with gentle steering correction
            lin = min(self.LANE_SPEED, 1.5 * dist)
            ang = max(-0.8, min(0.8, 2.0 * yaw_err))
            self._pub(lin, ang)

        # ── TURN_TO_SLOT ──────────────────────────────────────────────────
        elif self.state == 'TURN_TO_SLOT':
            err = angle_diff(self.slot_target_yaw, self.yaw)
            if abs(err) < 0.05:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"TURN done — yaw={math.degrees(self.yaw):.1f}° "
                    f"target={math.degrees(self.slot_target_yaw):.1f}° "
                    f"err={math.degrees(err):.1f}° "
                    f"pos=({self.x:.2f},{self.y:.2f})"
                )
                self.state = 'X_ALIGN'
            else:
                ang = max(-1.2, min(1.2, 2.5 * err))
                self._pub(0.0, ang)

        # ── X_ALIGN ───────────────────────────────────────────────────────
        # 차가 슬롯 반대 방향을 바라보며 후진으로 x를 맞춘다.
        # Row A (남향 -π/2, 후진→북): 동쪽 표류 = angular.z < 0 → sign = -1
        # Row B (북향 +π/2, 후진→남): 동쪽 표류 = angular.z > 0 → sign = +1
        elif self.state == 'X_ALIGN':
            row = self.target_slot['id'][0]
            ang_sign = -1.0 if row == 'A' else 1.0
            sx = self.target_slot['center_x']
            x_err = sx - self.x
            if abs(x_err) < 0.12:
                self._pub(0.0, 0.0)
                self.get_logger().info(
                    f"X_ALIGN done — x_err={x_err:.3f}m "
                    f"pos=({self.x:.2f},{self.y:.2f}) → PARK_FORWARD"
                )
                self.state = 'PARK_FORWARD'
            else:
                ang = max(-0.6, min(0.6, ang_sign * x_err))
                self.get_logger().debug(
                    f"X_ALIGN reverse nudge: x_err={x_err:.3f}m ang={ang:.2f} "
                    f"(row {row}, sign={ang_sign:+.0f})"
                )
                self._pub(-0.15, ang)

        # ── PARK_FORWARD ──────────────────────────────────────────────────
        # Reverse parking: robot faces AWAY from slot, reverses in (lin < 0).
        # Completion requires BOTH depth target reached AND car inside slot ROI.
        elif self.state == 'PARK_FORWARD':
            row = self.target_slot['id'][0]
            park_y = self.PARK_Y_A if row == 'A' else self.PARK_Y_B
            sx = self.target_slot['center_x']

            # 진입 직후 진단 로그 (debug 레벨, 매 tick)
            stopper_now = self._stopper_reached()
            sides_now   = self._sides_clear()
            rear_emrg   = self._rear_emergency()
            self.get_logger().debug(
                f"PARK_FWD entry: pos=({self.x:.2f},{self.y:.2f}) yaw={math.degrees(self.yaw):.1f}° "
                f"stopper={stopper_now} sides_ok={sides_now} rear_emrg={rear_emrg}"
            )

            y_remaining = abs(park_y - self.y)
            rear_dist  = self._lidar_range(math.pi)
            left_dist  = self._lidar_range(math.pi / 2)
            right_dist = self._lidar_range(-math.pi / 2)

            self.get_logger().debug(
                f"PARK_REV: y_rem={y_remaining:.2f}m "
                f"lidar REAR={rear_dist:.2f} L={left_dist:.2f} R={right_dist:.2f} "
                f"x_err={self.x - sx:.3f}m in_slot={self._in_slot_boundary()}"
            )

            # ── 멈춤 판정 (우선순위 순) ────────────────────────────────────
            # 1순위: LiDAR로 방지턱 감지 (방지턱에 0.3m 여유 남긴 시점)
            stopper_hit = self._stopper_reached()
            # 2순위: 깊이 체크 (LiDAR 미감지 fallback)
            depth_ok = (row == 'A' and self.y >= park_y) or \
                       (row == 'B' and self.y <= park_y)
            # 3순위: 슬롯 ROI 내부 진입 확인
            in_roi = self._in_slot_boundary()

            if stopper_hit or (depth_ok and in_roi):
                self._pub(0.0, 0.0)
                stop_reason = 'LiDAR 방지턱 감지' if stopper_hit else '목표 깊이 도달'
                self.get_logger().info(
                    f"PARKED in {self.target_slot['id']}! [{stop_reason}] "
                    f"pos=({self.x:.2f},{self.y:.2f}) "
                    f"x_err={self.x - sx:.3f}m "
                    f"rear={rear_dist:.2f}m"
                )
                # 하이라이트를 파란색으로 전환 (주차 완료 표시)
                self._highlight_slot(self.target_slot, color='blue')
                self.state = 'DONE'
                return
            elif depth_ok and not in_roi:
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"깊이 도달했으나 ROI 밖 — pos=({self.x:.2f},{self.y:.2f}) "
                    f"x_err={self.x - sx:.3f}m — 정지"
                )
                self._highlight_slot(self.target_slot, color='blue')
                self.state = 'DONE'
                return

            # 비상 후방 감지 (방지턱보다 훨씬 가까운 예상치 못한 장애물)
            if self._rear_emergency():
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"비상: 후방 장애물 {rear_dist:.2f}m — 정지"
                )
                self.state = 'DONE'
                return

            # Side collision guard (adjacent parked cars)
            if not self._sides_clear():
                self._pub(0.0, 0.0)
                self.get_logger().warn(
                    f"Side obstacle — L={left_dist:.2f}m R={right_dist:.2f}m — aborting slot"
                )
                self.slots[self.target_slot['id']]['occupied'] = True
                self._replan()
                return

            # Maintain alignment with slot centre while reversing.
            # Sign convention mirrors X_ALIGN:
            #   Row A (남향, 후진→북): ang_sign = -1
            #   Row B (북향, 후진→남): ang_sign = +1
            ang_sign = -1.0 if row == 'A' else 1.0
            x_err = sx - self.x
            ang_correction = max(-0.3, min(0.3, ang_sign * 0.6 * x_err))
            self._pub(-self.PARK_SPEED, ang_correction)

        # ── DONE ──────────────────────────────────────────────────────────
        elif self.state == 'DONE':
            self._pub(0.0, 0.0)

    # ── utilities ──────────────────────────────────────────────────────────

    def _set_target(self, slot):
        self.target_slot = slot
        # Reverse parking: face AWAY from slot so the car backs in.
        # Row A (north, y+): face south (-π/2) → reverse goes north
        # Row B (south, y-): face north (+π/2) → reverse goes south
        self.slot_target_yaw = (
            -math.pi / 2 if slot['id'][0] == 'A' else math.pi / 2
        )
        self.get_logger().info(
            f"Target: slot {slot['id']} "
            f"(x={slot['center_x']}, {'north←reverse' if slot['id'][0]=='A' else 'south←reverse'})"
        )
        # Gazebo 시각화: 초록 하이라이트로 목표 슬롯 표시
        self._highlight_slot(slot, color='green')

    def _replan(self):
        """Fall back to local yaml-based selection after a collision abort."""
        candidates = [
            s for s in self.slots.values()
            if not s['occupied'] and s['type'] == 'general'
        ]
        candidates.sort(key=lambda s: abs(s['center_x'] + 10.5))
        if candidates:
            self._set_target(candidates[0])
            self.state = 'LANE_DRIVE'
        else:
            self.get_logger().error('No more empty slots!')
            self.state = 'DONE'


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
