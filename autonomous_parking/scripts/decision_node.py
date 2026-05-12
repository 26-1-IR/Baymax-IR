#!/usr/bin/env python3
"""Decision node: user credential + slot selection.

Subscribes:
  /parking/slot_states  (std_msgs/String, JSON from vision_node)

Publishes:
  /parking/target_slot  (std_msgs/String, slot id e.g. "A3")
  /parking/no_slot      (std_msgs/Bool,   True when no valid slot found)

Parameters:
  user_credential  (string)  'general' | 'handicapped'   default: 'general'

Decision pipeline (§4 of project proposal):
  1. Load candidate slots from /parking/slot_states
  2. Remove occupied slots
  3. Filter by user credential:
       general     → exclude handicapped slots
       handicapped → all empty slots allowed (handicapped preferred)
  4. Sort by distance from lane entry (-10.5, 0) + prefer handicapped for handicapped users
  5. Publish best slot id
"""

import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class DecisionNode(Node):
    LANE_ENTRY_X = -10.5

    def __init__(self):
        super().__init__('decision_node')

        self.declare_parameter('user_credential', 'general')
        self.credential = self.get_parameter('user_credential').value.lower()
        self.get_logger().info(f'User credential: {self.credential}')

        self.slot_states = {}     # id → dict
        self.published = False    # publish at most once per observation cycle

        self.create_subscription(String, '/parking/slot_states', self._states_cb, 10)

        self.pub_target = self.create_publisher(String, '/parking/target_slot', 10)
        self.pub_no_slot = self.create_publisher(Bool, '/parking/no_slot', 10)

        self.get_logger().info('Decision node ready — waiting for slot states...')

    # ── callback ───────────────────────────────────────────────────────────

    def _states_cb(self, msg):
        if self.published:
            self.get_logger().debug('Already published target for this cycle — ignoring duplicate slot_states')
            return
        try:
            slots = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'JSON decode error: {e}')
            return

        self.slot_states = {s['id']: s for s in slots}
        self.get_logger().info(
            f'Received {len(self.slot_states)} slot states — selecting...'
        )
        self._select_and_publish()

    # ── decision logic ────────────────────────────────────────────────────

    def _select_and_publish(self):
        # Step 1 – empty slots only
        candidates = [
            s for s in self.slot_states.values() if not s['occupied']
        ]

        # Step 2 – filter by credential
        if self.credential == 'general':
            candidates = [s for s in candidates if s['type'] == 'general']
        # handicapped user: all empty slots are allowed

        if not candidates:
            self.get_logger().warn(
                f'No valid slot for credential="{self.credential}"'
            )
            msg = Bool()
            msg.data = True
            self.pub_no_slot.publish(msg)
            return

        # Step 3 – sort: handicapped users prefer handicapped slots first,
        #          then by distance from lane entry
        def sort_key(s):
            type_priority = 0 if (
                self.credential == 'handicapped' and s['type'] == 'handicapped'
            ) else 1
            dist = abs(s['center_x'] - self.LANE_ENTRY_X)
            return (type_priority, dist)

        candidates.sort(key=sort_key)
        best = candidates[0]

        self.get_logger().info(
            f"Selected slot {best['id']} "
            f"(type={best['type']}, x={best['center_x']}, y={best['center_y']})"
        )

        msg = String()
        msg.data = best['id']
        self.pub_target.publish(msg)
        self.published = True


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
