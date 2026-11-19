#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""终端键盘遥控节点。

机体坐标约定：
    +x 左，+y 上，+z 前。

按键：
    W/S       前进 / 后退目标
    A/D       左移 / 右移目标
    ↑/↓       上移 / 下移目标
    ←/→       左转 / 右转目标
    Space     急停
    X         解除键盘触发的急停
    R         重新捕获当前位姿
    Q         退出

终端没有可靠的松键事件，因此使用 0.4 秒按键保活窗口。
松键后目标冻结，机器人仍需追踪目标并减速。
"""

import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from auv_mission_control.goal_manager import GoalManager


KEY_VEL = {
    "w": ("surge", +1.0),
    "s": ("surge", -1.0),
    "a": ("sway", +1.0),
    "d": ("sway", -1.0),
    "up": ("heave", +1.0),
    "down": ("heave", -1.0),
}

KEY_YAW = {
    "left": +1.0,
    "right": -1.0,
}

KEY_PRESS_TTL = 0.4


class KeyboardTeleopNode(Node):

    def __init__(self):
        super().__init__("auv_keyboard_teleop")

        # 不能叫 declare_parameters：
        # 该名称属于 ROS Node 父类，覆盖后会导致初始化失败。
        self._declare_teleop_parameters()

        names = (
            "keyboard.surge_speed",
            "keyboard.sway_speed",
            "keyboard.heave_speed",
            "keyboard.yaw_rate_deg_s",
            "keyboard.max_goal_dist",
            "keyboard.max_goal_att_deg",
            "keyboard.state_timeout_s",
            "keyboard.goal_rate_hz",
            "keyboard.tick_hz",
            "keyboard.workspace_min",
            "keyboard.workspace_max",
            "keyboard.yaw_axis",
            "keyboard.yaw_rate_sign",
            "keyboard.state_topic",
            "keyboard.goal_topic",
        )
        p = {
            name: self.get_parameter(name).value
            for name in names
        }

        self.gm = GoalManager(
            surge_speed=float(p["keyboard.surge_speed"]),
            sway_speed=float(p["keyboard.sway_speed"]),
            heave_speed=float(p["keyboard.heave_speed"]),
            yaw_rate_deg_s=(
                float(p["keyboard.yaw_rate_deg_s"])
                * float(p["keyboard.yaw_rate_sign"])
            ),
            max_goal_dist=float(p["keyboard.max_goal_dist"]),
            max_goal_att_deg=float(p["keyboard.max_goal_att_deg"]),
            workspace_min=list(p["keyboard.workspace_min"]),
            workspace_max=list(p["keyboard.workspace_max"]),
            state_timeout=float(p["keyboard.state_timeout_s"]),
            yaw_axis=str(p["keyboard.yaw_axis"]),
        )

        self.state_frame = "auv_world"
        self.last_state_mono = 0.0
        self.estop = False
        self.auto_test_active_until = 0.0
        self.keys = {}
        self.lock = threading.Lock()

        self.pub_goal = self.create_publisher(
            PoseStamped,
            str(p["keyboard.goal_topic"]),
            10,
        )
        self.pub_estop = self.create_publisher(
            Bool,
            "/auv/control/estop",
            10,
        )

        self.create_subscription(
            Odometry,
            str(p["keyboard.state_topic"]),
            self._on_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/auv/training/experiment_id",
            self._on_exp_id,
            10,
        )

        self.tick_hz = max(
            5.0, float(p["keyboard.tick_hz"])
        )
        self.goal_period = 1.0 / max(
            0.5, float(p["keyboard.goal_rate_hz"])
        )

        self.last_goal_pub = 0.0
        self.last_print = 0.0

        self.timer = self.create_timer(
            1.0 / self.tick_hz,
            self.tick,
        )

        self.cli_estop = self.create_client(
            SetBool,
            "/mppi_controller_node/estop",
        )

        self.get_logger().info(
            "键盘遥控就绪："
            "W/S 前后，A/D 左右，↑↓ 升降，←→ 转向；"
            "Space 急停，X 恢复，R 重捕获，Q 退出"
        )

    def _declare_teleop_parameters(self):
        self.declare_parameter("keyboard.surge_speed", 0.30)
        self.declare_parameter("keyboard.sway_speed", 0.25)
        self.declare_parameter("keyboard.heave_speed", 0.25)
        self.declare_parameter("keyboard.yaw_rate_deg_s", 30.0)
        self.declare_parameter("keyboard.yaw_rate_sign", 1.0)

        self.declare_parameter("keyboard.max_goal_dist", 1.5)
        self.declare_parameter("keyboard.max_goal_att_deg", 45.0)
        self.declare_parameter("keyboard.state_timeout_s", 0.3)

        self.declare_parameter("keyboard.goal_rate_hz", 5.0)
        self.declare_parameter("keyboard.tick_hz", 20.0)

        self.declare_parameter(
            "keyboard.workspace_min",
            [-2.5, -1.0, -2.5],
        )
        self.declare_parameter(
            "keyboard.workspace_max",
            [2.5, 2.0, 2.5],
        )

        self.declare_parameter("keyboard.yaw_axis", "body_up")
        self.declare_parameter(
            "keyboard.state_topic",
            "/auv/state/control",
        )
        self.declare_parameter(
            "keyboard.goal_topic",
            "/auv/control/goal",
        )

    def _on_state(self, msg):
        pp = msg.pose.pose.position
        qq = msg.pose.pose.orientation

        now = time.monotonic()
        age = now - self.last_state_mono
        self.last_state_mono = now

        self.state_frame = (
            msg.header.frame_id or self.state_frame
        )

        self.gm.set_state(
            (pp.x, pp.y, pp.z),
            (qq.x, qq.y, qq.z, qq.w),
            age,
        )

    def _on_exp_id(self, msg):
        self.auto_test_active_until = (
            time.monotonic() + 5.0
        )

    def on_key(self, key):
        now = time.monotonic()

        with self.lock:
            if key in KEY_VEL or key in KEY_YAW:
                self.keys[key] = now

            elif key == "space":
                self.set_estop(True)

            elif key == "x":
                self.set_estop(False)

            elif key == "r":
                ok = self.gm.recapture()
                self.get_logger().info(
                    "重捕获当前位姿：%s"
                    % ("成功" if ok else "无状态")
                )

            elif key == "q":
                self.get_logger().info("退出")

    def set_estop(self, on):
        self.estop = on

        m = Bool()
        m.data = on
        self.pub_estop.publish(m)

        if self.cli_estop.service_is_ready():
            req = SetBool.Request()
            req.data = on
            self.cli_estop.call_async(req)

        self.get_logger().warn(
            "急停 %s"
            % ("触发" if on else "解除")
        )

    def tick(self):
        now = time.monotonic()
        state_age = now - self.last_state_mono

        with self.lock:
            self.keys = {
                key: stamp
                for key, stamp in self.keys.items()
                if now - stamp <= KEY_PRESS_TTL
            }

            surge = sum(
                sign
                for key, (axis, sign) in KEY_VEL.items()
                if key in self.keys and axis == "surge"
            )
            sway = sum(
                sign
                for key, (axis, sign) in KEY_VEL.items()
                if key in self.keys and axis == "sway"
            )
            heave = sum(
                sign
                for key, (axis, sign) in KEY_VEL.items()
                if key in self.keys and axis == "heave"
            )
            yaw = sum(
                sign
                for key, sign in KEY_YAW.items()
                if key in self.keys
            )

            keys_text = (
                ",".join(sorted(self.keys)) or "-"
            )

        blocked = now < self.auto_test_active_until

        if self.estop:
            pass
        elif blocked:
            self.gm.update(
                0.0, (0.0, 0.0, 0.0), 0.0
            )
        else:
            self.gm.update(
                1.0 / self.tick_hz,
                (sway, heave, surge),
                yaw,
            )

        if (
            self.gm.goal_initialized
            and now - self.last_goal_pub >= self.goal_period
        ):
            self.last_goal_pub = now

            g = PoseStamped()
            g.header.stamp = (
                self.get_clock().now().to_msg()
            )
            g.header.frame_id = self.state_frame

            (
                g.pose.position.x,
                g.pose.position.y,
                g.pose.position.z,
            ) = self.gm.goal_p

            (
                g.pose.orientation.x,
                g.pose.orientation.y,
                g.pose.orientation.z,
                g.pose.orientation.w,
            ) = self.gm.goal_q

            self.pub_goal.publish(g)

        if now - self.last_print >= 0.5:
            self.last_print = now

            if self.estop:
                note = "急停"
            elif blocked:
                note = "自动测试互斥"
            elif state_age > 1.0:
                note = "状态断流 %.1fs" % state_age
            else:
                note = self.gm.last_update_note

            if self.gm.goal_p is None:
                goal_text = "未初始化"
            else:
                goal_text = "(%.2f, %.2f, %.2f)" % (
                    self.gm.goal_p[0],
                    self.gm.goal_p[1],
                    self.gm.goal_p[2],
                )

            self.get_logger().info(
                "keys=[%s] %s | 目标=%s"
                % (keys_text, note, goal_text)
            )


def read_stdin(cb):
    """读取终端字符；方向键支持 CSI 和 SS3 序列。"""
    import os
    import select

    if not sys.stdin.isatty():
        raise RuntimeError(
            "键盘节点需要交互终端，请在 VS Code Terminal 中运行"
        )

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    arrows = {
        b"\x1b[A": "up",
        b"\x1b[B": "down",
        b"\x1b[C": "right",
        b"\x1b[D": "left",
        b"\x1bOA": "up",
        b"\x1bOB": "down",
        b"\x1bOC": "right",
        b"\x1bOD": "left",
    }

    try:
        tty.setcbreak(fd)

        while True:
            ch = os.read(fd, 1)
            if not ch:
                break

            if ch == b"\x1b":
                sequence = ch

                for _ in range(2):
                    ready, _, _ = select.select(
                        [fd], [], [], 0.05
                    )
                    if not ready:
                        break
                    sequence += os.read(fd, 1)

                key = arrows.get(sequence)
                if key:
                    cb(key)
                continue

            if ch == b"\x03":
                break

            if ch == b" ":
                cb("space")
                continue

            key = ch.decode(
                "ascii", errors="ignore"
            ).lower()

            if key in ("w", "s", "a", "d", "x", "r", "q"):
                cb(key)

                if key == "q":
                    break

    finally:
        termios.tcsetattr(
            fd, termios.TCSADRAIN, old
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    spin_thread = None

    try:
        node = KeyboardTeleopNode()

        spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(node,),
            daemon=True,
        )
        spin_thread.start()

        read_stdin(node.on_key)

    except (KeyboardInterrupt, EOFError):
        pass

    finally:
        rclpy.try_shutdown()

        if spin_thread is not None:
            spin_thread.join(timeout=2.0)

        if node is not None:
            node.destroy_node()


if __name__ == "__main__":
    main()