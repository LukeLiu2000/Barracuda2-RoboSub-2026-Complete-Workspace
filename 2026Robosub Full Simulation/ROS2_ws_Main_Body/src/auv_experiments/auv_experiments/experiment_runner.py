#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""experiment_runner —— 实验编排 ROS 2 节点(系统 python, rclpy)。

统一流程(三种控制器同一代码路径):
  载入配置 -> 桥复位(pause+set_pose+zero+扰动配置+控制器复位+play)
  -> 等有效状态 -> 开始记录(CSV + 可选 rosbag2) -> 发目标 -> 判据监控
  -> 回合结束(保持成功/超时/出界/状态过期) -> 落盘 -> 汇总。

时间约定:运动判据/指标用仿真时间(消息头,/clock 系);推理耗时与
等待用墙钟 time.monotonic();回合切分按仿真时间跳变(离线复算)。

记录(每回合目录):
  states.csv goals.csv thrust.csv disturbance.csv diag.csv
  metadata.json bag/(可选 rosbag2)
运行目录: run.json + summary.csv + 在线指标(与离线 analyze.py 同源)。

用法:
  ros2 run auv_experiments experiment_runner --ros-args -p out_dir:=... \
      -p controller:=pid -p bridge_port:=17790 -p tasks:=fwd_+0.4,dist_step_hold
"""

import csv
import json
import os
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from auv_experiments import tasks as task_lib                      # noqa: E402
from auv_experiments import metrics as metrics_lib                 # noqa: E402
from auv_common.frames import attitude_error_rotvec                # noqa: E402

BAG_TOPICS = [
    "/auv/state/control", "/auv/ground_truth/odometry", "/auv/control/goal",
    "/auv/control/thruster_cmd", "/auv/bridge/applied_forces",
    "/auv/disturbance/forces", "/auv/bridge/status", "/clock",
    "/auv/pid/diagnostics", "/auv/gru/diagnostics",
]


def tcp_call(host, port, cmd, payload=None, timeout=20.0):
    """桥 TCP JSON 行协议(与 bridge_client 同一格式)。
    timeout=20s:桥端 USD/timeline 操作排队主线程最长等 15s,须留裕量。"""
    import socket
    s = socket.create_connection((host, int(port)), timeout=timeout)
    try:
        req = {"cmd": cmd}
        if payload:
            req.update(payload)
        s.sendall((json.dumps(req) + "\n").encode("utf-8"))
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        line = buf.split(b"\n")[0].decode("utf-8", "replace")
        return json.loads(line) if line else {"ok": False, "error": "no resp"}
    finally:
        s.close()


class ExperimentRunner(Node):

    def __init__(self):
        super().__init__("auv_experiment_runner")
        self.declare_parameter("out_dir",
                               os.path.expanduser("~/auv_experiments/run1"))
        self.declare_parameter("controller", "pid")     # pid|mppi|gru
        self.declare_parameter("bridge_host", "127.0.0.1")
        self.declare_parameter("bridge_port", 17790)
        self.declare_parameter("tasks", "all")
        self.declare_parameter("record_bag", True)
        self.declare_parameter("pos_tol_m", 0.08)
        self.declare_parameter("ang_tol_rad", 0.087)
        self.declare_parameter("hold_time_s", 3.0)
        self.declare_parameter("state_timeout_wall_s", 1.0)
        self.declare_parameter("oob_radius_m", 2.5)
        # 注:use_sim_time 由 rclpy 自动声明(Jazzy),此处不再显式声明;
        # 判据一律用消息头时间,不依赖节点钟。

        self.out_dir = os.path.abspath(
            self.get_parameter("out_dir").value)
        os.makedirs(os.path.join(self.out_dir, "episodes"), exist_ok=True)
        self.controller = str(self.get_parameter("controller").value)
        self.host = str(self.get_parameter("bridge_host").value)
        self.port = int(self.get_parameter("bridge_port").value)
        task_sel = str(self.get_parameter("tasks").value)
        self.record_bag = bool(self.get_parameter("record_bag").value)
        self.pos_tol = float(self.get_parameter("pos_tol_m").value)
        self.ang_tol = float(self.get_parameter("ang_tol_rad").value)
        self.hold_time = float(self.get_parameter("hold_time_s").value)
        self.state_timeout = float(
            self.get_parameter("state_timeout_wall_s").value)
        self.oob = float(self.get_parameter("oob_radius_m").value)

        # ---- 订阅/发布
        self._lock = threading.Lock()
        self._state = None            # (t_sim, t_wall, p, q, v, w)
        self._goal_echo = None
        self._u_req = None
        self._u_applied = None
        self._dis = None
        self._diag = None
        self._buf = {"states": [], "goals": [], "thrust_req": [],
                     "thrust_applied": [], "disturb": [], "diag": []}
        self.create_subscription(Odometry, "/auv/state/control",
                                 self._on_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/auv/control/goal",
                                 self._on_goal_echo, 10)
        self.create_subscription(Float64MultiArray,
                                 "/auv/control/thruster_cmd",
                                 self._on_cmd, 10)
        # 桥端两个高频力话题以 sensor-data(BEST_EFFORT)发布,订阅侧必须
        # 用兼容 QoS,否则 DDS 静默丢弃(实测告警:incompatible RELIABILITY)。
        self.create_subscription(Float64MultiArray,
                                 "/auv/bridge/applied_forces",
                                 self._on_applied, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, "/auv/disturbance/forces",
                                 self._on_dis, qos_profile_sensor_data)
        for topic in ("/auv/pid/diagnostics", "/auv/gru/diagnostics"):
            self.create_subscription(String, topic,
                                     self._on_diag, 10)
        self.pub_goal = self.create_publisher(PoseStamped,
                                              "/auv/control/goal", 10)
        self.pub_reset = self.create_publisher(String, "/auv/control/reset",
                                               10)

    # ---------------------------------------------------------------- 回调
    def _on_state(self, m):
        h = m.header.stamp
        t_sim = h.sec + h.nanosec * 1e-9
        q = np.array([m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                      m.pose.pose.orientation.z, m.pose.pose.orientation.w])
        row = (t_sim, time.monotonic(),
               m.pose.pose.position.x, m.pose.pose.position.y,
               m.pose.pose.position.z, q[0], q[1], q[2], q[3],
               m.twist.twist.linear.x, m.twist.twist.linear.y,
               m.twist.twist.linear.z, m.twist.twist.angular.x,
               m.twist.twist.angular.y, m.twist.twist.angular.z)
        with self._lock:
            self._state = row
            self._buf["states"].append(row)

    def _on_goal_echo(self, m):
        h = m.header.stamp
        q = m.pose.orientation
        row = (h.sec + h.nanosec * 1e-9, time.monotonic(),
               m.pose.position.x, m.pose.position.y, m.pose.position.z,
               q.x, q.y, q.z, q.w)
        with self._lock:
            self._goal_echo = row
            self._buf["goals"].append(row)

    def _on_cmd(self, m):
        row = (time.monotonic(), list(m.data))
        with self._lock:
            self._u_req = row
            self._buf["thrust_req"].append(row)

    def _on_applied(self, m):
        row = (time.monotonic(), list(m.data))
        with self._lock:
            self._u_applied = row
            self._buf["thrust_applied"].append(row)

    def _on_dis(self, m):
        d = list(m.data)
        row = (time.monotonic(), d[:6], d[6:12] if len(d) >= 12 else [])
        with self._lock:
            self._dis = row
            self._buf["disturb"].append(row)

    def _on_diag(self, m):
        try:
            d = json.loads(m.data)
        except Exception:                               # noqa: BLE001
            return
        d["_t_wall"] = time.monotonic()
        with self._lock:
            self._diag = d
            self._buf["diag"].append(d)

    # ---------------------------------------------------------------- 主流程
    def run(self):
        st = self.tcp("step_state")
        # 桥返回 {"ok":true,"state":{pos_m,quat_xyzw,...}}(state 可能为 None)
        sst = st.get("state") if st.get("ok") else None
        if not sst:
            self.get_logger().error(
                "桥 step_state 失败(尚无物理步?): %s" % st)
            return
        spawn = {"p": np.asarray(sst["pos_m"], dtype=float),
                 "q": np.asarray(sst["quat_xyzw"], dtype=float)}
        self.get_logger().info(
            "出生位姿 p=%s q=%s" % (np.round(spawn["p"], 3).tolist(),
                                    np.round(spawn["q"], 4).tolist()))
        all_tasks = task_lib.task_list(spawn["p"], spawn["q"])
        sel = str(self.get_parameter("tasks").value)
        if sel and sel != "all":
            wanted = [x.strip() for x in sel.split(",") if x.strip()]
            all_tasks = [t for t in all_tasks if t[0] in wanted]
            missing = [w for w in wanted
                       if w not in [t[0] for t in all_tasks]]
            if missing:
                self.get_logger().warning("未知任务名忽略: %s" % missing)
        rows = []
        for i, (name, goal, disturb, dur, purpose) in enumerate(all_tasks):
            self.get_logger().info(
                "[ep %d/%d] %s (%s)" % (i + 1, len(all_tasks), name, purpose))
            try:
                md, online = self.run_episode(i, name, goal, disturb, dur,
                                              purpose, spawn)
            except Exception as exc:                    # noqa: BLE001
                import traceback
                self.get_logger().error(
                    "回合 %s 异常: %s\n%s" % (name, exc,
                                               traceback.format_exc()[-800:]))
                continue
            rows.append(online)
            self.get_logger().info(
                "[ep %d] %s -> %s succ=%s rmse=%.3f" % (
                    i, name, md["end_reason"], online.get("success"),
                    online.get("rmse_pos_m", float("nan"))))
        # 汇总
        with open(os.path.join(self.out_dir, "summary.csv"), "w",
                  newline="", encoding="utf-8") as f:
            if rows:
                w = csv.DictWriter(f, fieldnames=sorted(
                    {k for r in rows for k in r}))
                w.writeheader()
                for r in rows:
                    w.writerow(r)
        run_meta = {
            "controller": self.controller,
            "bridge": "%s:%d" % (self.host, self.port),
            "n_episodes": len(rows),
            "spawn_p": spawn["p"].tolist(),
            "spawn_q": spawn["q"].tolist(),
            "tolerances": {"pos_m": self.pos_tol,
                           "ang_rad": self.ang_tol,
                           "hold_s": self.hold_time},
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "python": sys.version.split()[0],
            "finished": time.strftime("%Y-%m-%d %H:%M:%S"),
            "workspace_note": "ROS2_ws_Main_Body 无 git 仓库;改动前快照 "
                              "auv_ws_src_snapshot_20261107.tar.gz(见报告)",
        }
        with open(os.path.join(self.out_dir, "run.json"), "w",
                  encoding="utf-8") as f:
            json.dump(run_meta, f, ensure_ascii=False, indent=2)
        self.tcp("zero", {})
        self.tcp("timeline", {"action": "pause"})
        self.get_logger().info("完成: %s" % self.out_dir)

    # ---------------------------------------------------------------- 回合
    def tcp(self, cmd, payload=None):
        return tcp_call(self.host, self.port, cmd, payload)

    def run_episode(self, idx, name, goal, disturb, dur, purpose, spawn):
        ep_dir = os.path.join(self.out_dir, "episodes",
                              "ep%02d_%s" % (idx, name))
        os.makedirs(ep_dir, exist_ok=True)
        # 1) 复位序列
        self.tcp("timeline", {"action": "pause"})
        time.sleep(0.5)
        self.tcp("set_pose", {"pos_m": spawn["p"].tolist(),
                              "quat_xyzw": spawn["q"].tolist()})
        self.tcp("zero", {})
        if disturb:
            patch = dict(disturb)
            # 注意:disturb_config 的补丁须嵌套在 "patch" 键下(与
            # bridge_client --patch 一致);顶层合并会被当成空补丁,
            # 扰动从不启用(2026-11-08 attempt4 实测定位)。
            r = self.tcp("disturb_config", {"patch": patch})
            # 显式核对配置生效(此前版本静默忽略返回值,异常无从发现)
            en = bool((r.get("disturbance") or {}).get("enabled"))
            self.get_logger().info(
                "disturb_config ok=%s enabled=%s resp_err=%s" % (
                    r.get("ok"), en, r.get("error")))
            if not (r.get("ok") and en):
                reset_note_extra = "disturb_config 未生效: %s" % \
                    str(r.get("error") or r)[:200]
            else:
                reset_note_extra = ""
        else:
            self.tcp("disturb_stop", {})
            reset_note_extra = ""
        reset_note = self.reset_controller()
        # 2) 清缓冲
        with self._lock:
            self._buf = {"states": [], "goals": [], "thrust_req": [],
                         "thrust_applied": [], "disturb": [], "diag": []}
            self._state = None
        self.tcp("timeline", {"action": "play"})
        if disturb:
            # play 后核对扰动确实在采样(2s 足够首拍)
            time.sleep(2.0)
            dg = self.tcp("disturb_get")
            chans = (dg.get("disturbance") or {}).get("channels", [])
            ns = [c.get("n_samples", 0) for c in chans]
            self.get_logger().info(
                "disturb_verify n_samples=%s enabled=%s" % (
                    ns, (dg.get("disturbance") or {}).get("enabled")))
        # 3) 等有效状态
        t0 = time.monotonic()
        while time.monotonic() - t0 < 20.0:
            with self._lock:
                s = self._state
            if s is not None and time.monotonic() - s[1] < 0.5:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("20s 内无有效状态")
        # 4) 录 bag
        bag_proc = None
        if self.record_bag:
            bag_log = open(os.path.join(ep_dir, "bag_stdout.log"), "w")
            bag_proc = subprocess.Popen(
                ["ros2", "bag", "record", "-o",
                 os.path.join(ep_dir, "bag")] + BAG_TOPICS,
                stdout=bag_log, stderr=subprocess.STDOUT)
        # 5) 发目标
        with self._lock:
            s = self._state
        t_sim_now = s[0]
        gm = PoseStamped()
        gm.header.stamp.sec = int(t_sim_now)
        gm.header.stamp.nanosec = int((t_sim_now % 1.0) * 1e9)
        gm.header.frame_id = "world"
        gm.pose.position.x, gm.pose.position.y, gm.pose.position.z = \
            goal["p"].tolist()
        gm.pose.orientation.x, gm.pose.orientation.y, \
            gm.pose.orientation.z, gm.pose.orientation.w = goal["q"].tolist()
        self.pub_goal.publish(gm)
        # 6) 判据监控
        end_reason = "timeout"
        t_goal_sim = t_sim_now
        hold_since = None
        last_print = 0.0
        while True:
            time.sleep(0.05)
            with self._lock:
                s = self._state
            if s is None or time.monotonic() - s[1] > self.state_timeout:
                end_reason = "state_stale"
                break
            t_sim = s[0]
            p = np.array(s[2:5])
            q = np.array(s[5:9])
            e_p = float(np.linalg.norm(goal["p"] - p))
            e_R = float(np.linalg.norm(
                attitude_error_rotvec(q, goal["q"])))
            if float(np.linalg.norm(p - spawn["p"])) > self.oob:
                end_reason = "oob_abort"
                break
            if t_sim - t_goal_sim > dur:
                end_reason = "timeout"
                break
            if e_p < self.pos_tol and e_R < self.ang_tol:
                if hold_since is None:
                    hold_since = t_sim
                elif t_sim - hold_since >= self.hold_time:
                    end_reason = "hold_success"
                    break
            else:
                hold_since = None
            if time.monotonic() - last_print > 5.0:
                self.get_logger().info(
                    "  %s t=%.1f e_p=%.3f e_R=%.3f" %
                    (name, t_sim - t_goal_sim, e_p, e_R))
                last_print = time.monotonic()
        self.tcp("zero", {})
        # 7) 停 bag + 落盘
        if bag_proc is not None:
            bag_proc.send_signal(2)      # SIGINT -> rosbag2 收尾元数据
            try:
                bag_proc.wait(timeout=15.0)
            except subprocess.TimeoutExpired:
                bag_proc.kill()
        with self._lock:
            buf = {k: list(v) for k, v in self._buf.items()}
            diag_latest = dict(self._diag) if self._diag else {}
        self._write_csvs(ep_dir, buf)
        st_bridge = self.tcp("status")
        infer = [d.get("infer_ms") for d in buf["diag"]
                 if d.get("infer_ms") is not None]
        md = {
            "task": name, "purpose": purpose, "controller": self.controller,
            "goal_p": goal["p"].tolist(), "goal_q": goal["q"].tolist(),
            "spawn_p": spawn["p"].tolist(), "spawn_q": spawn["q"].tolist(),
            "disturbance": disturb, "duration_limit_s": dur,
            "end_reason": end_reason,
            "reset": {**(reset_note if isinstance(reset_note, dict) else
                       {"note": str(reset_note)}),
                      **({"disturb_note": reset_note_extra}
                         if reset_note_extra else {})},
            "tolerances": {"pos_m": self.pos_tol,
                           "ang_rad": self.ang_tol,
                           "hold_s": self.hold_time},
            "bridge_status": {
                k: st_bridge.get(k) for k in (
                    "sim_time_s", "mode", "gravity_valid", "geom_check",
                    "disturbance", "thruster_cmd_publishers",
                    "multi_pub_fault")},
            "diag_latest": diag_latest,
            "bag": bool(self.record_bag),
            "n_samples": {k: len(v) for k, v in buf.items()},
        }
        with open(os.path.join(ep_dir, "metadata.json"), "w",
                  encoding="utf-8") as f:
            json.dump(md, f, ensure_ascii=False, indent=2,
                      default=str)
        online = self._online_metrics(ep_dir, md)
        return md, online

    # ---------------------------------------------------------------- 辅助
    def reset_controller(self):
        note = {}
        if self.controller == "pid":
            note["pid_reset"] = self._call_service(
                "/pid_controller_node/reset")
        elif self.controller == "gru":
            note["gru_reset_hidden"] = self._call_service(
                "/gru_controller_node/reset_hidden")
        m = String()
        m.data = "episode_reset"
        self.pub_reset.publish(m)
        note["reset_topic"] = "published /auv/control/reset"
        return note

    def _call_service(self, name):
        try:
            cli = self.create_client(Trigger, name)
            if not cli.wait_for_service(timeout_sec=5.0):
                return "service_not_available"
            fut = cli.call_async(Trigger.Request())
            t0 = time.monotonic()      # spin 线程驱动回调,这里只等待
            while not fut.done() and time.monotonic() - t0 < 3.0:
                time.sleep(0.02)
            if fut.done():
                return "ok: %s" % fut.result().message
            return "timeout"
        except Exception as exc:                        # noqa: BLE001
            return "error: %s" % exc

    def _write_csvs(self, ep_dir, buf):
        with open(os.path.join(ep_dir, "states.csv"), "w", newline="",
                  encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_sim", "t_wall", "px", "py", "pz", "qx", "qy",
                        "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"])
            w.writerows(buf["states"])
        with open(os.path.join(ep_dir, "goals.csv"), "w", newline="",
                  encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_sim", "t_wall", "px", "py", "pz", "qx", "qy",
                        "qz", "qw"])
            w.writerows(buf["goals"])
        with open(os.path.join(ep_dir, "thrust.csv"), "w", newline="",
                  encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_wall", "kind"] + ["u%d" % i for i in range(8)])
            for t, u in buf["thrust_req"]:
                w.writerow([t, "req"] + list(u))
            for t, u in buf["thrust_applied"]:
                w.writerow([t, "applied"] + list(u))
        with open(os.path.join(ep_dir, "disturbance.csv"), "w", newline="",
                  encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_wall", "f0x", "f0y", "f0z", "f1x", "f1y", "f1z",
                        "p0x", "p0y", "p0z", "p1x", "p1y", "p1z"])
            for t, f6, p6 in buf["disturb"]:
                w.writerow([t] + list(f6) + list(p6))
        with open(os.path.join(ep_dir, "diag.jsonl"), "w",
                  encoding="utf-8") as f:
            for d in buf["diag"]:
                f.write(json.dumps(d, ensure_ascii=False, default=str)
                        + "\n")

    def _online_metrics(self, ep_dir, md):
        """在线指标 = 离线 metrics 模块同一函数(可复现性由 analyze 验证)。"""
        try:
            states = list(csv.reader(open(os.path.join(
                ep_dir, "states.csv"), encoding="utf-8")))[1:]
            goals = list(csv.reader(open(os.path.join(
                ep_dir, "goals.csv"), encoding="utf-8")))[1:]
            if not states or not goals:
                return {"task": md["task"], "success": False,
                        "note": "no_samples"}
            t = np.array([float(r[0]) for r in states])
            p = np.array([[float(x) for x in r[2:5]] for r in states])
            quats = np.array([[float(x) for x in r[5:9]] for r in states])
            tg = np.array([float(r[0]) for r in goals])
            gp = np.array([[float(x) for x in r[2:5]] for r in goals])
            gq = np.array([[float(x) for x in r[5:9]] for r in goals])
            GP, GQ = metrics_lib.interp_goal(t, tg, gp, gq)
            e_p, e_R = metrics_lib.error_series(p, quats, GP, GQ)
            v = np.array([[float(x) for x in r[9:12]] for r in states])
            w = np.array([[float(x) for x in r[12:15]] for r in states])
            infer = []
            jl = os.path.join(ep_dir, "diag.jsonl")
            if os.path.isfile(jl):
                with open(jl, encoding="utf-8") as f:
                    for line in f:
                        try:
                            d = json.loads(line)
                        except Exception:               # noqa: BLE001
                            continue
                        if d.get("infer_ms") is not None:
                            infer.append(d["infer_ms"])
            rec = {"t_sim": t, "err_p": e_p, "err_r": e_R, "v": v, "w": w,
                   "t_thr": [], "u_req": np.zeros((0, 8)),
                   "u_applied": np.zeros((0, 8)), "t_dis": [],
                   "f_dis": np.zeros((0, 6)), "infer_ms": infer}
            m = metrics_lib.episode_metrics(rec, self.pos_tol, self.ang_tol)
            m["task"] = md["task"]
            m["end_reason"] = md["end_reason"]
            m["controller"] = md["controller"]
            return m
        except Exception as exc:                        # noqa: BLE001
            return {"task": md["task"], "metrics_error": str(exc)}


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    code = 0
    try:
        node.run()
    except Exception:                                  # noqa: BLE001
        import traceback
        traceback.print_exc()
        code = 1
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except Exception:                              # noqa: BLE001
            pass
        # rclpy(Jazzy)在有存续订阅线程时,解释器正常收尾会于 C++ 静态析构
        # 阶段 abort("terminate called without an active exception",
        # 2026-11-08 实测,退出码 250)。所有数据在 run() 内已落盘,
        # 此处显式退出码收尾。
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)


if __name__ == "__main__":
    main()
