#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""web_keyboard —— 轻量浏览器键盘面板(SSH 终端无法可靠给出 keyup/组合时的
替代方案;同一套 GoalManager 逻辑,与 keyboard_node 行为一致)。

运行(服务器上):
    python3 web_keyboard.py
笔记本上做 SSH 端口转发后用浏览器访问:
    ssh -L 8791:127.0.0.1:8791 liuhao@172.30.30.1
    http://127.0.0.1:8791

可靠性处理(任务书要求):
  - keydown/keyup 成对,支持组合键(W+↑ 同时按);
  - 页面失焦 / 切后台(visibilitychange)-> 立即上传空键集(进入保持);
  - 心跳 0.5 s 一拍,服务端 1.5 s 无心跳或 0.7 s 无键集帧 -> 视为断连,
    清空激活键(目标冻结 -> 控制器平滑保持);
  - 急停(Space)/恢复(X)/重捕获(R)按钮与键盘节点一致。

键位与 keyboard_node 相同:W/S 前后,A/D 左右,↑↓ 升降,←→ 绕自身上轴转向。
"""

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

from auv_mission_control.goal_manager import GoalManager

HB_TIMEOUT = 1.5       # 心跳超时 -> 断连
KEYS_TIMEOUT = 0.7     # 键集帧超时 -> 视为松开全部
VALID_KEYS = {"w", "s", "a", "d", "up", "down", "left", "right"}

PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AUV 键盘遥控</title><style>
body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;
padding:16px;max-width:640px}
h1{font-size:18px;margin:0 0 8px}
.box{background:#1b1b1b;border:1px solid #333;border-radius:8px;padding:12px;
margin-bottom:12px}
kbd{display:inline-block;min-width:34px;text-align:center;padding:6px 8px;
margin:3px;border-radius:6px;background:#2a2a2a;border:1px solid #444;
font-size:15px}
kbd.on{background:#0a6;background-color:#1a7f37;border-color:#2ea043}
#status{font-family:monospace;font-size:13px;white-space:pre-wrap}
.warn{color:#f66}.ok{color:#7d7}
button{font-size:15px;padding:8px 14px;margin:4px;border-radius:6px;
border:1px solid #555;background:#2a2a2a;color:#eee}
button:active{background:#333}
</style></head><body>
<h1>AUV 键盘遥控(浏览器面板)</h1>
<div class="box">
<kbd id="k-w">W 前</kbd><kbd id="k-s">S 后</kbd>
<kbd id="k-a">A 左移</kbd><kbd id="k-d">D 右移</kbd><br>
<kbd id="k-up">↑ 升</kbd><kbd id="k-down">↓ 降</kbd>
<kbd id="k-left">← 左转</kbd><kbd id="k-right">→ 右转</kbd>
</div>
<div class="box">
<button onclick="cmd('space')">Space 急停</button>
<button onclick="cmd('x')">X 恢复</button>
<button onclick="cmd('r')">R 重捕获保持</button>
</div>
<div class="box" id="status">连接中…</div>
<p style="font-size:12px;color:#999">按住键盘上的按键即可连续移动;松开平滑保持。
页面失焦自动进入保持。急停后需按 X 恢复。方向键需要先点一下页面获得焦点。</p>
<script>
const MAP={w:'w',s:'s',a:'a',d:'d',ArrowUp:'up',ArrowDown:'down',
ArrowLeft:'left',ArrowRight:'right'};
let keys=new Set();
function send(){fetch('/keys',{method:'POST',body:JSON.stringify(
{keys:[...keys]})});paint();}
function paint(){for(const k of ['w','s','a','d','up','down','left','right']){
const el=document.getElementById('k-'+k);if(el)el.className=keys.has(k)?'on':'';}}
function drop(){if(keys.size){keys=new Set();send();}}
addEventListener('keydown',e=>{const k=MAP[e.key];
if(k){e.preventDefault();if(!keys.has(k)){keys.add(k);send();}}
else if(e.key===' '){e.preventDefault();cmd('space');}
else if(e.key.toLowerCase()==='x'){cmd('x');}
else if(e.key.toLowerCase()==='r'){cmd('r');}});
addEventListener('keyup',e=>{const k=MAP[e.key];
if(k){e.preventDefault();keys.delete(k);send();}});
addEventListener('blur',drop);
document.addEventListener('visibilitychange',()=>{if(document.hidden)drop();});
function cmd(c){fetch('/cmd',{method:'POST',body:JSON.stringify({cmd:c})});}
setInterval(()=>fetch('/hb',{method:'POST',body:'{}'}),500);
setInterval(async()=>{try{const r=await fetch('/status');const s=await r.json();
document.getElementById('status').textContent=s.text;
document.getElementById('status').className='box '+
(s.estop?'warn':(s.ok?'ok':'warn'));}catch(e){
document.getElementById('status').textContent='连接断开,已进入保持';}},500);
send();
</script></body></html>"""


class WebKeyboardNode(Node):

    def __init__(self):
        super().__init__("auv_web_keyboard")
        # 参数与 keyboard_node 同名同默认
        for k, d in (
                ("keyboard.surge_speed", 0.30), ("keyboard.sway_speed", 0.25),
                ("keyboard.heave_speed", 0.25), ("keyboard.yaw_rate_deg_s", 30.0),
                ("keyboard.yaw_rate_sign", 1.0), ("keyboard.max_goal_dist", 1.5),
                ("keyboard.max_goal_att_deg", 45.0),
                ("keyboard.state_timeout_s", 0.3),
                ("keyboard.goal_rate_hz", 5.0), ("keyboard.tick_hz", 20.0),
                ("keyboard.workspace_min", [-2.5, -1.0, -2.5]),
                ("keyboard.workspace_max", [2.5, 2.0, 2.5]),
                ("keyboard.yaw_axis", "body_up"),
                ("keyboard.state_topic", "/auv/state/control"),
                ("keyboard.goal_topic", "/auv/control/goal"),
                ("keyboard.http_port", 8791)):
            self.declare_parameter(k, d)
        g = lambda n: self.get_parameter(n).value
        self.gm = GoalManager(
            surge_speed=float(g("keyboard.surge_speed")),
            sway_speed=float(g("keyboard.sway_speed")),
            heave_speed=float(g("keyboard.heave_speed")),
            yaw_rate_deg_s=float(g("keyboard.yaw_rate_deg_s")) *
                float(g("keyboard.yaw_rate_sign")),
            max_goal_dist=float(g("keyboard.max_goal_dist")),
            max_goal_att_deg=float(g("keyboard.max_goal_att_deg")),
            workspace_min=list(g("keyboard.workspace_min")),
            workspace_max=list(g("keyboard.workspace_max")),
            state_timeout=float(g("keyboard.state_timeout_s")),
            yaw_axis=str(g("keyboard.yaw_axis")))

        qos = qos_profile_sensor_data
        self.state_frame = "auv_world"
        self.last_state_mono = 0.0
        self.estop = False
        self.auto_test_until = 0.0
        self.lock = threading.Lock()
        self.ui_keys = set()
        self.last_hb = 0.0
        self.last_keys = 0.0
        self.link_note = "等待面板连接"

        self.pub_goal = self.create_publisher(
            PoseStamped, str(g("keyboard.goal_topic")), 10)
        self.pub_estop = self.create_publisher(Bool, "/auv/control/estop", 10)
        self.create_subscription(Odometry, str(g("keyboard.state_topic")),
                                 self._on_state, qos)
        self.create_subscription(String, "/auv/training/experiment_id",
                                 self._on_exp, 10)
        self.cli = self.create_client(SetBool, "/mppi_controller_node/estop")

        self.tick_hz = max(5.0, float(g("keyboard.tick_hz")))
        self.goal_period = 1.0 / max(0.5, float(g("keyboard.goal_rate_hz")))
        self.last_goal_pub = 0.0
        self.create_timer(1.0 / self.tick_hz, self.tick)

        port = int(g("keyboard.http_port"))
        self.http = ThreadingHTTPServer(("127.0.0.1", port), self._handler())
        threading.Thread(target=self.http.serve_forever, daemon=True).start()
        self.get_logger().info(
            "web 键盘面板: ssh -L %d:127.0.0.1:%d liuhao@172.30.30.1 后访问 "
            "http://127.0.0.1:%d" % (port, port, port))

    # ------------------------------------------------------------ ROS
    def _on_state(self, msg):
        pp = msg.pose.pose.position
        qq = msg.pose.pose.orientation
        age = time.monotonic() - self.last_state_mono
        self.last_state_mono = time.monotonic()
        self.state_frame = msg.header.frame_id or self.state_frame
        self.gm.set_state((pp.x, pp.y, pp.z), (qq.x, qq.y, qq.z, qq.w), age)

    def _on_exp(self, msg):
        self.auto_test_until = time.monotonic() + 5.0

    def _set_estop(self, on):
        self.estop = on
        m = Bool()
        m.data = on
        self.pub_estop.publish(m)
        if self.cli.service_is_ready():
            req = SetBool.Request()
            req.data = on
            self.cli.call_async(req)
        self.get_logger().warn("急停 %s" % ("触发" if on else "解除"))

    def _cmd(self, c):
        if c == "space":
            self._set_estop(True)
        elif c == "x":
            self._set_estop(False)
        elif c == "r":
            ok = self.gm.recapture()
            self.get_logger().info("重捕获: %s" % ("成功" if ok else "无状态"))

    # ------------------------------------------------------------ HTTP
    def _handler(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _json(self, obj, code=200):
                b = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

            def do_GET(self):
                if self.path == "/":
                    b = PAGE.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(b)))
                    self.end_headers()
                    self.wfile.write(b)
                elif self.path == "/status":
                    node._json(node.status_dict())
                else:
                    self.send_error(404)

            def do_POST(self):
                n = int(self.headers.get("Content-Length", 0))
                try:
                    data = json.loads(self.rfile.read(n) or b"{}")
                except Exception:                     # noqa: BLE001
                    data = {}
                now = time.monotonic()
                with node.lock:
                    if self.path == "/hb":
                        node.last_hb = now
                    elif self.path == "/keys":
                        node.last_keys = now
                        node.last_hb = now
                        ks = {str(k) for k in data.get("keys", [])
                              if str(k) in VALID_KEYS}
                        node.ui_keys = ks
                    elif self.path == "/cmd":
                        node.last_hb = now
                        node._cmd(str(data.get("cmd", "")))
                    else:
                        self.send_error(404)
                        return
                self._json({"ok": True})

        return H

    # ------------------------------------------------------------ 逻辑
    def effective_keys(self):
        now = time.monotonic()
        with self.lock:
            hb = self.last_hb
            ks = self.last_keys
            ui = set(self.ui_keys)
        if now - hb > HB_TIMEOUT:
            self.link_note = "断连 %.1fs -> 保持" % (now - hb)
            return set()
        if now - ks > KEYS_TIMEOUT:
            self.link_note = "键集超时 -> 保持"
            return set()
        self.link_note = "在线"
        return ui

    def tick(self):
        now = time.monotonic()
        keys = self.effective_keys()
        surge = (+1 if "w" in keys else 0) + (-1 if "s" in keys else 0)
        sway = (+1 if "a" in keys else 0) + (-1 if "d" in keys else 0)
        heave = (+1 if "up" in keys else 0) + (-1 if "down" in keys else 0)
        yaw = (+1 if "left" in keys else 0) + (-1 if "right" in keys else 0)
        blocked = now < self.auto_test_until
        if self.estop or blocked:
            self.gm.update(0.0, (0, 0, 0), 0.0)
        else:
            self.gm.update(1.0 / self.tick_hz, (sway, heave, surge), yaw)

        if self.gm.goal_initialized and now - self.last_goal_pub >= self.goal_period:
            self.last_goal_pub = now
            g = PoseStamped()
            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = self.state_frame
            g.pose.position.x, g.pose.position.y, g.pose.position.z = self.gm.goal_p
            g.pose.orientation.x, g.pose.orientation.y, \
                g.pose.orientation.z, g.pose.orientation.w = self.gm.goal_q
            self.pub_goal.publish(g)

    def status_dict(self):
        state_age = time.monotonic() - self.last_state_mono
        gp = self.gm.goal_p or (0, 0, 0)
        blocked = time.monotonic() < self.auto_test_until
        keys = self.effective_keys()
        if self.estop:
            note = "!! 急停中(按 X 恢复)!!"
        elif blocked:
            note = "自动测试运行中,手动互斥(目标冻结)"
        elif state_age > 1.0:
            note = "状态断流 %.1fs(目标冻结)" % state_age
        else:
            note = self.gm.last_update_note or "保持"
        sp = self.gm.state_p or (0, 0, 0)
        return {
            "estop": self.estop, "ok": not self.estop,
            "text": ("[%s] %s\n当前位置 (%.2f, %.2f, %.2f)\n"
                     "目标位置 (%.2f, %.2f, %.2f)\n激活键: %s") % (
                         self.link_note, note, sp[0], sp[1], sp[2],
                         gp[0], gp[1], gp[2],
                         " ".join(sorted(keys)) or "-"),
        }


def main(args=None):
    rclpy.init(args=args)
    node = WebKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.http.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
