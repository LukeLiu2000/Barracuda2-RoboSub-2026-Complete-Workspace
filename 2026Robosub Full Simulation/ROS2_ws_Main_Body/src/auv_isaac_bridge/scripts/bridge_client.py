#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bridge_client —— auv_isaac_bridge 的 TCP 客户端(纯标准库,系统 python3 即可)。

用法(在服务器上,任何终端):
  python3 bridge_client.py ping
  python3 bridge_client.py status
  python3 bridge_client.py stage_info
  python3 bridge_client.py robot_info
  python3 bridge_client.py test_thrust --index 0 --force 2 --dur 1
  python3 bridge_client.py test_thrust --all --force 0 --dur 0     # 清测试推力
  python3 bridge_client.py zero|estop_on|estop_off|hydro_get
  python3 bridge_client.py hydro_set '{"damping": {"linear": [4,4,4,0,0,0]}}'
  python3 bridge_client.py config_reload|stop|restart
  python3 bridge_client.py timeline --action pause [--pef false]
  python3 bridge_client.py evaluate --index 0 --force 2 --dur 1
  python3 bridge_client.py set_pose --pos 0,0,0 --quat 0,0,0,1
  python3 bridge_client.py watch            # 1Hz 轮询 status 打单行摘要

扰动力(2026-11-08;JSON 补丁一律走 --patch,注意引号):
  python3 bridge_client.py disturb_config --patch \
      '{"enabled":true,"mode":"cont","force_max_n":19.6133,"seed":1234}'
  python3 bridge_client.py disturb_config --patch \
      '{"enabled":true,"mode":"pulse","fixed_vec":[10,0,0], \
        "pulse_on_s":2,"pulse_off_s":2}'
  python3 bridge_client.py disturb_get|disturb_stop
  python3 bridge_client.py disturb_replay --patch \
      '[[2.0,[[1,0,0],[0,1,0]]], ...]' \
      # 序列项 [间隔s, [两通道体/世界系力向量]];空序列=回放本次会话记录

视窗可视化:
  python3 bridge_client.py viz_set --patch \
      '{"enabled":true,"newtons_per_meter":40,"rate_hz":10}'
  python3 bridge_client.py viz_get
  python3 bridge_client.py step_state     # 最近物理步状态快照
注:set_pose 负数坐标用等号形式:--pos=-15.36,-1.79,1.38

急停优先级:任何时刻 `estop_on` 立即清零全部输出。
"""

import argparse
import json
import socket
import sys
import time

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 17788


def call(host, port, req, timeout=10.0):
    s = socket.create_connection((host, port), timeout=timeout)
    try:
        s.sendall((json.dumps(req, ensure_ascii=False) + "\n").encode("utf-8"))
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                raise RuntimeError("连接被桥关闭")
            buf += chunk
        return json.loads(buf.split(b"\n", 1)[0].decode("utf-8"))
    finally:
        s.close()


def _flist(text):
    return [float(v) for v in text.split(",")]


def build_request(args):
    c = args.command
    if c in ("ping", "status", "stage_info", "robot_info", "zero",
             "hydro_get", "config_reload", "stop", "restart",
             "sensor_scan", "graph_scan", "disturb_stop", "disturb_get",
             "viz_get", "step_state"):
        return {"cmd": c}
    if c == "disturb_config":
        return {"cmd": "disturb_config",
                "patch": json.loads(args.patch)}
    if c == "disturb_replay":
        return {"cmd": "disturb_replay",
                "sequence": json.loads(args.patch)}
    if c == "viz_set":
        return {"cmd": "viz_set", "patch": json.loads(args.patch)}
    if c == "xform_dump":
        return {"cmd": "xform_dump",
                "paths": [p.strip() for p in args.paths.split(";") if p.strip()]}
    if c == "estop_on":
        return {"cmd": "estop", "enabled": True}
    if c == "estop_off":
        return {"cmd": "estop", "enabled": False}
    if c == "test_thrust":
        req = {"cmd": "test_thrust", "force_n": args.force, "duration_s": args.dur}
        if args.all:
            req["index"] = "all"
        else:
            req["index"] = args.index
        return req
    if c == "hydro_set":
        return {"cmd": "hydro_set", "patch": json.loads(args.patch)}
    if c == "timeline":
        req = {"cmd": "timeline", "action": args.action}
        if args.pef is not None:
            req["play_every_frame"] = args.pef.lower() in ("1", "true", "yes")
        return req
    if c == "evaluate":
        return {"cmd": "evaluate", "index": args.index, "force_n": args.force,
                "duration_s": args.dur, "settle_s": args.settle}
    if c == "set_pose":
        req = {"cmd": "set_pose"}
        if args.pos:
            req["pos_m"] = _flist(args.pos)
        if args.quat:
            req["quat_xyzw"] = _flist(args.quat)
        return req
    raise SystemExit("未知命令: %s" % c)


def watch(host, port, period=1.0):
    print("watch: Ctrl+C 退出")
    while True:
        try:
            st = call(host, port, {"cmd": "status"})
            if not st.get("ok"):
                print("status 返回异常: %s" % st.get("error"))
            else:
                sg = st.get("stage", {})
                print("sim=%-8.2f mode=%-24s cmd=%5.1fHz age=%-6s estop=%d "
                      "jumps=%d up=%s mpu=%s fscale=%s err=%d %s" % (
                          st.get("sim_time_s", 0), st.get("mode", ""),
                          st.get("cmd_rate_hz", 0),
                          st.get("cmd_age_s"), 1 if st.get("estop") else 0,
                          st.get("time_jumps", 0), sg.get("up_axis"),
                          sg.get("meters_per_unit"), sg.get("force_scale"),
                          st.get("consec_errors", 0),
                          st.get("last_error", "")[:60]))
        except Exception as exc:            # noqa: BLE001
            print("连接失败: %s" % exc)
        time.sleep(period)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", help="ping/status/stage_info/robot_info/"
                    "test_thrust/zero/estop_on/estop_off/hydro_get/hydro_set/"
                    "config_reload/timeline/set_pose/evaluate/watch/stop/restart")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--index", type=int, default=0, help="推进器下标 0..7")
    ap.add_argument("--all", action="store_true", help="test_thrust 全部推进器")
    ap.add_argument("--force", type=float, default=2.0, help="测试力 N")
    ap.add_argument("--dur", type=float, default=1.0, help="持续时间 s")
    ap.add_argument("--settle", type=float, default=0.3)
    ap.add_argument("--patch", default="{}", help="hydro_set 的 JSON 补丁")
    ap.add_argument("--action", default="play", help="play|pause|stop")
    ap.add_argument("--pef", default=None, help="play_every_frame true/false")
    ap.add_argument("--pos", default=None, help="set_pose 世界坐标 x,y,z [m]")
    ap.add_argument("--quat", default=None, help="set_pose 四元数 x,y,z,w")
    ap.add_argument("--paths", default="", help="xform_dump 路径列表,分号分隔")
    args = ap.parse_args()

    if args.command == "watch":
        try:
            watch(args.host, args.port)
        except KeyboardInterrupt:
            return
        return
    resp = call(args.host, args.port, build_request(args))
    print(json.dumps(resp, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
