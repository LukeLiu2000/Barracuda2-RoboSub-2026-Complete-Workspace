#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fg_probe —— 服务器端自检:直连 foxglove_bridge,统计话题实际到达率。

不装本地 Foxglove 也能回答"曲线到底会不会出":绕过 ros2 topic list
(它只证明话题存在,不证明 bridge 能收到,尤其 BEST_EFFORT 话题)。

用法(服务器上,需 pip install --user --break-system-packages websockets):
  source /opt/ros/jazzy/setup.bash
  python3 $(ros2 pkg prefix auv_bringup)/share/auv_bringup/scripts/fg_probe.py \
      8765 /auv/state/control,/auv/pid/effective_goal 5

msg_count>0 才代表数据真的能流到 Foxglove 客户端。
"""

import asyncio
import json
import struct
import sys
import time

import websockets


async def run(port, topics, dur):
    counts = {t: 0 for t in topics}
    first_ts = {}
    async with websockets.connect(
            "ws://127.0.0.1:%d" % port, max_size=None,
            subprotocols=["foxglove.sdk.v1"]) as ws:
        channels = {}
        idmap = {}
        sent = False
        deadline = time.monotonic() + dur
        while time.monotonic() < deadline:
            try:
                raw = await asyncio.wait_for(
                    ws.recv(), timeout=max(0.1, deadline - time.monotonic()))
            except (asyncio.TimeoutError,
                    websockets.exceptions.ConnectionClosed):
                break
            if isinstance(raw, str):
                msg = json.loads(raw)
                if msg.get("op") == "advertise":
                    for ch in msg.get("channels", []):
                        channels[ch["topic"]] = ch["id"]
                    subs = []
                    for i, t in enumerate(topics):
                        if t in channels:
                            subs.append({"id": i + 1,
                                         "channelId": channels[t]})
                            idmap[i + 1] = t
                    if subs and not sent:
                        await ws.send(json.dumps(
                            {"op": "subscribe", "subscriptions": subs}))
                        sent = True
            elif len(raw) >= 13 and raw[0] == 0x01:   # MessageData
                sid = struct.unpack_from("<I", raw, 1)[0]
                ts = struct.unpack_from("<Q", raw, 5)[0]
                t = idmap.get(sid)
                if t is not None:
                    counts[t] += 1
                    first_ts.setdefault(t, ts)
        print(json.dumps({
            "channel_id": {t: channels.get(t) for t in topics},
            "msg_count": counts,
            "first_ts_ns": first_ts}, ensure_ascii=False, indent=1))


if __name__ == "__main__":
    asyncio.run(run(int(sys.argv[1]), sys.argv[2].split(","),
                    float(sys.argv[3])))
