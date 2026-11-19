# -*- coding: utf-8 -*-
"""在 Isaac Sim 的 Script Editor 里执行本文件(一次),启动 AUV 桥。

Script Editor 操作:Window -> Script Editor,把本文件内容粘贴进输入区,
按 Ctrl+Enter(或 Run 按钮)执行。成功标志:输出区打印
"[auv_isaac_bridge] 已启动 TCP 127.0.0.1:17788"。

之后无需再碰 Script Editor:诊断/实验全部通过
  python3 src/auv_isaac_bridge/scripts/bridge_client.py <命令>
远程驱动;改了 auv_isaac_bridge.py 后用 `restart` 命令热重启。
"""

import runpy

BRIDGE = ("/home/liuhao/LWKProjectWorkspaces/AUVIssacSIMWorkspace/"
          "2026Robosub Full Simulation/ROS2_ws_Main_Body/src/"
          "auv_isaac_bridge/isaac/auv_isaac_bridge.py")

runpy.run_path(BRIDGE, run_name="__main__")
