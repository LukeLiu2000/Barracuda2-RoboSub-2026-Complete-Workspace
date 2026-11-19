# -*- coding: utf-8 -*-
"""auv_common —— RoboSub AUV 公共库。

体坐标系约定(与 auv_controller_MPPI / auv_isaac_bridge 一致):
  +x 左舷(port), +y 上(up), +z 前(forward, 艇体纵轴)。
  roll 绕 z, pitch 绕 x, yaw 绕 y。
四元数一律 (x, y, z, w), 右手系, 旋转方向 body->world(R 将体系向量映到世界系)。
"""

__version__ = "1.0.0"
