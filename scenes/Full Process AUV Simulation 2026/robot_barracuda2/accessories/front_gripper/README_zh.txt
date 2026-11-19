Barracuda 2 前夹爪已安装到原机器人USD，pool引用自动继承，不必新增Reference。
30cm固定杆，从机头下缘向前上翘15度，末端官方Robotiq 2F-140。原主体40kg字段不变；总装额外包含夹爪链接质量。
实际官方模型零角度内垫间距约136mm，属于标称140mm夹爪；闭合到40度空载内垫间距约7.7mm。
安装点tool_mount：X前/Y左/Z上；机器人原控制坐标系保持x左/y上/z前。
初始张开，Play后可通过barracuda_gripper_control.py调用open_gripper/close_gripper/set_open_fraction；fraction按角度插值而非毫米标定。
不包含ROS夹爪接口或自动抓取规划，不绘制鱼雷线段。
已在隔离场景验证空载开合、机器人受力随动、200g标准物体夹持与释放；用户当前GUI未重载或操控。
