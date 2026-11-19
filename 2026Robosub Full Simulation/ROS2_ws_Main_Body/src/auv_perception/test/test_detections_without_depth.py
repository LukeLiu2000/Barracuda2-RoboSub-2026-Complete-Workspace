import os
from pathlib import Path
import time
import json
import numpy as np
import pytest
from auv_perception.image_detections import summarize_masks


def test_mask_summary_uses_bounds_and_no_cloud_input():
    mask=np.zeros((100,200),bool);mask[10:90,20:25]=True;mask[10:15,20:180]=True
    result=summarize_masks([mask],['Gate'],[.9],200,100)
    assert result[0]['bbox']==[20,10,180,90]
    assert result[0]['confidence']==.9
    assert summarize_masks([],[],[],200,100)==[]


@pytest.mark.skipif(os.environ.get('AUV_2D_NO_DEPTH_TEST')!='1',reason='isolated CUDA/ROS integration')
def test_actual_yolo_publishes_2d_and_jpeg_without_any_cloud_or_state():
    assert int(os.environ.get('ROS_DOMAIN_ID','0'))>=60
    import rclpy
    import cv2
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo,CompressedImage
    from std_msgs.msg import String
    from auv_perception.node import Perception
    root=Path(__file__).resolve().parents[1]
    config=root/'config/perception_gui.yaml'
    rclpy.init(args=['--ros-args','-p','config:='+str(config)])
    perception=Perception();source=rclpy.create_node('rgb_only_test_source')
    images=source.create_publisher(Image,perception.cfg['topics']['image'],qos_profile_sensor_data)
    infos=source.create_publisher(CameraInfo,perception.cfg['topics']['camera_info'],qos_profile_sensor_data)
    detections=[];jpeg=[]
    source.create_subscription(String,'/auv/perception/detections_2d',lambda m:detections.append(json.loads(m.data)),10)
    source.create_subscription(CompressedImage,'/auv/perception/overlay/compressed',lambda m:jpeg.append(m),qos_profile_sensor_data)
    executor=SingleThreadedExecutor();executor.add_node(perception);executor.add_node(source)
    image=cv2.imread(str(root/'test/fixtures/gate_frame.png'));assert image is not None
    h,w=image.shape[:2];sent=0;last=0;end=time.monotonic()+20
    try:
        while time.monotonic()<end and (not jpeg or not detections):
            now=time.monotonic()
            if now-last>.3:
                last=now;sent+=1
                msg=Image();msg.header.frame_id='test_optical';msg.header.stamp.sec=10+sent
                msg.width=w;msg.height=h;msg.encoding='bgr8';msg.step=w*3;msg.data=image.tobytes()
                info=CameraInfo();info.header=msg.header;info.width=w;info.height=h
                info.p=[1000.,0.,w/2,0.,0.,1000.,h/2,0.,0.,0.,1.,0.]
                images.publish(msg);infos.publish(info)
            executor.spin_once(timeout_sec=.01)
        assert detections and jpeg
        assert not detections[-1]['matched_cloud'] and detections[-1]['detections']
        assert any(d['label']=='Gate' for d in detections[-1]['detections'])
        assert cv2.imdecode(np.frombuffer(bytes(jpeg[-1].data),np.uint8),1).shape[:2]==(270,480)
        assert not source.get_publishers_info_by_topic('/auv/control/goal')
    finally:
        executor.remove_node(perception);executor.remove_node(source)
        perception.destroy_node();source.destroy_node();executor.shutdown();rclpy.try_shutdown()
