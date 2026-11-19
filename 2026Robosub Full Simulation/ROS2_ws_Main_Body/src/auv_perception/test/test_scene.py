"""Run only with ROS_DOMAIN_ID=61, isolated from the actual robot graph."""
import os,json,time
import numpy as np
import pytest
pytestmark=pytest.mark.skipif(os.environ.get('ROS_DOMAIN_ID')!='61',reason='requires isolated ROS_DOMAIN_ID=61')


def test_world_markers_and_frozen_gate():
    import rclpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    from auv_perception.scene import Scene
    from visualization_msgs.msg import MarkerArray,Marker
    rclpy.init();node=Scene();seen=[]
    sub=node.create_subscription(MarkerArray,'/auv/perception/scene',seen.append,10)
    state=Odometry();state.header.frame_id='auv_world';state.pose.pose.orientation.w=1.;state.pose.pose.position.x=1.
    goal=PoseStamped();goal.header.frame_id='auv_world';goal.pose.orientation.w=1.;goal.pose.position.x=2.
    node.on_state(state);node.on_goal(goal)
    T=np.eye(4);T[2,3]=4
    node.on_map(String(data=json.dumps({'frame_id':'auv_world','objects':{'task1_gate':{'T':T.tolist(),'last_accepted':0,'quality':{}}}})))
    for _ in range(10):node.draw();rclpy.spin_once(node,timeout_sec=.05)
    assert seen
    a={(m.ns,m.id):m for m in seen[-1].markers if m.action==Marker.ADD}
    assert a['current',0].color.r==1 and a['current',0].color.g==1
    assert a['target',0].color.g==1 and a['target',0].color.r==0
    arrow=a['current',0];assert abs(arrow.points[1].z-arrow.points[0].z-.6)<1e-6
    frozen=[(p.x,p.y,p.z) for p in a['task1_gate',0].points]
    state.pose.pose.position.x=5.;node.on_state(state)
    for _ in range(6):node.draw();rclpy.spin_once(node,timeout_sec=.05)
    b={(m.ns,m.id):m for m in seen[-1].markers if m.action==Marker.ADD}
    assert [(p.x,p.y,p.z) for p in b['task1_gate',0].points]==frozen
    assert b['current',0].points[0].x==5.
    node.destroy_subscription(sub);node.destroy_node();rclpy.shutdown()
