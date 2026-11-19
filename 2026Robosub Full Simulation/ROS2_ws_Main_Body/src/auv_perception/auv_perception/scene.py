"""World-fixed persistent geometry; body arrows use +Z forward, not Marker +X."""
import json
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
from ament_index_python.packages import get_package_share_directory
from .node import pose_matrix
from .core import transform


def point(v):return Point(x=float(v[0]),y=float(v[1]),z=float(v[2]))


class Scene(Node):
    def __init__(self):
        super().__init__('auv_perception_scene');share=Path(get_package_share_directory('auv_perception'))
        self.declare_parameter('config',str(share/'config/perception.yaml'))
        cfg=yaml.safe_load(Path(self.get_parameter('config').value).read_text());self.frame=cfg['world_frame']
        self.templates={}
        for n in cfg['templates']:
            path=Path(n) if Path(n).is_absolute() else share/'config/templates'/n
            t=yaml.safe_load(path.read_text());self.templates[t['id']]=t
        self.state=None;self.goal=None;self.goal_received=0;self.state_received=0;self.objects={};self.last_ids=set()
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(MarkerArray,'/auv/perception/scene',retained)
        self.create_subscription(Odometry,'/auv/state/control',self.on_state,qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/auv/pid/effective_goal',self.on_goal,10)
        self.create_subscription(String,'/auv/perception/object_map',self.on_map,retained)
        self.create_timer(.2,self.draw,clock=Clock(clock_type=ClockType.STEADY_TIME))
    def on_state(self,m):
        if m.header.frame_id==self.frame:self.state=pose_matrix(m.pose.pose);self.state_received=time.monotonic()
    def on_goal(self,m):
        if m.header.frame_id==self.frame:self.goal=pose_matrix(m.pose);self.goal_received=time.monotonic()
    def on_map(self,m):
        data=json.loads(m.data)
        if data['frame_id']==self.frame:self.objects=data['objects']
    def make(self,namespace,id,kind,color):
        m=Marker();m.header.frame_id=self.frame;m.header.stamp=self.get_clock().now().to_msg();m.ns=namespace;m.id=id;m.type=kind;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color.r,m.color.g,m.color.b,m.color.a=color;m.frame_locked=False
        return m
    def text(self,namespace,id,p,label,color):
        m=self.make(namespace,id,Marker.TEXT_VIEW_FACING,color);m.pose.position=point(p);m.scale.z=.13;m.text=label;return m
    def draw(self):
        markers=[];white=(1.,1.,1.,1.);now=time.monotonic()
        for name,T,received,color in [('current',self.state,self.state_received,(1.,1.,0.,1.)),('target',self.goal,self.goal_received,(0.,1.,0.,1.))]:
            if T is None or now-received>1.:continue
            # At rest current and target coincide: different display lengths and
            # shaft widths keep both colors visible without moving their origins.
            length=.6 if name=='current' else .85
            m=self.make(name,0,Marker.ARROW,color);m.points=[point(T[:3,3]),point(transform(T,[[0,0,length]])[0])]
            m.scale.x=.045 if name=='current' else .025;m.scale.y=.09 if name=='current' else .07;m.scale.z=.13;markers.append(m)
            markers.append(self.text(name,1,T[:3,3]+[0,0,.2 if name=='current' else .4],name+' state',color))
            up=self.make(name,2,Marker.LINE_LIST,color);up.scale.x=.015;up.points=[point(T[:3,3]),point(transform(T,[[0,.16,0]])[0])];markers.append(up)
        for key,record in self.objects.items():
            if key not in self.templates:continue
            t=self.templates[key];T=np.array(record['T']);m=self.make(key,0,Marker.LINE_LIST,white);m.scale.x=.035
            m.points=[point(v) for v in transform(T,np.array(t['display_lines']).reshape(-1,3))];markers.append(m)
            age=max(0,self.get_clock().now().nanoseconds/1e9-record['last_accepted']);label=t['title']+(' | saved %.1fs'%age if age>2 else ' | tracking')
            markers.append(self.text(key,1,transform(T,[[0,-.25,0]])[0],label,white))
            for i,item in enumerate(t.get('labels',[])):
                center=np.array(item['position']);cross=[]
                for direction in [np.array([.07,.07,0]),np.array([.07,-.07,0])]:cross.extend([center-direction,center+direction])
                m=self.make(key,10+i*2,Marker.LINE_LIST,white);m.scale.x=.02;m.points=[point(v) for v in transform(T,cross)];markers.append(m)
                markers.append(self.text(key,11+i*2,transform(T,[center+[0,-.12,0]])[0],item['text'],white))
        ids={(m.ns,m.id) for m in markers}
        for namespace,id in self.last_ids-ids:
            m=self.make(namespace,id,Marker.ARROW,white);m.action=Marker.DELETE;markers.append(m)
        self.last_ids=ids;self.pub.publish(MarkerArray(markers=markers))


def main(args=None):
    rclpy.init(args=args);n=Scene()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
