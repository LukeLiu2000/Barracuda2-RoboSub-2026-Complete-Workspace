"""ROS consumer only. Never publishes commands or starts an SDK camera."""
import json
import os
import queue
import selectors
import subprocess
import tempfile
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.clock import Clock, ClockType
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock as ClockMsg
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from scipy.ndimage import binary_erosion
from .core import SOLVERS,StateBuffer,LandmarkStore,project_membership,clean_points
from .image_detections import summarize_masks
from .capture_clock import CaptureClock


def ns(stamp):return stamp.sec*1000000000+stamp.nanosec
def pose_matrix(pose):
    T=np.eye(4);q=pose.orientation
    T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    T[:3,3]=[pose.position.x,pose.position.y,pose.position.z];return T


class Perception(Node):
    def __init__(self):
        super().__init__('auv_perception')
        share=Path(get_package_share_directory('auv_perception'))
        self.declare_parameter('config',str(share/'config/perception.yaml'))
        self.cfg=yaml.safe_load(Path(self.get_parameter('config').value).read_text())
        self.tasks=[]
        for name in self.cfg['templates']:
            path=Path(name) if Path(name).is_absolute() else share/'config/templates'/name
            template=yaml.safe_load(path.read_text())
            if template['schema_version']!=1:raise ValueError('Unknown template schema')
            self.tasks.append((template,SOLVERS[template['solver']](template,self.cfg['fit'])))
        self.states=StateBuffer();self.store=LandmarkStore();self.images={};self.clouds={};self.infos={}
        self.lock=threading.Lock();self.jobs=queue.Queue(maxsize=1);self.results=queue.Queue(maxsize=2)
        self.busy=False;self.epoch=0;self.stop=threading.Event();self.worker=None;self.last_clock=None
        self.image_arrival={};self.detection_results=queue.Queue(maxsize=1)
        self.capture_clock=CaptureClock(**self.cfg.get('sensor_time_alignment',{}))
        self.last_clock_wall=None
        self.camera_to_body=np.asarray(self.cfg['camera_to_body']['matrix'],float)
        if not self.cfg['camera_to_body'].get('verified',False):raise ValueError('Camera extrinsic not verified')
        if self.camera_to_body.shape!=(4,4) or not np.allclose(np.linalg.det(self.camera_to_body[:3,:3]),1,atol=1e-4):raise ValueError('Invalid camera extrinsic')
        q=qos_profile_sensor_data
        self.create_subscription(Image,self.cfg['topics']['image'],self.on_image,q)
        self.create_subscription(PointCloud2,self.cfg['topics']['points'],self.on_cloud,q)
        self.create_subscription(CameraInfo,self.cfg['topics']['camera_info'],self.on_info,q)
        self.create_subscription(Odometry,'/auv/state/control',self.on_state,q)
        self.create_subscription(ClockMsg,'/clock',self.on_clock,q)
        retained=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub=self.create_publisher(String,'/auv/perception/object_map',retained)
        self.status_pub=self.create_publisher(String,'/auv/perception/status',10)
        self.obs_pub=self.create_publisher(String,'/auv/perception/observations',10)
        self.detection_pub=self.create_publisher(String,'/auv/perception/detections_2d',10)
        self.debug_pub=self.create_publisher(Image,'/auv/perception/overlay',q)
        self.compressed_pub=self.create_publisher(CompressedImage,'/auv/perception/overlay/compressed',q)
        self.last_overlay=0.0
        resume=os.environ.get('AUV_PERCEPTION_RESUME')
        if resume:
            self.store.records=json.loads(Path(resume).read_text())['objects']
        self.create_service(Trigger,'/auv/perception/clear_map',self.clear)
        self.create_timer(.1,self.poll,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(1.,self.publish_map,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.thread=threading.Thread(target=self.process,daemon=True);self.thread.start()
        self.get_logger().info('Perception ready; waiting for same-stamp left RGB + SDK XYZ + state. No control outputs.')

    def clear(self,req,resp):
        self.epoch+=1;self.store=LandmarkStore();self.publish_map();resp.success=True;resp.message='Landmark session cleared';return resp
    def on_clock(self,msg):
        t=ns(msg.clock)
        if self.last_clock is not None and t<self.last_clock-100000000:
            self.clear(None,type('Response',(),{})());self.states=StateBuffer();self.images.clear();self.clouds.clear();self.infos.clear()
            self.image_arrival.clear();self.capture_clock.reset()
        self.last_clock=t;self.last_clock_wall=time.monotonic()
    def on_state(self,msg):
        if msg.header.frame_id!=self.cfg['world_frame'] or msg.child_frame_id!='auv_com':return
        self.states.add(ns(msg.header.stamp)/1e9,pose_matrix(msg.pose.pose))
    def cache(self,cache,msg):
        cache[ns(msg.header.stamp)]=msg
        while len(cache)>8:del cache[min(cache)]
        self.try_job()
    def on_image(self,msg):
        if self.capture_clock.enabled:
            # Calibrate only at RGB arrival with a fresh bridge clock, BEFORE inference.
            if self.last_clock is not None and self.last_clock_wall is not None and time.monotonic()-self.last_clock_wall<=.3:
                self.capture_clock.observe(ns(msg.header.stamp)/1e9,self.last_clock/1e9)
        self.image_arrival[ns(msg.header.stamp)]=time.monotonic()
        while len(self.image_arrival)>8:del self.image_arrival[min(self.image_arrival)]
        self.cache(self.images,msg)
    def on_cloud(self,msg):self.cache(self.clouds,msg)
    def on_info(self,msg):self.cache(self.infos,msg)
    def try_job(self):
        if self.busy:return
        common=self.images.keys()&self.infos.keys()
        if not common:return
        stamp=max(common);image=self.images[stamp];cloud=self.clouds.get(stamp);info=self.infos[stamp]
        # Briefly wait for matching SDK XYZ, but 2D inference must work without depth.
        if cloud is None and time.monotonic()-self.image_arrival.get(stamp,time.monotonic())<.15:return
        if image.header.frame_id!=info.header.frame_id:return
        if cloud is not None and cloud.header.frame_id!=image.header.frame_id:cloud=None
        if (info.width,info.height)!=(image.width,image.height) or info.p[0]<=0:return
        capture=stamp/1e9 if self.last_clock is None else self.capture_clock.convert(stamp/1e9)
        if capture is None:return
        capture_ns=int(round(capture*1e9))
        Tbody=self.states.at(capture,self.cfg['state_tolerance_s'])
        self.jobs.put_nowait((self.epoch,capture_ns,image,cloud,info,None if Tbody is None else Tbody@self.camera_to_body,dict(self.store.records)))
        self.busy=True
        for t in list(self.image_arrival):
            if t<=stamp:del self.image_arrival[t]
        for cache in [self.images,self.clouds,self.infos]:
            for t in list(cache):
                if t<=stamp:del cache[t]

    def ask_worker(self,request,timeout):
        self.worker.stdin.write(json.dumps(request)+'\n');self.worker.stdin.flush()
        sel=selectors.DefaultSelector();sel.register(self.worker.stdout,selectors.EVENT_READ)
        try:
            if not sel.select(timeout):raise TimeoutError('YOLO worker timeout')
            line=self.worker.stdout.readline()
            if not line:raise RuntimeError('YOLO worker exited')
            response=json.loads(line)
            if not response.get('ok',False):raise RuntimeError(response.get('error','Worker error'))
        finally:sel.close()

    def process(self):
        try:
            worker=Path(__file__).with_name('worker.py')
            self.worker=subprocess.Popen([self.cfg['inference_python'],'-I',str(worker),self.cfg['weights']],
                stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
            sel=selectors.DefaultSelector();sel.register(self.worker.stdout,selectors.EVENT_READ)
            if not sel.select(90):raise TimeoutError('Model load timeout')
            ready=json.loads(self.worker.stdout.readline());sel.close()
            if not ready.get('ready'):raise RuntimeError(str(ready))
            with tempfile.TemporaryDirectory(prefix='auv_perception_') as temp:
                while not self.stop.is_set():
                    try:job=self.jobs.get(timeout=.2)
                    except queue.Empty:continue
                    epoch,stamp,img,cloud,info,Twc,records=job
                    try:
                        channels={'bgra8':4,'bgr8':3,'rgb8':3,'rgba8':4}.get(img.encoding)
                        if channels is None:raise ValueError('Unsupported image encoding '+img.encoding)
                        image=np.ndarray((img.height,img.width,channels),dtype=np.uint8,buffer=bytes(img.data),strides=(img.step,channels,1)).copy()[:,:,:3]
                        if img.encoding in ['rgb8','rgba8']:image=image[:,:,::-1].copy()
                        ip=str(Path(temp)/'image.npy');op=str(Path(temp)/'masks.npz');np.save(ip,image)
                        started=time.monotonic()
                        self.ask_worker({'input':ip,'output':op,'confidence':self.cfg['confidence'],'imgsz':self.cfg['imgsz'],'device':self.cfg['device']},45)
                        with np.load(op,allow_pickle=False) as data:
                            masks=data['masks'].copy();labels=data['labels'].tolist();scores=data['confidence'].tolist()
                        packet=dict(schema_version=1,epoch=epoch,stamp=stamp/1e9,frame_id=img.header.frame_id,
                                    width=img.width,height=img.height,projection=list(info.p),matched_cloud=cloud is not None,
                                    raw_sensor_stamp=ns(img.header.stamp)/1e9,sensor_time_offset_s=stamp/1e9-ns(img.header.stamp)/1e9,
                                    detections=summarize_masks(masks,labels,scores,img.width,img.height))
                        if self.detection_results.full():self.detection_results.get_nowait()
                        self.detection_results.put_nowait(packet)
                        xyz=np.empty((0,3),dtype=float)
                        if cloud is not None:
                            fields={f.name:f.offset for f in cloud.fields}
                            if not all(k in fields for k in ['x','y','z']):raise ValueError('Missing XYZ fields')
                            endian='>' if cloud.is_bigendian else '<'
                            dtype=np.dtype({'names':['x','y','z'],'formats':[endian+'f4']*3,'offsets':[fields[k] for k in ['x','y','z']],'itemsize':cloud.point_step})
                            a=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=bytes(cloud.data),strides=(cloud.row_step,cloud.point_step))
                            xyz=np.column_stack([a[k].ravel() for k in ['x','y','z']])
                        # Optional measured virtual-baseline calibration. Original SDK topic unchanged.
                        xyz=xyz*float(self.cfg.get('point_cloud_scale',1.0))
                        interior=[]
                        for mask in masks:
                            small=binary_erosion(mask,iterations=1)
                            interior.append(small if small.sum()>.4*mask.sum() else mask)
                        groups=project_membership(xyz,interior,info.p,img.width,img.height)
                        observations=[{'label':l,'confidence':float(s),'points':p} for l,s,p in zip(labels,scores,groups)]
                        fitted=[]
                        for template,solver in (self.tasks if Twc is not None else []):
                            # Keep strongest instance per part; do not merge unrelated same-class objects.
                            selected={}
                            for o in observations:
                                canonical=template.get('aliases',{}).get(o['label'],o['label'])
                                if canonical in template['parts'] and (canonical not in selected or o['confidence']>selected[canonical]['confidence']):selected[canonical]=dict(o,label=canonical)
                            covered=np.zeros((img.height,img.width),bool)
                            whole=[]
                            for index,label in enumerate(labels):
                                canonical=template.get('aliases',{}).get(label,label)
                                if canonical in template['parts']:covered|=masks[index]
                                elif canonical=='__whole__':whole.append(index)
                            if whole:
                                index=max(whole,key=lambda i:scores[i]);extra=interior[index]&~covered
                                pts=project_membership(xyz,[extra],info.p,img.width,img.height)[0]
                                selected['__whole__']={'label':'__whole__','confidence':float(scores[index]),'points':pts}
                            previous=records.get(template['id']);prior=np.linalg.inv(Twc)@np.asarray(previous['T']) if previous else None
                            result=solver.fit(list(selected.values()),prior)
                            if result:
                                Tcam=result.pop('T');fitted.append({'id':template['id'],'T_camera':Tcam.tolist(),'T_world':(Twc@Tcam).tolist(),'quality':result})
                        # Display only: preserve full-resolution inference and fitting.
                        h,w=image.shape[:2]; display=cv2.resize(image,(480,270),interpolation=cv2.INTER_AREA)
                        for i,(mask,label,score,points) in enumerate(zip(masks,labels,scores,groups)):
                            import zlib
                            hue=zlib.crc32(label.encode())%180
                            color=cv2.cvtColor(np.uint8([[[hue,210,255]]]),cv2.COLOR_HSV2BGR)[0,0]
                            small=cv2.resize(mask.astype(np.uint8),(480,270),interpolation=cv2.INTER_NEAREST).astype(bool)
                            display[small]=(.6*display[small]+.4*color).astype(np.uint8)
                            # Conservative density-cleaned candidates, not certified fit inliers.
                            pts=clean_points(points,limit=800)
                            if len(pts):
                                uv=np.c_[info.p[0]*pts[:,0]/pts[:,2]+info.p[2],info.p[5]*pts[:,1]/pts[:,2]+info.p[6]]
                                uv=np.rint(uv*np.array([480/w,270/h])).astype(int)
                                for x,y in uv:
                                    if 0<=x<480 and 0<=y<270:cv2.circle(display,(int(x),int(y)),1,tuple(int(c) for c in color),-1)
                            yy,xx=np.where(small)
                            if len(xx):
                                pos=(int(xx.min()),max(14,int(yy.min())))
                                cv2.putText(display,f'{label} {score:.2f}',pos,cv2.FONT_HERSHEY_SIMPLEX,.42,(0,0,0),3,cv2.LINE_AA)
                                cv2.putText(display,f'{label} {score:.2f}',pos,cv2.FONT_HERSHEY_SIMPLEX,.42,tuple(int(c) for c in color),1,cv2.LINE_AA)
                        image=display
                        self.results.put((epoch,stamp,fitted,image,img.header,time.monotonic()-started,None))
                    except Exception as e:self.results.put((epoch,stamp,[],None,img.header,0,str(e)))
        except Exception as e:self.results.put((self.epoch,0,[],None,None,0,'Worker startup failed: '+str(e)))

    def poll(self):
        try:
            packet=self.detection_results.get_nowait()
            if packet['epoch']==self.epoch:self.detection_pub.publish(String(data=json.dumps(packet)))
        except queue.Empty:pass
        try:epoch,stamp,fitted,image,header,duration,error=self.results.get_nowait()
        except queue.Empty:
            self.try_job();return
        self.busy=False
        if epoch!=self.epoch:return
        if error:
            self.get_logger().error(error);self.status_pub.publish(String(data=json.dumps({'error':error})));return
        commits=[]
        for result in fitted:
            commit=self.store.update(result['id'],np.array(result['T_world']),result['quality'],stamp/1e9);commits.append(commit)
        self.obs_pub.publish(String(data=json.dumps({'stamp':stamp/1e9,'frame_id':self.cfg['world_frame'],'objects':fitted,'committed':commits})))
        self.status_pub.publish(String(data=json.dumps({'stamp':stamp/1e9,'inference_and_fit_s':duration,'objects':len(fitted),'committed':commits,
            'sensor_time_offset_s':self.capture_clock.offset,'sensor_time_alignment':self.capture_clock.reason})))
        if image is not None and time.monotonic()-self.last_overlay>=1.0:
            self.last_overlay=time.monotonic()
            ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,45])
            if ok:
                msg=CompressedImage();msg.header=header;msg.format='bgr8; jpeg compressed bgr8';msg.data=encoded.tobytes();self.compressed_pub.publish(msg)
            if False:  # Raw overlay disabled to protect livestream bandwidth.
                raw=Image();raw.header=header;raw.height,raw.width=image.shape[:2];raw.encoding='bgr8';raw.step=raw.width*3;raw.data=image.tobytes();self.debug_pub.publish(raw)
        self.publish_map();self.try_job()
    def publish_map(self):
        self.map_pub.publish(String(data=json.dumps({'schema_version':1,'frame_id':self.cfg['world_frame'],'epoch':self.epoch,'objects':self.store.records})))
    def destroy_node(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.terminate()
            try:self.worker.wait(timeout=3)
            except subprocess.TimeoutExpired:self.worker.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args);node=Perception()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
