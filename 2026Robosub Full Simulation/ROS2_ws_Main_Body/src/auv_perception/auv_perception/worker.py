"""Persistent YOLO subprocess: isolated existing CUDA environment, JSON line IPC."""
import contextlib
import json
import sys


def main():
    import numpy as np
    with contextlib.redirect_stdout(sys.stderr):
        from ultralytics import YOLO
        model=YOLO(sys.argv[1])
        if model.task!='segment':raise ValueError('Segmentation weights required')
    print(json.dumps({'ready':True,'names':model.names}),flush=True)
    for line in sys.stdin:
        try:
            job=json.loads(line);image=np.load(job['input'],allow_pickle=False)
            with contextlib.redirect_stdout(sys.stderr):
                result=model.predict(image,conf=job['confidence'],imgsz=job['imgsz'],
                                     retina_masks=True,verbose=False,device=job.get('device','0'))[0]
            if result.masks is None:
                masks=np.empty((0,*image.shape[:2]),dtype=bool);labels=[];conf=[]
            else:
                masks=result.masks.data.cpu().numpy()>.5
                if tuple(masks.shape[1:])!=tuple(image.shape[:2]):raise ValueError('Mask/image shape mismatch')
                labels=[result.names[int(c)] for c in result.boxes.cls.cpu().numpy()]
                conf=result.boxes.conf.cpu().numpy()
            np.savez(job['output'],masks=masks,labels=np.asarray(labels,dtype='U80'),confidence=conf)
            print(json.dumps({'ok':True}),flush=True)
        except Exception as e:print(json.dumps({'ok':False,'error':str(e)}),flush=True)


if __name__=='__main__':main()
