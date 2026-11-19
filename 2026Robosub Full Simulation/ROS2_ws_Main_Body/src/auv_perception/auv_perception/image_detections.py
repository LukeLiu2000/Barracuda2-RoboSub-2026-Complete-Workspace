"""Small 2D summaries independent of depth, cloud validity or 3D fitting."""
import numpy as np


def summarize_masks(masks, labels, scores, width, height):
    detections = []
    for mask, label, confidence in zip(masks, labels, scores):
        y, x = np.where(np.asarray(mask, dtype=bool))
        if not len(x): continue
        bbox = [int(x.min()), int(y.min()), int(x.max()+1), int(y.max()+1)]
        detections.append(dict(label=str(label), confidence=float(confidence), bbox=bbox,
                               mask_fraction=float(len(x)/(width*height)),
                               clipped=bool(bbox[0]<=1 or bbox[1]<=1 or bbox[2]>=width-1 or bbox[3]>=height-1)))
    return detections
