from regulator import KalmanCV1D
from math import sqrt

class HybridBodyTracker:
    def __init__(self, detect_fn, detect_every_n=3, gate_radius=30, confirm_frames=3, max_misses=15, q=0.3, r=1.0):
        self.detect_fn = detect_fn
        self.detect_every_n = detect_every_n
        self.gate_radius = gate_radius
        self.confirm_frames = confirm_frames
        self.max_misses = max_misses

        self.filter_x = KalmanCV1D(q=q, r=r)
        self.filter_y = KalmanCV1D(q=q, r=r)

        self.state = "LOST"
        self.frame_idx = 0

        self.candidate_pos = None
        self.candidate_count = 0

        self.cv_tracker = None
        
    def is_same_target(self, det_x, det_y, pred_x, pred_y, gate_radius):
        if self.distance(det_x, det_y, pred_x, pred_y) <= gate_radius:
            return True
        return False
    
    def distance(self, x1, y1, x2, y2):
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    
    def _make_cv_tracker(self):
        import cv2
        return cv2.legacy.TrackerMOSSE_create()

    def _update_lost(self, det_x, det_y, det_bbox, frame):
        if det_x is None:
            self.candidate_pos = None
            self.candidate_count = 0
            return False
        
        if self.candidate_pos is None:
            self.candidate_pos = (det_x, det_y)
            self.candidate_count = 1
        else:
            if self.is_same_target(det_x, det_y, self.candidate_pos[0], self.candidate_pos[1], self.gate_radius):
                self.candidate_count += 1
            else:
                self.candidate_count = 1
                self.candidate_pos = (det_x, det_y)
            
        if self.candidate_count >= self.confirm_frames:
            self.filter_x.init(det_x)
            self.filter_y.init(det_y)
            self.state = "LOCKED"
            self.cv_tracker = self._make_cv_tracker()
            self.cv_tracker.init(frame, det_bbox)
            
            return True
        else:
            return False

    def _update_locked(self, det_x, det_y, frame, det_bbox):
        locked = True

        track_ok, tbox = self.cv_tracker.update(frame)
        if track_ok:
            tcx = tbox[0] + tbox[2] / 2
            tcy = tbox[1] + tbox[3] / 2
        else:
            tcx, tcy = None, None

        self.filter_x.predict()
        self.filter_y.predict()
        pred_x = self.filter_x.x
        pred_y = self.filter_y.x

        if det_x is not None and self.is_same_target(det_x, det_y, pred_x, pred_y, self.gate_radius):
            self.filter_x.correct(det_x)
            self.filter_y.correct(det_y)
            self.cv_tracker = self._make_cv_tracker()
            self.cv_tracker.init(frame, det_bbox)
            locked = True
        elif det_x is None and track_ok:
            self.filter_x.correct(tcx)
            self.filter_y.correct(tcy)
        else:
            self.filter_x.misses += 1
            self.filter_y.misses += 1
            
        if self.filter_x.misses > self.max_misses:
            self.state = "LOST"
            self.candidate_count = 0
            self.candidate_pos = None
            locked = False
        
        cx = self.filter_x.x
        cy = self.filter_y.x
    
        return cx, cy, locked
    
    def update(self, frame):
        self.frame_idx += 1
        run_detection = (self.state !="LOCKED") or (self.frame_idx % self.detect_every_n == 0)
        if run_detection:
            result = self.detect_fn(frame)
            if result is None:
                det_x, det_y, det_bbox = None, None, None
            else:
                det_x, det_y, det_bbox = result
        else:
            det_x, det_y, det_bbox = None, None, None

        if self.state == "LOCKED" :
            cx, cy, locked = self._update_locked(det_x, det_y, frame, det_bbox)
        else:
            confirmed = self._update_lost(det_x, det_y, det_bbox, frame) 
            if confirmed:
                cx = self.filter_x.x
                cy = self.filter_y.x
                locked = True
            else:
                cx = None
                cy = None
                locked = False

        return cx, cy, locked

    def reset(self):
        self.state = "LOST"
        self.candidate_pos = None
        self.candidate_count = 0
        self.cv_tracker = None
            

