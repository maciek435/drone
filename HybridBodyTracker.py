from regulator import KalmanCV1D
from math import sqrt

class HybridBodyTracker:
    def __init__(self, detect_every_n=3, gate_radius=30, confirm_frames=3, max_misses=15):
        self.detect_every_n = detect_every_n
        self.gate_radius = gate_radius
        self.confirm_frames = confirm_frames
        self.max_misses = max_misses

        self.filter_x = KalmanCV1D(q=0.3, r=1.0)
        self.filter_y = KalmanCV1D(q=0.3, r=1.0)

        self.state = "LOST"
        self.frame_idx = 0

        self.candidate_pos = None
        self.candidate_count = 0

    def is_same_target(self, det_x, det_y, pred_x, pred_y, gate_radius):
        if self.distance(det_x, det_y, pred_x, pred_y) <= gate_radius:
            return True
        return False
    
    def distance(self, x1, y1, x2, y2):
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))


    def _update_lost(self, det_x, det_y):
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
            return True
        else:
            return False

    def _update_locked(self, det_x, det_y):
        locked = True
        self.filter_x.predict()
        self.filter_y.predict()
        pred_x = self.filter_x.x
        pred_y = self.filter_y.x

        if det_x is not None and self.is_same_target(det_x, det_y, pred_x, pred_y, self.gate_radius):
            self.filter_x.correct(det_x)
            self.filter_y.correct(det_y)
            locked = True
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
            

