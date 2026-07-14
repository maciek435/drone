class SafetyGuard:
    def __init__(self, min_htors_px, max_misses=15):
        self.min_htors_px = min_htors_px
        self.max_misses = max_misses
        self.misses = 0


    def clamp_forward_speed(self, fwd_speed, h_tors):
        if h_tors is None:
            self.misses += 1
            if self.misses > self.max_misses:
                return 0
            else:
                return fwd_speed
        else:
            self.misses = 0
            if h_tors >= self.min_htors_px:
                return 0
            else:
                return fwd_speed
        #return fwd_speed