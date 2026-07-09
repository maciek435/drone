from regulator import KalmanCV1D
from math import sqrt

GATE_RADIUS = 30

filter_x = KalmanCV1D(q=0.3, r=1.0)
filter_y = KalmanCV1D(q=0.3, r=1.0)

filter_x.init(100)
filter_y.init(50)

sequence = [106, 112, 118, 500, 130]

# -----------------------
# adaptacyjny gate
# effective_gate = gate_radius * (1 + misses * 0.2)
# -------------------------------


def distance(x1, y1, x2, y2):
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))

def is_same_target(det_x, det_y, pred_x, pred_y, gate_radius):
    if distance(det_x, det_y, pred_x, pred_y) <= gate_radius:
        return True
    return False

for i, det_x in enumerate(sequence):
    filter_x.predict()
    pred_x = filter_x.x

    if is_same_target(det_x, 0, pred_x, 0, GATE_RADIUS):
        filter_x.correct(det_x)
    
    print(f"klatka {i+1}: det={det_x}  pred={pred_x:.2f}  x_po={filter_x.x:.2f}  "
        f"przyjeto={is_same_target(det_x, 0, pred_x, 0, GATE_RADIUS)}")


