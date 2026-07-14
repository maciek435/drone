# ---- Hybrid Body Tracker ----
TRACKER_DETECT_EVERY_N = 3
TRACKER_GATE_RADIUS = 30
TRACKER_CONFIRM_FRAMES = 3
TRACKER_MAX_MISSES = 15
TRACKER_KALMAN_Q = 0.3
TRACKER_KALMAN_R = 1.0

# ---- Filtr Z (KalmanLite, odleglosc) ----
FILTER_Z_PROCESS_NOISE = 0.1
FILTER_Z_MEASUREMENT_NOISE = 3.0

# ---- Regulator X (obrot/yaw) ----
REG_X_KP = 0.4
REG_X_KD = 0.3
REG_X_MAX_OUTPUT = 40

# ---- Regulator Y (gora/dol) ----
REG_Y_KP = 0.5
REG_Y_KD = 0.3
REG_Y_MAX_OUTPUT = 40

# ---- Regulator Z (przod/tyl) ----
REG_Z_KP = 1.4
REG_Z_MAX_JUMP = 200
REG_Z_MAX_OUTPUT = 40
MIN_HTORS_PX = 200

# --- Other ---
DEADZONE = 5