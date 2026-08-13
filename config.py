# config.py

# ── Kamera ────────────────────────────────────────────────────────────────────
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

# ── Flask ─────────────────────────────────────────────────────────────────────
FLASK_HOST = "0.0.0.0"   
FLASK_PORT = 5000
JPEG_QUALITY = 30        # 1-100, wyżej = lepsza jakość ale wolniejszy stream

# ── MediaPipe ─────────────────────────────────────────────────────────────────
MP_DETECTION_CONFIDENCE = 0.5
MP_TRACKING_CONFIDENCE  = 0.5
MP_MODEL_COMPLEXITY     = 0    # 0 = najszybszy 

# ── Body Tracker ─────────────────────────────────────────────────────────────
TRACKER_DETECT_EVERY_N = 3
TRACKER_GATE_RADIUS = 30
TRACKER_CONFIRM_FRAMES = 3
TRACKER_MAX_MISSES = 15
TRACKER_KALMAN_Q = 0.3
TRACKER_KALMAN_R = 1.0

HEIGHT_GATE_THRESHOLD = 30 

#──── Filtr Kalmana Line - oś Z ────────────────────────────────────────────
FILTER_Z_PROCESS_NOISE = 0.1
FILTER_Z_MEASUREMENT_NOISE = 3.0

# ── Regulator PD — oś X (yaw) ─────────────────────────────────────────────────
REG_X_KP         = 0.9
REG_X_KD         = 0.3
REG_X_MAX_OUTPUT = 150

# ── Regulator PD — oś Y (góra/dół) ──────────────────────────────────
REG_Y_KP         = 0.3
REG_Y_KD         = 0.5
REG_Y_MAX_OUTPUT = 40

# ── Regulator PD — oś Z (dystans) ────────────────────────────────────
REG_Z_KP = 5.0 
REG_Z_KD = 0.05
REG_Z_MAX_JUMP = 50
REG_Z_MAX_OUTPUT = 500
MIN_HTORS_PX = 100

# ── Deadzone (prostokąt na ekranie) ───────────────────────────────────────────
DEADZONE_W = 5
DEADZONE_H = 5

#───── Gimbal ───────────────────────────────────────────────────────────────────
GIMBAL_SERVO_PIN = 12  
GIMBAL_CENTER_ANGLE = 90
GIMBAL_MIN_ANGLE = 30
GIMBAL_MAX_ANGLE = 150

# ── UART — komunikacja z F405 (iNAV) ──────────────────────────────────────────
UART_PORT     = "/dev/serial0"  
UART_BAUDRATE = 115200

# ── INNE ───────────────────────
MAX_DATA_AGE_S = 0.3 