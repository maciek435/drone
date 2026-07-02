# config.py

# ── Kamera ────────────────────────────────────────────────────────────────────
FRAME_WIDTH  = 320
FRAME_HEIGHT = 240

# ── Flask ─────────────────────────────────────────────────────────────────────
FLASK_HOST = "0.0.0.0"   
FLASK_PORT = 5000
JPEG_QUALITY = 50         # 1-100, wyżej = lepsza jakość ale wolniejszy stream

# ── MediaPipe ─────────────────────────────────────────────────────────────────
MP_DETECTION_CONFIDENCE = 0.5
MP_TRACKING_CONFIDENCE  = 0.5
MP_MODEL_COMPLEXITY     = 0    # 0 = najszybszy 

# ── Target lock (vision.py) ───────────────────────────────────────────────────
LOCK_MAX_JUMP       = 40   # maks. przeskok punktu między klatkami [px]
LOST_FRAMES_LIMIT   = 10   # ile klatek bez detekcji zanim reset locka

# ── Filtr Kalmana ─────────────────────────────────────────────────────────────
KALMAN_PROCESS_NOISE_XY      = 0.05
KALMAN_MEASUREMENT_NOISE_XY  = 5.0
KALMAN_PROCESS_NOISE_Z       = 0.1
KALMAN_MEASUREMENT_NOISE_Z   = 3.0

# ── Regulator PD — oś X (yaw) ─────────────────────────────────────────────────
REG_X_KP         = 0.5
REG_X_KD         = 0.3
REG_X_MAX_OUTPUT = 30

# ── Regulator PD — oś Y (góra/dół) ──────────────────────────────────
REG_Y_KP         = 0.5
REG_Y_KD         = 0.3
REG_Y_MAX_OUTPUT = 40

# ── Regulator PD — oś Z (dystans) ────────────────────────────────────
REG_Z_KP         = 1.5
REG_Z_KD         = 0.05
REG_Z_MAX_JUMP   = 15
REG_Z_MAX_OUTPUT = 50

# ── Deadzone (prostokąt na ekranie) ───────────────────────────────────────────
DEADZONE_W = 60   # połowa szerokości [px] — od środka kadru w lewo i prawo
DEADZONE_H = 40   # połowa wysokości [px]  — od środka kadru w górę i dół

# ── UART — komunikacja z F405 (iNAV) ──────────────────────────────────────────
UART_PORT     = "/dev/serial0"  
UART_BAUDRATE = 115200