"""
test_htors_hybrid_flask.py

Test pomiaru h_tors z wykorzystaniem HYBRYDOWEGO TRACKERA (MOSSE + MediaPipe)
oraz podglądem na żywo w przeglądarce (Flask).

Zapisuje dane do: test_htors_na_wysokosci.csv

Użycie:
    python3 test_htors_hybrid_flask.py
    Przeglądarka: http://<IP_MALINKI>:5000
"""

import time
import csv
import threading
import cv2
from flask import Flask, Response, render_template_string, jsonify
import config

# Importy z Twojego projektu
from pi_camera import PiVideoStream
from vision import PoseDetector
from tracker import HybridBodyTracker

# Konfiguracja rozdzielczości
RESOLUTION_LABEL = "320x240"
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
OUTPUT_CSV = "test_htors_na_wysokosci.csv"

config.FRAME_WIDTH = FRAME_WIDTH
config.FRAME_HEIGHT = FRAME_HEIGHT

# Inicjalizacja sprzętu i detektorów
vs = PiVideoStream().start()
detector = PoseDetector()

last_extra = {"h_tors": None, "pts": None}

def detect_fn(frame):
    result = detector.find_torso(frame)
    if result is None:
        return None
    
    last_extra["h_tors"] = result["h_tors"]
    last_extra["pts"] = result["pts"]

    return (result["cx"], result["cy"], result["bbox"])

# Instancja hybrydowego trackera (MOSSE + MediaPipe)
tracker = HybridBodyTracker(
    detect_fn,
    detect_every_n=config.TRACKER_DETECT_EVERY_N,
    gate_radius=config.TRACKER_GATE_RADIUS,
    confirm_frames=config.TRACKER_CONFIRM_FRAMES,
    max_misses=config.TRACKER_MAX_MISSES,
    q=config.TRACKER_KALMAN_Q,
    r=config.TRACKER_KALMAN_R,
)

time.sleep(2)  # Rozgrzewanie kamery

# Zmienne globalne stanu
recording = False
samples = []
sample_nr = 0
start_time = None
last_htors = None
is_locked = False
lock = threading.Lock()

app = Flask(__name__)

# Szablon HTML interfejsu
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="pl">
<head>
    <meta charset="UTF-8">
    <title>Hybrid MOSSE+MediaPipe h_tors Test</title>
    <style>
        body { font-family: Arial, sans-serif; background: #18181c; color: #fff; text-align: center; margin: 20px; }
        .container { max-width: 720px; margin: 0 auto; background: #24242c; padding: 20px; border-radius: 12px; }
        img { border: 2px solid #444; border-radius: 8px; width: 100%; max-width: 640px; height: auto; }
        .btn { padding: 12px 24px; font-size: 16px; font-weight: bold; border: none; border-radius: 6px; cursor: pointer; margin: 10px; }
        .btn-start { background-color: #28a745; color: white; }
        .btn-stop { background-color: #dc3545; color: white; }
        .btn:disabled { background-color: #555; cursor: not-allowed; }
        .stats { font-size: 17px; margin: 15px 0; background: #18181c; padding: 12px; border-radius: 6px; display: inline-block; width: 85%; }
        .badge-locked { color: #00ffcc; font-weight: bold; }
        .badge-search { color: #ff3366; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        
        <img src="/video_feed" alt="Strumień wideo">
        
        <div class="stats">
            
            Tracker: <span id="tracker-status">--</span><br>
            Liczba próbek w CSV: <b id="sample-count">0</b> | Ostatni h_tors: <b id="last-htors">--</b>
        </div>

        <div>
            <button id="btn-start" class="btn btn-start" onclick="startRecording()">▶ Rozpocznij Pomiar</button>
            <button id="btn-stop" class="btn btn-stop" onclick="stopRecording()" disabled>⏹ Zakończ i Zapisz</button>
        </div>

        <div id="summary" style="margin-top: 15px; text-align: left; background: #111116; padding: 12px; border-radius: 6px; display: none;">
            <b>Podsumowanie zebranych danych:</b>
            <pre id="summary-data" style="color: #00ffcc; font-size: 14px;"></pre>
        </div>
    </div>

    <script>
        function updateStatus() {
            fetch('/status')
                .then(r => r.json())
                .then(data => {
                    document.getElementById('sample-count').innerText = data.sample_count;
                    document.getElementById('last-htors').innerText = data.last_htors !== null ? data.last_htors : '--';
                    
                    let trkElem = document.getElementById('tracker-status');
                    if (data.is_locked) {
                        trkElem.innerText = "LOCKED";
                        trkElem.className = "badge-locked";
                    } else {
                        trkElem.innerText = "SEARCHING";
                        trkElem.className = "badge-search";
                    }

                    let stElem = document.getElementById('status-text');
                    if (data.recording) {
                        stElem.innerText = "NAGRYWANIE...";
                        stElem.style.color = "#28a745";
                    } else {
                        stElem.innerText = "Bezczynny";
                        stElem.style.color = "#ffc107";
                    }
                });
        }
        setInterval(updateStatus, 200);

        function startRecording() {
            document.getElementById('summary').style.display = 'none';
            fetch('/start', {method: 'POST'})
                .then(() => {
                    document.getElementById('btn-start').disabled = true;
                    document.getElementById('btn-stop').disabled = false;
                });
        }

        function stopRecording() {
            fetch('/stop', {method: 'POST'})
                .then(r => r.json())
                .then(data => {
                    document.getElementById('btn-start').disabled = false;
                    document.getElementById('btn-stop').disabled = true;
                    if (data.stats) {
                        let text = `Plik: ${data.stats.csv}\nZapisanych próbek (locked): ${data.stats.count}\nŚrednia h_tors: ${data.stats.avg} px\nOdchylenie Std: ${data.stats.std} px\nZakres (min - max): ${data.stats.min} - ${data.stats.max} px`;
                        document.getElementById('summary-data').innerText = text;
                        document.getElementById('summary').style.display = 'block';
                    }
                });
        }
    </script>
</body>
</html>
"""

def gen_frames():
    """Wątek generowania klatek z działającym trackerem hybrydowym."""
    global recording, samples, sample_nr, start_time, last_htors, is_locked

    while True:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue

        img = frame.copy()
        h, w, _ = img.shape
        c_x, c_y = w // 2, h // 2

        # Rysowanie celownika środkowego
        cv2.line(img, (c_x - 8, c_y), (c_x + 8, c_y), (255, 255, 255), 1)
        cv2.line(img, (c_x, c_y - 8), (c_x, c_y + 8), (255, 255, 255), 1)

        # Aktualizacja trackera hybrydowego (MOSSE + MediaPipe)
        try:
            cx, cy, locked, h_est = tracker.update(frame)
        except Exception as e:
            print(f"[TRACKER ERROR] {e}")
            cx, cy, locked, h_est = None, None, False, None

        h_tors = last_extra["h_tors"]
        pts = last_extra["pts"]

        with lock:
            is_locked = locked
            last_htors = h_tors if h_tors is not None else h_est

            # Rejestracja próbek (zapisujemy gdy trwa pomiar i tracker ma blokadę LOCKED)
            if recording and locked and last_htors is not None:
                sample_nr += 1
                elapsed = time.time() - start_time
                samples.append((sample_nr, round(elapsed, 3), round(last_htors, 2)))

        # Wizualizacja na obrazie
        if locked:
            # Rysowanie linii tułowia (jeśli MediaPipe zaktualizował punkty)
            if pts:
                cv2.line(img, pts[11], pts[12], (0, 255, 0), 2)
                cv2.line(img, pts[11], pts[23], (0, 255, 0), 2)
                cv2.line(img, pts[12], pts[24], (0, 255, 0), 2)
                cv2.line(img, pts[23], pts[24], (0, 255, 0), 2)

            # Rysowanie estymowanego środka (z MOSSE/Kalmana)
            if cx is not None and cy is not None:
                cv2.circle(img, (int(cx), int(cy)), 5, (0, 0, 255), -1)

            htors_val = last_htors if last_htors else 0
            cv2.putText(img, f"LOCKED | h_tors: {htors_val:.1f}px", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            cv2.putText(img, "SEARCHING...", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Wskaźnik nagrywania
        with lock:
            rec_on = recording
            cnt = sample_nr

        if rec_on:
            cv2.putText(img, f"REC #{cnt}", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Kodowanie JPEG
        ret, buffer = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        
        time.sleep(0.03)


@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/start', methods=['POST'])
def start_recording():
    global recording, samples, sample_nr, start_time
    with lock:
        tracker.reset()  # Reset stanu trackera przed nową serią
        recording = True
        samples = []
        sample_nr = 0
        start_time = time.time()
    return jsonify({"status": "started"})

@app.route('/stop', methods=['POST'])
def stop_recording():
    global recording, samples
    with lock:
        recording = False
        saved_samples = list(samples)

    if saved_samples:
        with open(OUTPUT_CSV, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["nr_probki", "czas_s", "h_tors"])
            for row in saved_samples:
                writer.writerow(row)

        vals = [s[2] for s in saved_samples]
        count = len(vals)
        avg = sum(vals) / count
        variance = sum((x - avg) ** 2 for x in vals) / count
        std = variance ** 0.5

        stats = {
            "csv": OUTPUT_CSV,
            "count": count,
            "avg": round(avg, 2),
            "std": round(std, 2),
            "min": round(min(vals), 2),
            "max": round(max(vals), 2)
        }
        return jsonify({"status": "stopped", "stats": stats})

    return jsonify({"status": "stopped", "stats": None})

@app.route('/status')
def status():
    with lock:
        return jsonify({
            "recording": recording,
            "is_locked": is_locked,
            "sample_count": len(samples),
            "last_htors": round(last_htors, 2) if last_htors is not None else None
        })

if __name__ == "__main__":
    try:
        print("\n=== SERWER FLASK z HYBRYDOWYM TRACKEREM URUCHOMIONY ===")
        print("Adres w przeglądarce: http://<IP_MALINKI>:5000\n")
        app.run(host=config.FLASK_HOST, port=5000, threaded=True)
    finally:
        vs.stop()