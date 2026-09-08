"""
test_speed_orientation_static.py

Test 10 (wersja STATYCZNA): wplyw predkosci oddalania sie i orientacji ciala
na utrate celu. Dziala NIEZALEZNIE od main.py - tylko kamera + tracker,
BEZ potrzeby lotu drona (dron moze stac/lezec, kamera musi tylko widziec).

6 kombinacji (3 predkosci x 2 orientacje) x 3 powtorzenia = 18 prob,
kazda po 15s (oddalanie sie).

Uzycie:
    python3 test_speed_orientation_static.py
"""

import time
from pi_camera import PiVideoStream
from vision import PoseDetector
from tracker import HybridBodyTracker
import config

TRIAL_DURATION_S = 15
REPETITIONS = 3
PAUSE_BETWEEN_TRIALS_S = 3

COMBINATIONS = [
    ("wolny marsz", "przodem"),
    ("wolny marsz", "plecami"),
    ("normalny marsz", "przodem"),
    ("normalny marsz", "plecami"),
    ("szybki marsz/trucht", "przodem"),
    ("szybki marsz/trucht", "plecami"),
]


def make_detect_fn(detector):
    last_extra = {"h_tors": None, "pts": None}

    def detect_fn(frame):
        result = detector.find_torso(frame)
        if result is None:
            return None
        last_extra["h_tors"] = result["h_tors"]
        last_extra["pts"] = result["pts"]
        return (result["cx"], result["cy"], result["bbox"])

    return detect_fn, last_extra


def run_trial(vs, detector, trial_duration_s):
    detect_fn, last_extra = make_detect_fn(detector)

    tracker = HybridBodyTracker(
        detect_fn,
        detect_every_n=config.TRACKER_DETECT_EVERY_N,
        gate_radius=config.TRACKER_GATE_RADIUS,
        confirm_frames=config.TRACKER_CONFIRM_FRAMES,
        max_misses=config.TRACKER_MAX_MISSES,
        q=config.TRACKER_KALMAN_Q,
        r=config.TRACKER_KALMAN_R,
    )

    lost_transitions = 0
    prev_locked = True
    total_frames = 0
    locked_frames = 0

    start_time = time.time()
    while time.time() - start_time < trial_duration_s:
        frame = vs.read()
        if frame is None:
            time.sleep(0.01)
            continue

        cx, cy, locked, h_est = tracker.update(frame)
        total_frames += 1
        if locked:
            locked_frames += 1
        if (not locked) and prev_locked:
            lost_transitions += 1
        prev_locked = locked

    pct_locked = (100 * locked_frames / total_frames) if total_frames else 0
    return {
        "lost_transitions": lost_transitions,
        "pct_locked": pct_locked,
        "total_frames": total_frames,
    }


def main():
    print("=== Test 10 (STATYCZNY): predkosc/orientacja vs utrata celu ===\n")
    print("Uruchamiam kamere...")
    vs = PiVideoStream().start()
    detector = PoseDetector()
    time.sleep(2)

    all_results = []

    for speed, orientation in COMBINATIONS:
        for rep in range(1, REPETITIONS + 1):
            input(f"\n>>> Predkosc: {speed} | Orientacja: {orientation} | "
                  f"powtorzenie {rep}/{REPETITIONS}\n"
                  f"    Stan BLISKO kamery (zwrocony {orientation}). "
                  f"Nacisnij Enter, aby rozpoczac {TRIAL_DURATION_S}s proby...")

            print(f"Start! Oddalaj sie ({speed}, {orientation}) przez {TRIAL_DURATION_S}s...")
            result = run_trial(vs, detector, TRIAL_DURATION_S)
            result["speed"] = speed
            result["orientation"] = orientation
            all_results.append(result)

            print(f"  Wynik: utraty celu={result['lost_transitions']}, "
                  f"% LOCKED={result['pct_locked']:.1f}%")