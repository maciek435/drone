"""
test_throttle_sweep.py

Test 6: koszt CPU vs jakosc sledzenia, w zaleznosci od detect_every_n.
Dziala NIEZALEZNIE od main.py - tylko kamera + detektor + tracker,
bez MSPController/ToF/serwa (niepotrzebne do tego testu).

Automatycznie przechodzi przez wszystkie kombinacje:
    detect_every_n = [1, 3, 5, 10] x 3 powtorzenia = 12 prob po 60s

Dla kazdej proby: stan przed startem daje Ci czas na ustawienie sie,
potem 60s w ktorym po prostu CHODZISZ przed kamera (ten sam, powtarzalny
wzorzec ruchu za kazdym razem - np. "krok w prawo, krok w lewo" w petli).

Na koniec: automatyczne podsumowanie - tabela wszystkich 12 prob +
usrednione wyniki per wartosc detect_every_n.

Uzycie:
    python3 test_throttle_sweep.py
"""

import time
import os
from pi_camera import PiVideoStream
from vision import PoseDetector
from tracker import HybridBodyTracker
import config

TRIAL_DURATION_S = 30
REPETITIONS = 3
DETECT_EVERY_N_VALUES = [1, 3, 5, 10]
PAUSE_BETWEEN_TRIALS_S = 5


def make_detect_fn(detector):
    """Zwraca (detect_fn, durations_list, last_extra_dict)."""
    durations = []
    last_extra = {"h_tors": None, "pts": None}

    def detect_fn(frame):
        start = time.time()
        result = detector.find_torso(frame)
        durations.append((time.time() - start) * 1000)

        if result is None:
            return None
        last_extra["h_tors"] = result["h_tors"]
        last_extra["pts"] = result["pts"]
        return (result["cx"], result["cy"], result["bbox"])

    return detect_fn, durations, last_extra


def run_trial(vs, detector, detect_every_n, trial_duration_s):
    detect_fn, durations, last_extra = make_detect_fn(detector)

    tracker = HybridBodyTracker(
        detect_fn,
        detect_every_n=detect_every_n,
        gate_radius=config.TRACKER_GATE_RADIUS,
        confirm_frames=config.TRACKER_CONFIRM_FRAMES,
        max_misses=config.TRACKER_MAX_MISSES,
        q=config.TRACKER_KALMAN_Q,
        r=config.TRACKER_KALMAN_R,
    )

    lost_transitions = 0
    prev_locked = True
    loads = []
    total_frames = 0
    locked_frames = 0

    start_time = time.time()
    last_load_sample = 0

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

        now = time.time()
        if now - last_load_sample > 1.0:
            loads.append(os.getloadavg()[0])
            last_load_sample = now

    avg_load = sum(loads) / len(loads) if loads else None
    avg_duration = sum(durations) / len(durations) if durations else None
    if durations:
        variance = sum((d - avg_duration) ** 2 for d in durations) / len(durations)
        std_duration = variance ** 0.5
    else:
        std_duration = None

    pct_locked = (100 * locked_frames / total_frames) if total_frames else 0

    return {
        "detect_every_n": detect_every_n,
        "lost_transitions": lost_transitions,
        "avg_load": avg_load,
        "n_detections": len(durations),
        "avg_duration_ms": avg_duration,
        "std_duration_ms": std_duration,
        "pct_locked": pct_locked,
        "total_frames": total_frames,
    }


def main():
    print("=== Test 6: throttling detekcji - koszt CPU vs jakosc sledzenia ===\n")
    print("Uruchamiam kamere...")
    vs = PiVideoStream().start()
    detector = PoseDetector()
    time.sleep(2)

    all_results = []

    for detect_every_n in DETECT_EVERY_N_VALUES:
        for rep in range(1, REPETITIONS + 1):
            input(f"\n>>> detect_every_n={detect_every_n}, powtorzenie {rep}/{REPETITIONS}. "
                  f"Stan przed kamera, nacisnij Enter aby rozpoczac {TRIAL_DURATION_S}s proby...")

            print(f"Start! Chodz przed kamera przez {TRIAL_DURATION_S}s "
                  "(ten sam wzorzec ruchu co zawsze)...")

            result = run_trial(vs, detector, detect_every_n, TRIAL_DURATION_S)
            all_results.append(result)

            print(f"  Koniec proby. Utrat celu: {result['lost_transitions']}, "
                  f"% LOCKED: {result['pct_locked']:.1f}%, "
                  f"sredni czas detekcji: {result['avg_duration_ms']:.1f}ms, "
                  f"obciazenie: {result['avg_load']:.2f}")

            if not (detect_every_n == DETECT_EVERY_N_VALUES[-1] and rep == REPETITIONS):
                print(f"Przerwa {PAUSE_BETWEEN_TRIALS_S}s przed kolejna proba...")
                time.sleep(PAUSE_BETWEEN_TRIALS_S)

    vs.stop()

    print("\n\n=== PODSUMOWANIE WSZYSTKICH 12 PROB ===\n")
    print(f"{'detect_n':>9} | {'rep':>3} | {'utraty':>7} | {'%LOCKED':>8} | "
          f"{'sr.czas_det[ms]':>15} | {'obciazenie':>10}")
    print("-" * 70)
    for r in all_results:
        print(f"{r['detect_every_n']:>9} | {'':>3} | {r['lost_transitions']:>7} | "
              f"{r['pct_locked']:>7.1f}% | {r['avg_duration_ms']:>15.1f} | "
              f"{r['avg_load']:>10.2f}")

    print("\n=== SREDNIE PER detect_every_n (3 powtorzenia) ===\n")
    print(f"{'detect_n':>9} | {'sr.utraty':>10} | {'sr.%LOCKED':>11} | "
          f"{'sr.czas_det[ms]':>15} | {'sr.obciazenie':>13}")
    print("-" * 70)
    for n in DETECT_EVERY_N_VALUES:
        group = [r for r in all_results if r["detect_every_n"] == n]
        avg_lost = sum(r["lost_transitions"] for r in group) / len(group)
        avg_pct = sum(r["pct_locked"] for r in group) / len(group)
        avg_dur = sum(r["avg_duration_ms"] for r in group) / len(group)
        avg_load = sum(r["avg_load"] for r in group) / len(group)
        print(f"{n:>9} | {avg_lost:>10.2f} | {avg_pct:>10.1f}% | "
              f"{avg_dur:>15.1f} | {avg_load:>13.2f}")


if __name__ == "__main__":
    main()