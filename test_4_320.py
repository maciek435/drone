"""
test_htors_calibration_320x240.py

Test 4: dokladnosc i granice proxy dystansu h_tors, rozdzielczosc 320x240.
Dziala NIEZALEZNIE od main.py - tylko kamera + MediaPipe.

Dla kazdej z 10 znanych odleglosci (1.0-10.0m), 3 powtorzenia:
- Ustaw osobe DOKLADNIE w tej odleglosci (zmierzonej tasma)
- Program zbiera ~30 probek h_tors, liczy srednia + odchylenie std

Zapisuje dane do htors_calibration_320x240.csv (z odchyleniem std -
to ono pokazuje GDZIE zaczyna sie degradacja pomiaru).

Uzycie:
    python3 test_htors_calibration_320x240.py
"""

import time
import csv
import config

RESOLUTION_LABEL = "320x240"
FRAME_WIDTH = 320
FRAME_HEIGHT = 240

SAMPLES_PER_POSITION = 30
SAMPLE_INTERVAL_S = 0.1

DISTANCES_M = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]
REPETITIONS = 3


def collect_samples(detector, vs, n_samples, interval_s):
    samples = []
    attempts = 0
    max_attempts = n_samples * 8

    while len(samples) < n_samples and attempts < max_attempts:
        frame = vs.read()
        if frame is not None:
            result = detector.find_torso(frame)
            if result is not None:
                samples.append(result["h_tors"])
        attempts += 1
        time.sleep(interval_s)

    return samples


def linear_fit(xs, ys):
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n

    numerator = sum((xs[i] - mean_x) * (ys[i] - mean_y) for i in range(n))
    denominator = sum((xs[i] - mean_x) ** 2 for i in range(n))

    if denominator == 0:
        return None, None, None

    slope = numerator / denominator
    intercept = mean_y - slope * mean_x

    predictions = [slope * x + intercept for x in xs]
    squared_errors = [(ys[i] - predictions[i]) ** 2 for i in range(n)]
    rmse = (sum(squared_errors) / n) ** 0.5

    return slope, intercept, rmse


def main():
    print(f"=== Test 4: kalibracja h_tors - rozdzielczosc {RESOLUTION_LABEL} ===\n")

    config.FRAME_WIDTH = FRAME_WIDTH
    config.FRAME_HEIGHT = FRAME_HEIGHT

    from pi_camera import PiVideoStream
    from vision import PoseDetector

    print(f"Uruchamiam kamere w rozdzielczosci {FRAME_WIDTH}x{FRAME_HEIGHT}...")
    vs = PiVideoStream().start()
    detector = PoseDetector()
    time.sleep(2)

    raw_csv = f"htors_raw_samples_{RESOLUTION_LABEL}.csv"
    summary_csv = f"htors_summary_{RESOLUTION_LABEL}.csv"

    raw_rows = []     # (dystans_cm, powtorzenie, nr_probki, h_tors) - KAZDA pojedyncza probka
    all_data = []      # (dystans_cm, srednia, odch_std, n) - do podsumowania w terminalu

    for dist_m in DISTANCES_M:
        dist_cm = dist_m * 100
        for rep in range(1, REPETITIONS + 1):
            input(f"\n>>> Ustaw osobe DOKLADNIE na {dist_m}m ({dist_cm:.0f}cm), "
                  f"powtorzenie {rep}/{REPETITIONS}. Nacisnij Enter, gdy gotowe...")

            print(f"Zbieram {SAMPLES_PER_POSITION} probek...")
            samples = collect_samples(detector, vs, SAMPLES_PER_POSITION, SAMPLE_INTERVAL_S)

            if not samples:
                print("  BRAK udanych probek! Sprobuj ponownie tej samej pozycji.")
                continue

            # zapisz KAZDA pojedyncza probke osobno - to sa prawdziwe, surowe dane
            for i, s in enumerate(samples):
                raw_rows.append((dist_cm, rep, i + 1, s))

            avg_h_tors = sum(samples) / len(samples)
            variance = sum((s - avg_h_tors) ** 2 for s in samples) / len(samples)
            std_h_tors = variance ** 0.5

            print(f"  Zebrano {len(samples)} probek, srednia h_tors = {avg_h_tors:.2f}, "
                  f"odch.std = {std_h_tors:.2f}")

            all_data.append((dist_cm, avg_h_tors, std_h_tors, len(samples)))

    vs.stop()

    # plik 1: PRAWDZIWE surowe dane - kazda pojedyncza probka osobno
    with open(raw_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["dystans_cm", "powtorzenie", "nr_probki", "h_tors"])
        for row in raw_rows:
            writer.writerow(row)
    print(f"\nZapisano WSZYSTKIE pojedyncze probki ({len(raw_rows)} wierszy) do {raw_csv}")

    # plik 2: podsumowanie (srednia/std per przystanek) - wygodne do szybkiego wgladu
    with open(summary_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["dystans_cm", "h_tors_srednie", "h_tors_odch_std", "liczba_probek"])
        for row in all_data:
            writer.writerow(row)
    print(f"Zapisano podsumowanie ({len(all_data)} wierszy) do {summary_csv}")

    if len(all_data) < 2:
        print("Za malo danych do dopasowania.")
        return

    xs = [1.0 / (d / 100.0) for d, h, s, n in all_data]
    ys = [h for d, h, s, n in all_data]
    slope, intercept, rmse = linear_fit(xs, ys)

    print(f"\n=== [{RESOLUTION_LABEL}] Wynik dopasowania: h_tors = k*(1/dystans_m) + b ===")
    if slope is not None:
        print(f"k (stala kalibracyjna)  = {slope:.2f}")
        print(f"b (wyraz wolny)         = {intercept:.3f}")
        print(f"RMSE dopasowania        = {rmse:.3f}")

    print(f"\n=== [{RESOLUTION_LABEL}] Tabela: gdzie h_tors dziala dobrze / degraduje ===")
    print(f"{'dystans_cm':>10} | {'h_tors_sr':>10} | {'odch_std':>9} | uwaga")
    print("-" * 55)

    valid_stds = [s for d, h, s, n in all_data if s is not None]
    min_std = min(valid_stds) if valid_stds else 0

    for d, h, s, n in all_data:
        note = ""
        if min_std > 0 and s > 3 * min_std:
            note = "  <<< ODCHYLENIE STD > 3x MINIMUM - mozliwa degradacja"
        print(f"{d:>10.0f} | {h:>10.2f} | {s:>9.2f} |{note}")


if __name__ == "__main__":
    main()