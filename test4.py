"""
test_htors_calibration.py

Test 4: dokladnosc proxy dystansu h_tors + kalibracja stalej k.
Dziala NIEZALEZNIE od main.py - wlasny odczyt kamery + MediaPipe,
nie wymaga uruchamiania calego systemu sledzenia/lotu.

Dla kazdej z 5 znanych odleglosci (1.0/1.5/2.0/2.5/3.0m), 3 powtorzenia:
- Ustaw osobe DOKLADNIE w tej odleglosci (zmierzonej tasma)
- Program prosi o potwierdzenie Enterem, potem zbiera ~30 probek h_tors
  przez kilka sekund i usrednia

Na koniec:
- zapisuje wszystkie dane do CSV
- dopasowuje prosta h_tors = k*(1/dystans) + b (metoda najmniejszych kwadratow)
- podaje k (stala kalibracyjna) i RMSE dopasowania

Uzycie:
    python3 test_htors_calibration.py
"""

import time
import csv
from pi_camera import PiVideoStream
from vision import PoseDetector

SAMPLES_PER_POSITION = 30
SAMPLE_INTERVAL_S = 0.1
OUTPUT_CSV = "htors_calibration_data.csv"

DISTANCES_M = [1.0, 1.5, 2.0, 2.5, 3.0]
REPETITIONS = 3


def collect_samples(detector, vs, n_samples, interval_s):
    """Zbiera n probek h_tors, pomijajac klatki bez udanej detekcji."""
    samples = []
    attempts = 0
    max_attempts = n_samples * 8  # zabezpieczenie przed nieskonczona petla

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
    """Zwraca (slope, intercept, rmse) metoda najmniejszych kwadratow."""
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
    print("=== Test 4: kalibracja h_tors vs rzeczywisty dystans ===\n")
    print("Uruchamiam kamere...")
    vs = PiVideoStream().start()
    detector = PoseDetector()
    time.sleep(2)

    all_data = []  # lista (dystans_cm, h_tors_srednie, liczba_probek)

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

            avg_h_tors = sum(samples) / len(samples)
            print(f"  Zebrano {len(samples)} probek, srednia h_tors = {avg_h_tors:.2f}")

            all_data.append((dist_cm, avg_h_tors, len(samples)))

    vs.stop()

    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["dystans_cm", "h_tors_srednie", "liczba_probek"])
        for row in all_data:
            writer.writerow(row)
    print(f"\nZapisano surowe dane do {OUTPUT_CSV}")

    if len(all_data) < 2:
        print("Za malo danych do dopasowania.")
        return

    xs = [1.0 / d for d, h, n in all_data]
    ys = [h for d, h, n in all_data]

    slope, intercept, rmse = linear_fit(xs, ys)

    if slope is None:
        print("Nie mozna dopasowac prostej (brak zroznicowania w danych).")
        return

    print("\n=== Wynik dopasowania: h_tors = k*(1/dystans) + b ===")
    print(f"k (stala kalibracyjna)  = {slope:.2f}")
    print(f"b (wyraz wolny)         = {intercept:.3f}")
    print(f"RMSE dopasowania        = {rmse:.3f}")
    print(f"\nWzor: dystans_cm = {slope:.2f} / (h_tors - ({intercept:.3f}))")
    print("(jesli |b| jest male wzgledem k, mozna uproscic do: dystans_cm = k / h_tors)")


if __name__ == "__main__":
    main()