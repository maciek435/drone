"""
analyze_speed_test.py

Analiza logu z Testu 10 (wplyw predkosci oddalania sie na utrate celu).
Wypisuje wszystkie zarejestrowane wpisy 'tracker_state' w kolejnosci czasowej,
z WYRAZNYM oznaczeniem momentow przejscia LOCKED -> LOST.

Uzycie (PO zakonczeniu testu, po Ctrl+C w main.py):
    python3 analyze_speed_test.py
"""

LOG_PATH = "/home/pi4/drone/test_crash_resilience.log"


def parse_tracker_line(data_str):
    result = {}
    for part in data_str.split(","):
        key, val = part.split("=")
        if val == "None":
            result[key] = None
        elif val in ("True", "False"):
            result[key] = (val == "True")
        else:
            try:
                result[key] = float(val)
            except ValueError:
                result[key] = val
    return result


def main():
    with open(LOG_PATH) as f:
        lines = [l.strip().split(",", 2) for l in f if l.strip()]

    events = [l for l in lines if l[1] == "tracker_state"]
    print(f"Zarejestrowano {len(events)} wpisow tracker_state\n")

    if not events:
        print("Brak danych - sprawdz czy log_test_event zostalo dodane do tracking_worker.")
        return

    print(f"{'czas':>10} | {'locked':>7} | {'misses':>7} | {'h_tors':>8} | {'h_est':>8} | uwaga")
    print("-" * 70)

    prev_locked = True
    lost_transitions = 0
    locked_count = 0
    total_count = 0

    # do wykrywania duzych przerw miedzy probami (>3s = nowa proba)
    prev_ts = None

    for ts, _, data_str in events:
        ts_f = float(ts)
        data = parse_tracker_line(data_str)
        locked = data.get("locked")
        misses = data.get("misses")
        h_tors = data.get("h_tors")
        h_est = data.get("h_est")

        note = ""
        if prev_ts is not None and (ts_f - prev_ts) > 3.0:
            note += "  [NOWA PROBA - przerwa >3s]"

        if locked is False and prev_locked is True:
            note += "  <<< PRZEJSCIE LOCKED -> LOST"
            lost_transitions += 1

        if locked:
            locked_count += 1
        total_count += 1

        h_tors_str = f"{h_tors:.1f}" if h_tors is not None else "None"
        h_est_str = f"{h_est:.1f}" if h_est is not None else "None"

        print(f"{ts_f:>10.2f} | {str(locked):>7} | {str(misses):>7} | "
              f"{h_tors_str:>8} | {h_est_str:>8} |{note}")

        prev_locked = locked
        prev_ts = ts_f

    print(f"\n=== Podsumowanie ===")
    print(f"Calkowity czas w stanie LOCKED: {locked_count}/{total_count} "
          f"({100*locked_count/total_count:.1f}%)")
    print(f"Liczba przejsc LOCKED -> LOST: {lost_transitions}")
    print("\nDopasuj powyzsze 'NOWA PROBA' i 'PRZEJSCIE LOCKED->LOST' do swoich")
    print("recznych notatek (numer kombinacji/proby), zeby zbudowac tabele wynikow.")


if __name__ == "__main__":
    main()