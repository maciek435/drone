"""
analyze_safety_test.py

Analiza logu z testu progu bezpieczenstwa SafetyGuard (Test 3).
Wypisuje wszystkie zarejestrowane pomiary "safety_check" w kolejnosci
czasowej, zaznaczajac WYRAZNIE moment, w ktorym clamped != raw
(czyli moment aktywacji zabezpieczenia).

Uzycie (PO zakonczeniu testu, po Ctrl+C w main.py):
    python3 analyze_safety_test.py
"""

LOG_PATH = "/home/pi4/drone/test_crash_resilience.log"


def parse_safety_line(data_str):
    """Parsuje 'h_tors=98,raw=-95,clamped=-95' na slownik liczb."""
    result = {}
    for part in data_str.split(","):
        key, val = part.split("=")
        try:
            result[key] = float(val) if val not in ("None",) else None
        except ValueError:
            result[key] = None
    return result


def main():
    with open(LOG_PATH) as f:
        lines = [l.strip().split(",", 2) for l in f if l.strip()]

    safety_events = [l for l in lines if l[1] == "safety_check"]
    print(f"Zarejestrowano {len(safety_events)} pomiarow safety_check\n")

    if not safety_events:
        print("Brak danych - sprawdz czy log_test_event zostalo poprawnie dodane do flight_worker.")
        return

    print(f"{'czas':>10} | {'h_tors':>8} | {'raw':>8} | {'clamped':>8} | uwaga")
    print("-" * 60)

    prev_was_clamped = False
    activation_count = 0

    for ts, _, data_str in safety_events:
        data = parse_safety_line(data_str)
        h_tors = data.get("h_tors")
        raw = data.get("raw")
        clamped = data.get("clamped")

        is_clamped_now = (raw is not None and clamped is not None and raw != clamped)

        note = ""
        if is_clamped_now and not prev_was_clamped:
            note = "  <<< AKTYWACJA ZABEZPIECZENIA TUTAJ"
            activation_count += 1

        h_tors_str = f"{h_tors:.1f}" if h_tors is not None else "None"
        raw_str = f"{raw:.1f}" if raw is not None else "None"
        clamped_str = f"{clamped:.1f}" if clamped is not None else "None"

        print(f"{float(ts):>10.2f} | {h_tors_str:>8} | {raw_str:>8} | {clamped_str:>8} |{note}")

        prev_was_clamped = is_clamped_now

    print(f"\nLiczba wykrytych aktywacji zabezpieczenia (przejsc z 'przepuszcza' na 'blokuje'): {activation_count}")
    print("\nDLA KAZDEJ aktywacji powyzej - zanotuj rzeczywisty dystans zmierzony tasma")
    print("w tym momencie testu (na podstawie Twoich recznych notatek/obserwacji na zywo).")


if __name__ == "__main__":
    main()