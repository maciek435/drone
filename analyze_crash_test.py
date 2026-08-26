"""analyze_crash_test.py"""
LOG_PATH = "/home/pi4/drone/test_crash_resilience.log"

def main():
    with open(LOG_PATH) as f:
        lines = [l.strip().split(",", 2) for l in f if l.strip()]
    if not lines:
        print("Log pusty.")
        return

    timestamps = [float(l[0]) for l in lines]
    events = lines

    print(f"Liczba zdarzen: {len(lines)}")
    print(f"Czas trwania sesji: {timestamps[-1] - timestamps[0]:.2f}s\n")

    print("Przerwy dluzsze niz 1s (potencjalne 'zamrozenia'):")
    found = False
    for i in range(1, len(timestamps)):
        gap = timestamps[i] - timestamps[i-1]
        if gap > 1.0:
            print(f"  {gap:.2f}s przerwy po wpisie {i-1} ({events[i-1][1]})")
            found = True
    if not found:
        print("  brak")

    print(f"\nOstatni zarejestrowany wpis: t={timestamps[-1]:.3f} ({events[-1][1]})")
    print("Jesli test trwal dluzej niz to (sprawdz zegarek/stoper) - watek UMARL bez powrotu.")

if __name__ == "__main__":
    main()