"""
test_tof_long.py

Test 3x VL53L1X w trybie LONG (distance_mode=2) - maksymalny mozliwy zasieg
wg dokumentacji biblioteki Adafruit (do ~360cm w sprzyjajacych warunkach),
na potrzeby prostego, twardego hamowania przed przeszkoda (bez logiki
omijania/odpychania na boki).

WAZNE: sensor.distance zwraca wartosc JUZ W CENTYMETRACH (nie mm!) -
zgodnie z oficjalnym przykladem Adafruit. Brak tu mnozenia razy 10.

timing_budget=140ms - zgodnie z datasheet ST, ta wartosc pozwala osiagnac
pelny, deklarowany zasieg 4m w trybie LONG (kosztem wolniejszego odswiezania
- ok. 7 pomiarow/s na czujnik przy tym budzecie czasowym).

Uzycie:
    python3 test_tof_long.py
Zatrzymanie: Ctrl+C
"""

import time
import board
import digitalio
import adafruit_vl53l1x

# ZWERYFIKUJ - podmien na rzeczywiste piny GPIO (BCM)
XSHUT_PINS = [board.D17, board.D27, board.D22]   # [srodek, lewy, prawy]
ADDRESSES = [0x30, 0x32, 0x34]                     # co dwa, zgodnie z Twoja obserwacja
LABELS = ["SRODEK", "LEWY  ", "PRAWY "]

DISTANCE_MODE = 2        # 2 = LONG (maksymalny zasieg, do 360cm wg dokumentacji biblioteki)
TIMING_BUDGET_MS = 200   # WAZNE: dozwolone tylko: 15(tylko SHORT), 20, 33, 50, 100, 200, 500
                          # 200 = najwiekszy rozsadny wybor dla maksymalnego zasiegu


def main():
    print("=== Test 3x VL53L1X - tryb LONG (maksymalny zasieg) ===\n")

    i2c = board.I2C()
    xshut = [digitalio.DigitalInOut(pin) for pin in XSHUT_PINS]

    print("Usypiam wszystkie czujniki (XSHUT = LOW)...")
    for shut in xshut:
        shut.switch_to_output(value=False)
    time.sleep(0.2)

    sensors = []
    for i, shut in enumerate(xshut):
        print(f"Budze czujnik {i} ({LABELS[i].strip()}), adres {hex(ADDRESSES[i])}, "
              f"tryb LONG, timing_budget={TIMING_BUDGET_MS}ms...")
        shut.value = True
        time.sleep(0.2)

        try:
            sensor = adafruit_vl53l1x.VL53L1X(i2c)
            sensor.set_address(ADDRESSES[i])
            sensor.distance_mode = DISTANCE_MODE
            sensor.timing_budget = TIMING_BUDGET_MS
            sensor.start_ranging()
            sensors.append(sensor)
            print(f"  -> OK.\n")
        except Exception as e:
            print(f"  -> BLAD przy inicjalizacji czujnika {i}: {e}\n")
            sensors.append(None)

    if all(s is None for s in sensors):
        print("ZADEN czujnik nie zainicjalizowal sie poprawnie.")
        return

    print("=== Odczyt na zywo (Ctrl+C aby zakonczyc) ===")
    print("Testuj na roznych, ZNANYCH odleglosciach (np. 1m, 2m, 3m) i porownaj")
    print("z rzeczywista odlegloscia zmierzona np. tasma.\n")

    history = {i: [] for i in range(len(sensors))}

    try:
        while True:
            readings = []
            for i, sensor in enumerate(sensors):
                if sensor is None:
                    readings.append("BRAK")
                    continue
                try:
                    if sensor.data_ready:
                        dist_cm = sensor.distance  # JUZ w cm, bez konwersji
                        sensor.clear_interrupt()
                        if dist_cm is not None and 0 < dist_cm <= 400:
                            history[i].append(dist_cm)
                            readings.append(f"{dist_cm:6.1f} cm")
                        else:
                            readings.append("  ---   ")
                    else:
                        readings.append("(czekam)")
                except Exception as e:
                    readings.append(f"BLAD:{e}")

            line = "  |  ".join(f"{LABELS[i]}: {r}" for i, r in enumerate(readings))
            print(f"\r{line}", end="", flush=True)
            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n\nZatrzymano przez uzytkownika.")

    finally:
        print("Zatrzymuje czujniki...")
        for sensor in sensors:
            if sensor is not None:
                try:
                    sensor.stop_ranging()
                except Exception:
                    pass

        print("\n=== Statystyki sesji ===")
        for i, label in enumerate(LABELS):
            data = history[i]
            if data:
                avg = sum(data) / len(data)
                variance = sum((d - avg) ** 2 for d in data) / len(data)
                print(f"{label.strip()}: n={len(data)}  srednia={avg:.1f}cm  "
                      f"wariancja={variance:.2f}  odchylenie_std={variance**0.5:.2f}cm")
            else:
                print(f"{label.strip()}: brak udanych odczytow")

        print("\nKoniec testu.")


if __name__ == "__main__":
    main()