import os
import serial

SERIAL_PORT = "/dev/ttyS3"
BAUD_RATE = 9600
TARGET_FILE = "/tmp/estop_value.txt"
TEMP_FILE = "/tmp/estop_value.tmp"


def run_estop_monitor():
    current_state = None

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
            print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud for '0' and '1'...")

            while True:
                if ser.in_waiting <= 0:
                    continue

                line = ser.readline().decode("utf-8", errors="ignore").strip()
                if line not in ("0", "1"):
                    continue
                if line == current_state:
                    continue

                # Atomic replace so readers never see a half-written file.
                with open(TEMP_FILE, "w") as f:
                    f.write(line)
                os.replace(TEMP_FILE, TARGET_FILE)

                current_state = line
                state_str = "STOP" if line == "1" else "SAFE"
                print(f"State changed: {line} -> {state_str}")

    except serial.SerialException as e:
        print(f"Error opening or reading serial port: {e}")
    except KeyboardInterrupt:
        print("\nStopping monitor.")


if __name__ == "__main__":
    run_estop_monitor()
