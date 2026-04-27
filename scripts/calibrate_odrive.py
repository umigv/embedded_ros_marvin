import time

import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE


def calibrate_odrive(serial_number: str) -> None:
    print(f"Finding ODrive with serial number {serial_number}...")
    odrv = odrive.find_any(serial_number=serial_number)

    print("Clearing pre-existing errors...")
    odrv.clear_errors()

    print("Starting calibration...")
    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    while odrv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    print(f"ODrive {serial_number} calibrated!")


def main():
    calibrate_odrive("395534753331")  # Left
    calibrate_odrive("384934743539")  # Right


if __name__ == "__main__":
    main()
