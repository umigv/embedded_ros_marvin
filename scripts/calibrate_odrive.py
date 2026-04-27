import time

import odrive
from odrive.enums import AxisState


def calibrate_odrive(serial_number: str) -> None:
    print(f"Finding ODrive with serial number {serial_number}...")
    odrv = odrive.find_any(serial_number=serial_number)

    print("Clearing pre-existing errors...")
    odrv.clear_errors()

    print("Starting calibration...")
    print("There should be a beep and the indicator lights should flash green. Do not touch the robot!")
    odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE

    # Wait for the ODrive to leave idle, confirming calibration has started
    while odrv.axis0.current_state == AxisState.IDLE:
        time.sleep(0.1)

    while odrv.axis0.current_state != AxisState.IDLE:
        time.sleep(0.1)

    print(f"ODrive {serial_number} calibrated!")


def main():
    calibrate_odrive("395534753331")  # Left
    calibrate_odrive("384934743539")  # Right
    print("Make sure the indicator lights are blue")


if __name__ == "__main__":
    main()
