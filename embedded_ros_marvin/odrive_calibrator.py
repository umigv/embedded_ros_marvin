import time

import odrive
from odrive.enums import AXIS_STATE_FULL_CALIBRATION_SEQUENCE, AXIS_STATE_IDLE

def calibrate_odrive(serial_number: str) -> None:
    print(f"Finding ODrive with serial number {serial_number}...")
    odrv = odrive.find_any(serial_number=serial_number)

    print(f"Calibrating {serial_number}...")
    odrv.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

    while odrv.axis0.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
        pass

    print(f"Calibration of {serial_number} complete!")

if __name__ == "__main__":
    # Odrive Left
    calibrate_odrive("395934763331")
    # Odrive Right
    calibrate_odrive("384934743539")