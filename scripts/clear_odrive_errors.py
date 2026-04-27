import odrive


def clear_odrive_errors(serial_number: str) -> None:
    print(f"Finding ODrive with serial number {serial_number}...")
    odrv = odrive.find_any(serial_number=serial_number)

    print("Clearing errors...")
    odrv.clear_errors()

    print(f"Errors cleared from {serial_number}!")


def main():
    clear_odrive_errors("395534753331")  # Left
    clear_odrive_errors("384934743539")  # Right


if __name__ == "__main__":
    main()
