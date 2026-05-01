from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EstopDriverConfig:
    """Config for EstopDriver.

    Attributes:
        serial_port: Path to the serial device.
        baud_rate: Serial baud rate for communication.
        poll_period_s: Period (s) at which the serial port is polled for new data.
        estop_file_path: Path to the file the latest e-stop state is written to ("1" = stopped).
    """

    serial_port: Path = Path("/dev/estop")
    baud_rate: int = 9600
    poll_period_s: float = 0.05
    estop_file_path: Path = Path("/tmp/estop_value.txt")

    def __post_init__(self) -> None:
        if self.baud_rate <= 0:
            raise ValueError("EstopDriverConfig: baud_rate must be > 0")
        if self.poll_period_s <= 0:
            raise ValueError("EstopDriverConfig: poll_period_s must be > 0")
