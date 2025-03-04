import numpy as np
import time
import os
from gnuradio import gr

class blk(gr.sync_block):  
    """Embedded Python Block to toggle an integer variable on input value 1.0 with cooldown and write to a shared file"""

    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='Toggle Integer to Shared File Block',  # Block name
            in_sig=[np.float32],  # Accept float input signal
            out_sig=None          # No output signal
        )
        self.toggled_value = 1  # Initialize variable to 1
        self.last_toggle_time = 0  # Track the last toggle time
        self.cooldown = 2  # Cooldown period in seconds
        self.file_path = "/tmp/estop_value.txt"  # Shared file location

        # Write the initial value to the file
        try:
            with open(self.file_path, 'w') as f:
                f.write(f"{self.toggled_value}\n")
            print(f"Initial value written to file: {self.toggled_value}")
        except IOError as e:
            print(f"Error writing initial value to file: {e}")

    def work(self, input_items, output_items):
        current_time = time.time()  # Get the current time

        for value in input_items[0]:  # Iterate through input float values
            if value == 1.0 and (current_time - self.last_toggle_time) >= self.cooldown:
                # Toggle the integer value between 0 and 1
                self.toggled_value = 1 if self.toggled_value == 0 else 0
                self.last_toggle_time = current_time  # Update the last toggle time

                # Write the toggled integer value to the shared file
                try:
                    with open(self.file_path, 'w') as f:
                        f.write(f"{self.toggled_value}\n")
                    print(f"Toggled value written to file: {self.toggled_value}")
                except IOError as e:
                    print(f"Error writing to file: {e}")

        return len(input_items[0])

