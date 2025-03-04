import numpy as np
import time
import os
import sys
from gnuradio import gr

class blk(gr.sync_block):  
    """Real-time Toggle Integer Block with Immediate File Updates"""
    
    def __init__(self):
        gr.sync_block.__init__(
            self,
            name='Real-Time Toggle Integer Block',
            in_sig=[np.float32],  
            out_sig=None          
        )
        
        self.toggled_value = 1  
        self.last_toggle_time = 0  
        self.cooldown = 2  # Cooldown in seconds
        self.file_path = "/tmp/estop_value.txt"

        # Create or initialize the file
        with open(self.file_path, "w") as f:
            f.write("1\n")

    def work(self, input_items, output_items):
        """Real-time processing with forced file updates"""
        current_time = time.time()
        
        if input_items[0][0] == 1.0 and (current_time - self.last_toggle_time) >= self.cooldown:
            self.toggled_value = 1 if self.toggled_value == 0 else 0
            self.last_toggle_time = current_time  

            # **Force an immediate file update**
            try:
                with open(self.file_path, 'w') as f:
                    f.write(f"{self.toggled_value}\n")
                    f.flush()  # Ensure data is written
                    os.fsync(f.fileno())  # Force write to disk

                print(f"[TOGGLED] New Value: {self.toggled_value}")
                sys.stdout.flush()  # Ensure immediate output

            except IOError as e:
                print(f"Error writing to file: {e}")

        return len(input_items[0])

