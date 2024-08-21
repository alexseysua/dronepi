import time
import json
import os

# Path to the bind mount directory where data will be read
file_path = "/mnt/shared/localization_data.json"

def read_data_from_file(file_path):
    last_data = None
    while True:
        start_time = time.time()
        
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                data = json.load(file)
                
                # Only print and measure time if the data has changed
                if data != last_data:
                    elapsed_time = time.time() - start_time
                    print(f"Data read from {file_path}: {data}")
                    print(f"Time taken to detect change: {elapsed_time:.6f} seconds")
                    last_data = data
        else:
            print(f"File not found: {file_path}")

        time.sleep(0.1)  # Adjust this interval as needed

if __name__ == "__main__":
    read_data_from_file(file_path)
