import time
import random
import json

# Path to the bind mount directory where data will be written
file_path = "/home/pi/shared_w_docker/localization_data.json"

def generate_mock_localization_data():
    # Generate some random localization data
    localization_data = {
        "x": round(random.uniform(-10.0, 10.0), 2),
        "y": round(random.uniform(-10.0, 10.0), 2),
        "z": round(random.uniform(0.0, 5.0), 2)
    }
    return localization_data

def write_data_to_file(file_path):
    while True:
        # Generate mock data
        data = generate_mock_localization_data()
        
        # Write data to file in JSON format
        with open(file_path, 'w') as file:
            json.dump(data, file)
        
        print(f"Data written to {file_path}: {data}")
        
        # Wait for a short period before updating again
        time.sleep(1)  # Adjust this interval as needed

if __name__ == "__main__":
    write_data_to_file(file_path)
