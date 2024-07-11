import re
from datetime import datetime

def analyze_log(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    grid_start_times = []
    grid_end_times = []
    stitcher_start_times = []
    stitcher_end_times = []

    for line in lines:
        timestamp = re.search(r'\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}', line).group()
        time_obj = datetime.strptime(timestamp, '%Y-%m-%d %H:%M:%S,%f')

        if "Starting grid.py" in line:
            grid_start_times.append(time_obj)
        elif "Finished grid.py" in line:
            grid_end_times.append(time_obj)
        elif "Starting stitcher.py" in line:
            stitcher_start_times.append(time_obj)
        elif "Finished stitcher.py" in line:
            stitcher_end_times.append(time_obj)

    # Calculate total maps generated and maps per second
    total_maps = len(grid_end_times)
    total_time = (grid_end_times[-1] - grid_start_times[0]).total_seconds() if total_maps > 0 else 0
    maps_per_second = total_maps / total_time if total_time > 0 else 0

    # Calculate average latencies for grid.py
    grid_latencies = [(end - start).total_seconds() for start, end in zip(grid_start_times, grid_end_times)]
    avg_grid_latency = sum(grid_latencies) / len(grid_latencies) if grid_latencies else 0

    # Calculate average latencies for stitcher.py
    stitcher_latencies = [(end - start).total_seconds() for start, end in zip(stitcher_start_times, stitcher_end_times)]
    avg_stitcher_latency = sum(stitcher_latencies) / len(stitcher_latencies) if stitcher_latencies else 0

    print(f"Total maps generated: {total_maps}")
    print(f"Total time: {total_time:.2f} seconds")
    print(f"Maps generated per second: {maps_per_second:.2f}")
    print(f"Average latency for grid.py: {avg_grid_latency:.2f} seconds")
    print(f"Average latency for stitcher.py: {avg_stitcher_latency:.2f} seconds")

# Analyze the log file
analyze_log('latency_log.txt')
