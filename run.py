import subprocess
import sys
import threading
import time
import logging

# Set up logging
logging.basicConfig(filename='Test/latency_log.txt', level=logging.INFO, format='%(asctime)s - %(message)s')

def log_time(event):
    logging.info(event)

def run_listener(script_name):
    log_time(f'Starting {script_name}')
    subprocess.run([sys.executable, script_name])
    log_time(f'Finished {script_name}')

def run_repeatedly(script_name, interval):
    while True:
        start_time = time.time()
        log_time(f'Starting {script_name}')
        subprocess.run([sys.executable, script_name])
        end_time = time.time()
        log_time(f'Finished {script_name}')
        log_time(f'{script_name} took {end_time - start_time} seconds')
        time.sleep(interval)

def run_image_publisher():
    while True:
        start_time = time.time()
        log_time('Starting publish.py')
        subprocess.run([sys.executable, 'publish.py'])
        end_time = time.time()
        log_time('Finished publish.py')
        log_time(f'publish.py took {end_time - start_time} seconds')
        time.sleep(1)  # Adjust the interval as needed

if __name__ == "__main__":
    listener_thread = threading.Thread(target=run_listener, args=('listener.py',))
    script1_thread = threading.Thread(target=run_repeatedly, args=('stitcher.py', 1))
    script2_thread = threading.Thread(target=run_repeatedly, args=('grid.py', 1))
    publisher_thread = threading.Thread(target=run_image_publisher)

    listener_thread.start()
    script1_thread.start()
    script2_thread.start()
    publisher_thread.start()

    listener_thread.join()
    script1_thread.join()
    script2_thread.join()
    publisher_thread.join()
