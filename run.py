import subprocess
import sys
import threading
import time

def run_listener(script_name):
    subprocess.run([sys.executable, script_name])

def run_repeatedly(script_name, interval):
    while True:
        subprocess.run([sys.executable, script_name])
        time.sleep(interval)

def run_image_publisher():
    subprocess.Popen([sys.executable, 'publish.py'])

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

