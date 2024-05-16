import subprocess
import time
import concurrent.futures

command = ['ros2', 'launch', './python_processing/launch/4011_launch.py']


def execute_command():
    print("Hello")
    subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)

# execute_command()

# with concurrent.futures.ThreadPoolExecutor() as executor:
#     executor.map(execute_command, commands)