import subprocess
import time
import concurrent.futures

command = ['ros2', 'launch', './launch/4011_launch.py']


def execute_command():
    subprocess.run(command)

# execute_command()

# with concurrent.futures.ThreadPoolExecutor() as executor:
#     executor.map(execute_command, commands)