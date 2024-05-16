import subprocess
import time
import concurrent.futures

commands = [
    ['ros2', 'launch', '4011_launch.py']
]

def execute_command(cmd):
    subprocess.run(cmd)

with concurrent.futures.ThreadPoolExecutor() as executor:
    executor.map(execute_command, commands)