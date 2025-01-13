import os
import signal
import subprocess
import time
import argparse
import re
import psutil

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
import threading

# Argument parsing
parser = argparse.ArgumentParser(description="Run camera crash test with specified launch command.")
parser.add_argument('-p', '--python', action='store_true', help="Use the Python script to launch the camera.")
parser.add_argument('-c', '--camera', action='store_true', help="Use the compiled camera binary to launch the camera.")
parser.add_argument('-r', '--ros', action='store_true', help="Use the ROS camera node to launch the camera.")
parser.add_argument('-e', '--enjoy', action='store_true')
args = parser.parse_args()

# Configuration
if args.python:
    print("Using Python script to launch the camera.")
    camera_launch_cmd = ['python3', './camera.py']
    FIRST_N_LINES_TO_PRINT = 5
elif args.camera:
    print("Using compiled camera binary to launch the camera.")
    camera_launch_cmd = ['./build/camera']
    FIRST_N_LINES_TO_PRINT = 5
elif args.ros:
    print("Using ROS camera node to launch the camera.")
    camera_launch_cmd = ['ros2', 'launch', 'depthai_examples', 'stereo_inertial_node.launch.py', 'enableRviz:=false']
    FIRST_N_LINES_TO_PRINT = 25
elif args.enjoy:
    print("Using custom pipeline")
    camera_launch_cmd = ["ros2", "launch", "service_robot_bringup", "oak_camera_simple.launch.py"]
    FIRST_N_LINES_TO_PRINT = 50
else:
    raise ValueError("You must specify either -p or -c to choose the launch command.")


num_trials = 500  # Number of SIGKILL tests
usb_check_cmd = "lsusb"

camera_launched_identifier = "Camera ready!"
camera_identifier = ["Intel Myriad", "Intel Luxonis Bootloader"]


def is_camera_connected():
    """Check if the camera is still listed in the USB stack."""
    try:
        lsusb_output = subprocess.check_output(usb_check_cmd, shell=True).decode('utf-8')
        for line in lsusb_output.splitlines():
            if any(identifier in line for identifier in camera_identifier):
                print(f"Camera found: {line}")
                return True
        return False
    except subprocess.CalledProcessError as e:
        print(f"Error running lsusb: {e}")
        return False


def kill_ros():
    """Kill all ROS processes using SIGKILL."""

    GREP_PATTERN = r"ros|gazebo|gzserver|gzclient|ros-args"
    GREP_IGNORE_PATTERN = r"grep|rqt|rviz|teleop|ros2 topic"

    # Find all processes matching the GREP_PATTERN
    pids = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            if any(re.search(GREP_PATTERN, arg) for arg in proc.info['cmdline']) and \
               not any(re.search(GREP_IGNORE_PATTERN, arg) for arg in proc.info['cmdline']):
                pids.append(proc.info['pid'])
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    if not pids:
        print("No ROS processes found")
        return

    while pids:
        for pid in pids:
            try:
                proc = psutil.Process(pid)
                proc.terminate()  # SIGTERM
                print(f"Terminated process {pid}: {proc.name()}")
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

        time.sleep(1)
        # Re-check if there are any remaining processes
        pids = []
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                if any(re.search(GREP_PATTERN, arg) for arg in proc.info['cmdline']) and \
                   not any(re.search(GREP_IGNORE_PATTERN, arg) for arg in proc.info['cmdline']):
                    pids.append(proc.info['pid'])
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass


class StopCameraNode(Node):
    def __init__(self):
        super().__init__('stop_camera_node')
        self.cli = self.create_client(Trigger, '/camera/oak_top/stop_camera')
        self.req = Trigger.Request()

    def call_stop_camera_service(self):
        self.get_logger().info('Calling /camera/oak_top/stop_camera service...')
        try:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Service not available, waiting...')
            response = self.cli.call(self.req)
            if response:
                self.get_logger().info(f'Response: {response.message}')
            else:
                self.get_logger().error('Service call returned no response.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def spin_executor(executor):
    try:
        executor.spin()
    except rclpy.executors.ExternalShutdownException:
        print('Executor failed to spin')


def stress_test():
    """Repeatedly launch the camera process and send SIGTERM/SIGKILL to it."""
    # run node
    rclpy.init()

    stop_node = StopCameraNode()
    executor = MultiThreadedExecutor()
    executor.add_node(stop_node)

    thread = threading.Thread(target=spin_executor, args=(executor, ), daemon=True)
    thread.start()

    try:
        for trial in range(1, num_trials + 1):
            print(f"_____ Trial {trial}/{num_trials} _____")

            # Check if the camera is recognized
            if not is_camera_connected():
                print(f"Camera dropped from USB stack before trial {trial}.")
                break
            else:
                print("Camera detected.")

            # Launch the camera process
            process = subprocess.Popen(camera_launch_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            # Wait for the camera to launch
            first_n_lines_to_print = FIRST_N_LINES_TO_PRINT
            while True:
                line = process.stdout.readline().strip()
                if not line:  # Check for an empty line (EOF or process terminated)
                    time.sleep(1)
                    continue
                if first_n_lines_to_print > 0:
                    print(line)
                    first_n_lines_to_print -= 1
                if camera_launched_identifier in line:
                    print(line)
                    print("_____ Camera launched successfully. _____")
                    break

            time.sleep(2)

            # call stop camera service
            if args.enjoy is True:
                print("_____ Calling /camera/oak_top/stop_camera service _____")
                stop_node.call_stop_camera_service()
                time.sleep(2)

            # Send SIGKILL to the camera process
            print("Sending SIGKILL to the camera process...")
            kill_ros()
            os.kill(process.pid, signal.SIGTERM)

            # Ensure process cleanup
            process.wait()
            print("Process terminated cleanly.\n")

            # print("_____ Killing all ROS processes. _____")
            # killing ros processes manually so container doesn't get stuck
            # kill_ros()

            # stdout, stderr = process.communicate()
            # print(stdout)
            # print(stderr)

            # Allow time for USB stack to recover
            time.sleep(5)

            # Check if the camera is still recognized
            if not is_camera_connected():
                print(f"Camera dropped from USB stack after trial {trial}.")
                break
            else:
                print("Camera still detected.")
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")

    stop_node.destroy_node()
    executor.shutdown()
    thread.join()
    rclpy.shutdown()


def main():
    stress_test()


if __name__ == "__main__":
    exit(main())