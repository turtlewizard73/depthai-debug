import os
import signal
import subprocess
import time
import argparse

# Argument parsing
parser = argparse.ArgumentParser(description="Run camera crash test with specified launch command.")
parser.add_argument('-p', '--python', action='store_true', help="Use the Python script to launch the camera.")
parser.add_argument('-c', '--camera', action='store_true', help="Use the compiled camera binary to launch the camera.")
args = parser.parse_args()

# Configuration
if args.python:
    print("Using Python script to launch the camera.")
    camera_launch_cmd = ['python3', './camera.py']
elif args.camera:
    print("Using compiled camera binary to launch the camera.")
    camera_launch_cmd = ['./build/camera']
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


def stress_test():
    """Repeatedly launch the camera process and send SIGTERM/SIGKILL to it."""
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
        while True:
            line = process.stdout.readline().strip()
            if not line:  # Check for an empty line (EOF or process terminated)
                time.sleep(1)
                continue
            # print(line)  # Print each line as it comes
            if camera_launched_identifier in line:
                print(line)
                print("_____ Camera launched successfully. _____")
                break

        time.sleep(2)

        # Send SIGKILL to the camera process
        print("Sending SIGKILL to the camera process...")
        os.kill(process.pid, signal.SIGKILL)

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

        # Ensure process cleanup
        process.wait()
        print("Process terminated cleanly.\n")


def main():
    try:
        stress_test()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    return 0


if __name__ == "__main__":
    exit(main())