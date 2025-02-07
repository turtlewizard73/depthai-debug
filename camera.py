#!/usr/bin/env python3

# stripped from https://github.com/luxonis/depthai-python/blob/main/examples/Camera/camera_preview.py

import depthai as dai

with dai.Device() as device:
    pipeline = dai.Pipeline()
    device.startPipeline(pipeline)

    # Device name
    print('Device name:', device.getDeviceName())
    # Bootloader version
    if device.getBootloaderVersion() is not None:
        print('Bootloader version:', device.getBootloaderVersion())
    # Print out usb speed
    print('Usb speed:', device.getUsbSpeed().name)
    # Connected cameras
    print('Connected cameras:', device.getConnectedCameraFeatures())
    # create mockup pipeline

    while not device.isClosed():
        print('Camera ready!')

