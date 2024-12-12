// stripped from https://github.com/luxonis/depthai-core/blob/main/examples/MonoCamera/mono_preview.cpp

#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main()
{
  // Create pipeline
  dai::Pipeline pipeline;

  // Connect to device and start pipeline
  dai::Device device(pipeline);

  // Print device name
  std::cout << "Device name: " << device.getDeviceName() << std::endl;

  // Bootloader version
  if (device.getBootloaderVersion()) {
    std::cout << "Bootloader version: " << device.getBootloaderVersion()->toString() << std::endl;
  }

  // Print out USB speed
  std::cout << "Usb speed: " << device.getUsbSpeed() << std::endl;

  // Connected cameras
  auto cameras = device.getConnectedCameraFeatures();
  std::cout << "Connected cameras: ";
  for (const auto & cam : cameras) {
    std::cout << cam << " ";
  }
  std::cout << std::endl;

  // Output queues will be used to get the grayscale frames from the outputs defined above
  while (true) {
    printf("Camera ready!\n");
  }
  return 0;
}
