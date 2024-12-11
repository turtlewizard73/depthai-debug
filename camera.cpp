#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main()
{
  // Create pipeline
  dai::Pipeline pipeline;

  // Connect to device and start pipeline
  dai::Device device(pipeline);

  // Output queues will be used to get the grayscale frames from the outputs defined above
  while (true) {
    printf("Camera ready!\n");
  }
  return 0;
}
