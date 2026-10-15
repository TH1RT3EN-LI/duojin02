#include "duojin02_navigation/gpu_geometry.hpp"
#include <cuda_runtime.h>
#include <cassert>
#include <iostream>

using namespace duojin02_navigation;

__global__ void faultKernel(int *destination) {
  *destination = 1;
}

int main() {
  GeometryCompute geometry({true, 0});
  if (!geometry.available()) {
    std::cout << "CUDA device unavailable: " << geometry.deviceName() << '\n';
    return 77;
  }
  const Polygon polygon{{-.2, -.2}, {.2, -.2}, {.2, .2}, {-.2, .2}};
  const std::vector<Point> points{{1, 0, 0}};
  const std::vector<Pose> poses{{0, 0, 0}};
  geometry.setScene(polygon, points);
  const auto valid = geometry.evaluate(poses, .01);
  assert(valid.used_cuda && valid.collisions == std::vector<uint8_t>{0});

  faultKernel<<<1, 1>>>(nullptr);
  assert(cudaGetLastError() == cudaSuccess);
  assert(cudaDeviceSynchronize() == cudaErrorIllegalAddress);
  auto failure = [&]() {
    try {
      geometry.evaluate(poses, .01);
    } catch (const ComputeFailure &error) {
      return std::string(error.what());
    }
    assert(false && "CUDA execution failure must reject the geometry answer");
    return std::string{};
  };
  const auto first = failure();
  assert(first.find("CUDA") != std::string::npos);
  assert(first.find("fault latched") == std::string::npos);
  const auto repeated = failure();
  assert(repeated.find("fault latched") != std::string::npos);
  // A later tiny/empty query must not bypass a CUDA fault via CPU dispatch.
  bool empty_rejected = false;
  try {
    geometry.evaluate({}, .01);
  } catch (const ComputeFailure &error) {
    empty_rejected = std::string(error.what()).find("fault latched") != std::string::npos;
  }
  assert(empty_rejected);
  bool scene_rejected = false;
  try {
    geometry.setScene(polygon, {{0, 0, 0}});
  } catch (const ComputeFailure &error) {
    scene_rejected = std::string(error.what()).find("fault latched") != std::string::npos;
  }
  assert(scene_rejected);
  assert(geometry.statistics().cuda_batches == 1);
  assert(geometry.statistics().cpu_batches == 0);
  std::cout << "CUDA_FAULT_CHECKS first=" << first << " repeated=" << repeated << " passed\n";
}
