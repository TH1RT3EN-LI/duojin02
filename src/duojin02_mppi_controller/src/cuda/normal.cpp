#include "duojin02_mppi_controller/cuda_optimizer.hpp"
#include "normal.hpp"
namespace duojin_mppi {
float deterministicNormal(uint64_t seed, uint64_t epoch, uint64_t index, unsigned axis) {
  return sharedNormal(seed, epoch, index, axis);
}
void fillDeterministicNoises(float *nx, float *ny, float *nw, size_t n,
  float sx, float sy, float sw, uint64_t seed, uint64_t epoch) {
  for (size_t i=0; i<n; ++i) {
    nx[i]=sx*sharedNormal(seed,epoch,i,0);
    ny[i]=sy*sharedNormal(seed,epoch,i,1);
    nw[i]=sw*sharedNormal(seed,epoch,i,2);
  }
}
}
