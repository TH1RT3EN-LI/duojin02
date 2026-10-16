#pragma once
#include <cmath>
#include <cstdint>
#ifdef __CUDACC__
#define MPPI_HD __host__ __device__
#else
#define MPPI_HD
#endif
namespace duojin_mppi {
MPPI_HD inline uint64_t mixNoise(uint64_t z) {
  z += 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}
MPPI_HD inline float sharedNormal(uint64_t seed, uint64_t epoch, uint64_t index, unsigned axis) {
  const uint64_t key = mixNoise(seed) ^ mixNoise(epoch + 0xd1b54a32d192ed03ULL) ^
    mixNoise(index * 3 + axis);
  const float u1 = (static_cast<float>(mixNoise(key) >> 40) + .5f) * (1.0f / 16777216.0f);
  const float u2 = (static_cast<float>(mixNoise(key ^ 0x94d049bb133111ebULL) >> 40) + .5f) *
    (1.0f / 16777216.0f);
  return sqrtf(-2.0f * logf(u1)) * cosf(6.2831853071795864769f * u2);
}
}
