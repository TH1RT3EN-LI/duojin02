#include <algorithm>
#include <vector>
#include <cstdio>
#include <stdexcept>
inline size_t findClosestPathPt(const std::vector<float> & vec, float dist, size_t init = 0)
{
  // Integrated candidate travel can exceed the last path segment. The upstream
  // implementation dereferenced end() in that case, making costs depend on
  // unrelated heap contents. Keep the search within a nonempty monotone range.
  if (vec.empty()) {
    throw std::invalid_argument("findClosestPathPt requires a nonempty distance vector");
  }
  init = std::min(init, vec.size() - 1);
  auto iter = std::lower_bound(vec.begin() + init, vec.end(), dist);
  if (iter == vec.end()) {
    return vec.size() - 1;
  }
  if (iter == vec.begin() + init) {
    return init;
  }
  // Preserve upstream tie ordering: an exact midpoint selects the next point.
  if (dist - *(iter - 1) < *iter - dist) {
    return iter - 1 - vec.begin();
  }
  return iter - vec.begin();
}
int main(){std::vector<float> path{0,.1,.2,.3};std::printf("%zu\n",findClosestPathPt(path,.5,2));}
