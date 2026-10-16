#include <algorithm>
#include <vector>
#include <cstdio>
inline size_t findClosestPathPt(const std::vector<float> &vec,float dist,size_t init=0) {
  auto iter=std::lower_bound(vec.begin()+init,vec.end(),dist);
  if(iter==vec.begin()+init)return 0;
  if(dist-*(iter-1)<*iter-dist)return iter-1-vec.begin();
  return iter-vec.begin();
}
int main(){std::vector<float> path{0,.1,.2,.3};std::printf("%zu\n",findClosestPathPt(path,.5,2));}
