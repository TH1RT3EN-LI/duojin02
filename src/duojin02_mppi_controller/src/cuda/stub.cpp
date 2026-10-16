#include "duojin02_mppi_controller/cuda_optimizer.hpp"
namespace duojin_mppi {
struct CudaOptimizerCore::Impl {OptimizerStatistics stats;std::string device{"built without CUDA"};};
CudaOptimizerCore::CudaOptimizerCore():impl_(new Impl){}
CudaOptimizerCore::~CudaOptimizerCore()=default;
bool CudaOptimizerCore::available()const{return false;}
const std::string &CudaOptimizerCore::deviceName()const{return impl_->device;}
const OptimizerStatistics &CudaOptimizerCore::statistics()const{return impl_->stats;}
void CudaOptimizerCore::rollout(const CoreSettings &,CoreBuffers,CorePose,uint64_t,uint64_t,const float *,const float *,const float *){throw OptimizerComputeError("MPPI built without CUDA");}
void CudaOptimizerCore::update(const CoreSettings &,CoreBuffers){throw OptimizerComputeError("MPPI built without CUDA");}
void CudaOptimizerCore::scoreCritic(unsigned,float,unsigned,float,float,float,float,float,float,float *){throw OptimizerComputeError("MPPI built without CUDA");}
}
