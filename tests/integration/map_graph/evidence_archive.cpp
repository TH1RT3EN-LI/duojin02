// Run the canonical ownership/LinkInfo-v2 archive checks against the installed
// production Karto library, without re-running the covariance Monte Carlo suite.
#define main complete_model_probe_main
#include "../karto_model_evidence/model_probe.cpp"
#undef main
int main(int argc,char **argv) {
  try {
    if(argc!=2)throw std::runtime_error("usage: evidence_archive OUTPUT_DIRECTORY");
    output_directory=argv[1];
    std::filesystem::create_directories(output_directory);
    std::cout<<std::setprecision(17);
    lifecycle();
    actualOwnership();
    return 0;
  } catch(const karto::Exception &error) {
    std::cerr<<error.GetErrorMessage()<<'\n';return 1;
  } catch(const std::exception &error) {
    std::cerr<<error.what()<<'\n';return 1;
  }
}
