#include "time_analysis.h"
#include <iostream>

namespace icp_cov {
TimeAnalysis::TimeAnalysis(const bool print) : print_(print)  {
  if (print) {
    start_ = std::chrono::high_resolution_clock::now();
  }
} 
TimeAnalysis::~TimeAnalysis() {}
double TimeAnalysis::Stop(const std::string& name) {
  if (!print_) return -1.0;
  auto stop = std::chrono::high_resolution_clock::now(); 
  std::chrono::duration<double> duration = stop - start_; 
  double time_ms = duration.count() * 1.0e+3;
  if (!name.empty()) {
    std::cout << name << " cost " << time_ms << " ms" << std::endl;
  }
  return time_ms;
} 
} // namespace icp_cov