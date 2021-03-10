#pragma once
#include <chrono>
#include <string>

namespace icp_cov {
class TimeAnalysis {
 public:
  TimeAnalysis(const bool print = true);
  ~TimeAnalysis();
  double Stop(const std::string& name = "");
 private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  bool print_;
};
} // namespace icp_cov