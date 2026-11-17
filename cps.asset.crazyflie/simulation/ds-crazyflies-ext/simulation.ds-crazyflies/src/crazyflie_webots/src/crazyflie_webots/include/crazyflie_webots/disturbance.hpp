#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>

namespace ds_motion {
// Same half-open cells as Java GridGraph.cell(); no position clamping.
struct TrialGrid {
  Eigen::Vector3d origin=Eigen::Vector3d::Zero(), step=Eigen::Vector3d::Ones();
  int rows=1, cols=1, layers=1;
  void validate() const {
    if (!origin.allFinite() || !step.allFinite() || step.minCoeff()<=0
        || rows<1 || cols<1 || layers<1
        || static_cast<double>(rows)*cols*layers>2147483647)
      throw std::invalid_argument("Invalid observation grid geometry");
  }
  int cell(const Eigen::Vector3d& p) const {
    if (!p.allFinite()) return -1;
    Eigen::Vector3d q=(p-origin).cwiseQuotient(step);
    const double x=std::floor(q.x()+.5), y=std::floor(q.y()+.5), z=std::floor(q.z());
    if (x<0 || x>=cols || y<0 || y>=rows || z<0 || z>=layers) return -1;
    return static_cast<int>(z)*rows*cols+static_cast<int>(x)*rows+static_cast<int>(y);
  }
};

// One aircraft's replayable wind schedule. It never inspects flight policy.
struct WindPulse {
  double start, ramp, hold;
  Eigen::Vector3d peak;
  double end() const { return start+2*ramp+hold; }
};
class Disturbance {
 public:
  void arm(std::vector<WindPulse> pulses, double now) {
    if(!std::isfinite(now)) throw std::invalid_argument("Invalid schedule time");
    std::sort(pulses.begin(),pulses.end(),[](const auto& a,const auto& b){return a.start<b.start;});
    for(std::size_t i=0;i<pulses.size();++i) {
      const auto& p=pulses[i];
      if(!std::isfinite(p.start+p.ramp+p.hold) || p.start<now || p.ramp<=0 || p.hold<0
          || 2*p.ramp+p.hold>60 || !p.peak.allFinite() || p.peak.norm()>1
          || (i>0 && p.start<pulses[i-1].end()))
        throw std::invalid_argument("Invalid or overlapping wind pulses (maximum 1 m/s)");
    }
    pulses_=std::move(pulses);
  }
  void cancel() { pulses_.clear(); }
  Eigen::Vector3d at(double now) const {
    for(const auto& p:pulses_) {
      if(now<p.start) break;
      const double t=now-p.start;
      if(now>=p.end()) continue;
      double gain=1;
      if(t<p.ramp) gain=smooth(t/p.ramp);
      else if(t>=p.ramp+p.hold) gain=1-smooth((t-p.ramp-p.hold)/p.ramp);
      return p.peak*gain;
    }
    return Eigen::Vector3d::Zero();
  }
 private:
  static double smooth(double u) { return u*u*u*u*(35+u*(-84+u*(70-20*u))); }
  std::vector<WindPulse> pulses_;
};
}
