#pragma once
#include <gimbalrotor/perching_trajectory.h>

namespace roll_yaw
{
struct AdmittanceConfig
{
  double inertia = .40, damping = 5, stiffness = 4.5, torque_limit = .18, angle_limit = .0383972;
  double rate_limit = .15, acceleration_limit = .1, reference_torque = 0;
  double filter_time = .1, on_threshold = .04, off_threshold = .02, on_dwell = .1, off_dwell = .2;
  int baseline_samples = 80;
  bool valid() const;
};
class PitchAdmittance
{
public:
  AdmittanceConfig config;
  bool enabled = false, contact = false, baseline_ready = false;
  double bias = 0, filtered = 0, offset = 0, rate = 0, acceleration = 0;
  int samples = 0;
  void reset();
  void observe(double torque, double stamp, bool no_cut);
  bool step(double dt, bool input_fresh, bool relief_only = false, double available_relief = -1);

private:
  double sum_ = 0, stamp_ = 0, on_ = 0, off_ = 0;
};
// Local constrained reference synthesis; never edits a nominal Path or goal.
bool compliantReference(const Kinematics&, const Reference& nominal, const Reference& previous, double offset,
                        double rate, double acceleration, double dt, const PlanningConfig&, Reference&, std::string&);
bool advanceCompliance(const Kinematics&, const Reference& nominal, const Reference& previous, double dt, bool fresh,
                       const PlanningConfig&, PitchAdmittance&, Reference&, std::string&);
}  // namespace roll_yaw
