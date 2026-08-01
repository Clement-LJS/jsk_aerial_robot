#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace aerial_robot_simulation
{
class MujocoCuttingForceModel
{
public:
  struct Layer
  {
    double thickness = 0.0;
    double stiffness = 0.0;
    double damping = 0.0;
  };

  struct Profile
  {
    double preload_force = 0.0;
    double maximum_force = 0.0;
    double ripple_amplitude = 0.0;
    double ripple_wavelength = 0.0;
    std::vector<Layer> layers;
  };

  struct Config
  {
    double free_travel = 0.0;
    double force_ramp_time = 0.0;
    double force_filter_time_constant = 0.0;
    double maximum_control_dt = 0.1;
    bool noise_enabled = false;
    double noise_stddev = 0.0;
    unsigned int noise_seed = 1;
  };

  struct Input
  {
    bool enabled = false;
    bool cutting_active = false;
    bool valid_geometry = false;
    double dt = 0.0;
    double signed_travel = 0.0;
    double tangential_velocity = 0.0;
  };

  struct Output
  {
    bool active = false;
    bool contact = false;
    bool completed = false;
    bool dt_valid = false;
    int layer_index = -1;
    double penetration = 0.0;
    double penetration_velocity = 0.0;
    double raw_force = 0.0;
    double filtered_force = 0.0;
    double applied_force = 0.0;
  };

  MujocoCuttingForceModel();

  bool configure(const Profile& profile, const Config& config, std::string* error = nullptr);
  void reset();
  bool isConfigured() const;
  double totalThickness() const;
  const Profile& profile() const;
  const Config& config() const;
  Output update(const Input& input);

private:
  bool validateProfile(const Profile& profile, std::string* error) const;
  bool validateConfig(const Config& config, std::string* error) const;
  Output zeroOutput(bool dt_valid) const;
  int computeLayerIndex(double penetration) const;
  double computeElasticForce(double penetration) const;
  double computeActiveDamping(double penetration) const;

  Profile profile_;
  Config config_;
  bool configured_;
  double filtered_force_;
  double ramp_state_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_dist_;
};
}  // namespace aerial_robot_simulation
