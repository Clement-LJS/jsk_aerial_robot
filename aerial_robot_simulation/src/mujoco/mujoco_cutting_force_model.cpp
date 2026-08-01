#include <aerial_robot_simulation/mujoco/mujoco_cutting_force_model.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace aerial_robot_simulation
{
namespace
{
bool finiteScalar(const double value)
{
  return std::isfinite(value);
}

double clampScalar(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(value, upper));
}
}  // namespace

MujocoCuttingForceModel::MujocoCuttingForceModel()
  : configured_(false)
  , filtered_force_(0.0)
  , ramp_state_(0.0)
  , rng_(1)
  , noise_dist_(0.0, 1.0)
{
}

bool MujocoCuttingForceModel::configure(const Profile& profile, const Config& config, std::string* error)
{
  if(!validateProfile(profile, error) || !validateConfig(config, error))
    {
      configured_ = false;
      return false;
    }

  profile_ = profile;
  config_ = config;
  configured_ = true;
  rng_.seed(config_.noise_seed);
  noise_dist_ = std::normal_distribution<double>(0.0, config_.noise_stddev);
  reset();
  return true;
}

void MujocoCuttingForceModel::reset()
{
  filtered_force_ = 0.0;
  ramp_state_ = 0.0;
}

bool MujocoCuttingForceModel::isConfigured() const
{
  return configured_;
}

double MujocoCuttingForceModel::totalThickness() const
{
  double total = 0.0;
  for(const auto& layer : profile_.layers)
    {
      total += layer.thickness;
    }
  return total;
}

const MujocoCuttingForceModel::Profile& MujocoCuttingForceModel::profile() const
{
  return profile_;
}

const MujocoCuttingForceModel::Config& MujocoCuttingForceModel::config() const
{
  return config_;
}

MujocoCuttingForceModel::Output MujocoCuttingForceModel::update(const Input& input)
{
  const bool dt_valid = finiteScalar(input.dt) && input.dt > 0.0 && input.dt <= config_.maximum_control_dt;
  if(!configured_ || !dt_valid || !input.enabled || !input.cutting_active || !input.valid_geometry ||
     !finiteScalar(input.signed_travel) || !finiteScalar(input.tangential_velocity))
    {
      reset();
      return zeroOutput(dt_valid);
    }

  const double penetration = std::max(0.0, input.signed_travel - config_.free_travel);
  Output output;
  output.active = true;
  output.dt_valid = true;
  output.penetration = penetration;
  output.penetration_velocity = std::max(0.0, input.tangential_velocity);
  output.contact = penetration > 0.0;
  output.completed = penetration >= totalThickness();
  output.layer_index = computeLayerIndex(penetration);

  if(!output.contact)
    {
      reset();
      output.filtered_force = 0.0;
      output.applied_force = 0.0;
      output.raw_force = 0.0;
      output.layer_index = -1;
      return output;
    }

  const double elastic_force = computeElasticForce(penetration);
  const double damping_force = computeActiveDamping(penetration) * std::max(0.0, input.tangential_velocity);
  double ripple_force = 0.0;
  if(config_.free_travel < input.signed_travel && profile_.ripple_wavelength > std::numeric_limits<double>::epsilon())
    {
      ripple_force = profile_.ripple_amplitude * std::sin((2.0 * M_PI * penetration) / profile_.ripple_wavelength);
    }

  double noise_force = 0.0;
  if(config_.noise_enabled && config_.noise_stddev > 0.0)
    {
      noise_force = noise_dist_(rng_);
    }

  output.raw_force = clampScalar(elastic_force + damping_force + ripple_force + noise_force, 0.0, profile_.maximum_force);

  if(config_.force_filter_time_constant > 0.0)
    {
      const double alpha = clampScalar(input.dt / (config_.force_filter_time_constant + input.dt), 0.0, 1.0);
      filtered_force_ += alpha * (output.raw_force - filtered_force_);
    }
  else
    {
      filtered_force_ = output.raw_force;
    }

  if(config_.force_ramp_time > 0.0)
    {
      ramp_state_ = clampScalar(ramp_state_ + input.dt / config_.force_ramp_time, 0.0, 1.0);
    }
  else
    {
      ramp_state_ = 1.0;
    }

  output.filtered_force = filtered_force_;
  output.applied_force = filtered_force_ * ramp_state_;
  return output;
}

bool MujocoCuttingForceModel::validateProfile(const Profile& profile, std::string* error) const
{
  if(!finiteScalar(profile.preload_force) || profile.preload_force < 0.0)
    {
      if(error) *error = "invalid preload_force";
      return false;
    }
  if(!finiteScalar(profile.maximum_force) || profile.maximum_force < 0.0)
    {
      if(error) *error = "invalid maximum_force";
      return false;
    }
  if(!finiteScalar(profile.ripple_amplitude) || !finiteScalar(profile.ripple_wavelength))
    {
      if(error) *error = "invalid ripple parameters";
      return false;
    }
  if(profile.ripple_wavelength < 0.0)
    {
      if(error) *error = "ripple_wavelength must be >= 0";
      return false;
    }
  if(profile.layers.empty())
    {
      if(error) *error = "at least one layer is required";
      return false;
    }

  for(const auto& layer : profile.layers)
    {
      if(!finiteScalar(layer.thickness) || !finiteScalar(layer.stiffness) || !finiteScalar(layer.damping) ||
         layer.thickness <= 0.0 || layer.stiffness < 0.0 || layer.damping < 0.0)
        {
          if(error) *error = "invalid layer parameters";
          return false;
        }
    }

  return true;
}

bool MujocoCuttingForceModel::validateConfig(const Config& config, std::string* error) const
{
  if(!finiteScalar(config.free_travel) || config.free_travel < 0.0)
    {
      if(error) *error = "invalid free_travel";
      return false;
    }
  if(!finiteScalar(config.force_ramp_time) || config.force_ramp_time < 0.0)
    {
      if(error) *error = "invalid force_ramp_time";
      return false;
    }
  if(!finiteScalar(config.force_filter_time_constant) || config.force_filter_time_constant < 0.0)
    {
      if(error) *error = "invalid force_filter_time_constant";
      return false;
    }
  if(!finiteScalar(config.maximum_control_dt) || config.maximum_control_dt <= 0.0)
    {
      if(error) *error = "invalid maximum_control_dt";
      return false;
    }
  if(!finiteScalar(config.noise_stddev) || config.noise_stddev < 0.0)
    {
      if(error) *error = "invalid noise_stddev";
      return false;
    }
  return true;
}

MujocoCuttingForceModel::Output MujocoCuttingForceModel::zeroOutput(const bool dt_valid) const
{
  Output output;
  output.dt_valid = dt_valid;
  return output;
}

int MujocoCuttingForceModel::computeLayerIndex(const double penetration) const
{
  if(penetration <= 0.0)
    {
      return -1;
    }

  double accumulated = 0.0;
  for(std::size_t i = 0; i < profile_.layers.size(); ++i)
    {
      accumulated += profile_.layers[i].thickness;
      if(penetration <= accumulated)
        {
          return static_cast<int>(i);
        }
    }
  return static_cast<int>(profile_.layers.size()) - 1;
}

double MujocoCuttingForceModel::computeElasticForce(const double penetration) const
{
  double force = profile_.preload_force;
  double accumulated = 0.0;
  for(const auto& layer : profile_.layers)
    {
      const double local = clampScalar(penetration - accumulated, 0.0, layer.thickness);
      force += layer.stiffness * local;
      accumulated += layer.thickness;
    }
  return force;
}

double MujocoCuttingForceModel::computeActiveDamping(const double penetration) const
{
  double accumulated = 0.0;
  for(const auto& layer : profile_.layers)
    {
      accumulated += layer.thickness;
      if(penetration <= accumulated)
        {
          return layer.damping;
        }
    }
  if(profile_.layers.empty())
    {
      return 0.0;
    }
  return profile_.layers.back().damping;
}
}  // namespace aerial_robot_simulation
