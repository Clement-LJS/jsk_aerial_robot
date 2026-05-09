// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <mutex>
#include <string>
#include <vector>

class GimbalrotorMultilinkRobotModel : public GimbalrotorRobotModel
{
public:
  GimbalrotorMultilinkRobotModel(bool init_with_rosparam = true,
                                 bool verbose = false,
                                 double fc_t_min_thre = 0,
                                 double epsilon = 10);

  virtual ~GimbalrotorMultilinkRobotModel() = default;

  bool hasMovableBodyJoints() const
  {
    return has_movable_body_joints_;
  }

  const std::vector<std::string>& getMovableBodyJointNames() const
  {
    return movable_body_joint_names_;
  }

  const std::vector<std::string>& getMovableBodySegmentNames() const
  {
    return movable_body_segment_names_;
  }

  std::vector<KDL::Rotation> getBodyLinksRotationFromCog()
  {
    std::lock_guard<std::mutex> lock(body_links_rotation_mutex_);
    return body_links_rotation_from_cog_;
  }

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

private:
  void detectMovableBodySegments();
  bool isTargetBodyJointForMultilinkDetection(const std::string& joint_name) const;
  
  bool has_movable_body_joints_;
  bool movable_body_joint_detection_done_;

  std::vector<std::string> movable_body_joint_names_;
  std::vector<std::string> movable_body_segment_names_;

  std::vector<KDL::Rotation> body_links_rotation_from_cog_;
  std::mutex body_links_rotation_mutex_;
};
