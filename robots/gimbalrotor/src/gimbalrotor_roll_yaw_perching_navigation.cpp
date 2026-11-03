#include <gimbalrotor/gimbalrotor_roll_yaw_perching_navigation.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>
#include <algorithm>
#include <sstream>
#include <iomanip>

namespace aerial_robot_navigation
{
using namespace roll_yaw;
namespace
{
tf::Vector3 tfVector(const Vec& v) { return {v.x(),v.y(),v.z()}; }
tf::Quaternion tfQuaternion(const Mat& r) { Eigen::Quaterniond q(r); return {q.x(),q.y(),q.z(),q.w()}; }
geometry_msgs::Pose poseMsg(const Pose& p)
{
  geometry_msgs::Pose m; tf::pointTFToMsg(tfVector(p.translation()),m.position);
  tf::quaternionTFToMsg(tfQuaternion(p.linear()),m.orientation); return m;
}
std::string frameName(std::string s) { if(!s.empty()&&s.front()=='/') s.erase(0,1); return s; }
bool readVector(ros::NodeHandle& nh,const std::string& key,Vec& value,bool required=false)
{
  if(!nh.hasParam(key)) return !required;
  std::vector<double> v;
  if(!nh.getParam(key,v)||v.size()!=3) return false;
  value=Vec(v[0],v[1],v[2]); return value.allFinite();
}
}
void GimbalrotorRollYawPerchingNavigator::initialize(ros::NodeHandle nh,ros::NodeHandle nhp,
  boost::shared_ptr<aerial_robot_model::RobotModel> model,boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,double dt)
{
  GimbalrotorPerchingNavigator::initialize(nh,nhp,model,estimator,dt);
  ros::NodeHandle pn(nh,"navigation/roll_yaw_perching");
  pn.param<std::string>("roll_joint",roll_,"joint_roll"); pn.param<std::string>("yaw_joint",yaw_,"joint_yaw");
  pn.param<std::string>("contact_link",contact_,"roll_hand_link");
  pn.param<std::string>("debug_prefix",prefix_,"perching/roll_yaw"); pn.param<std::string>("world_frame",world_,"world");
  std::string tf_prefix; nhp.param<std::string>("tf_prefix",tf_prefix,"");
  pn.param<std::string>("body_frame",body_frame_,tf::resolve(tf_prefix,robot_model_->getBaselinkName()));
  Vec xyz=Vec::Zero(),rpy=Vec::Zero();
  configuration_valid_=readVector(pn,"pivot_offset",xyz,true)&&readVector(pn,"pivot_rpy",rpy,true)&&
    readVector(pn,"hinge_axis",axis_,true)&&std::abs(axis_.norm()-1)<1e-7;
  cp_.translation()=xyz; cp_.linear()=expSO3(Vec::UnitZ()*rpy.z())*expSO3(Vec::UnitY()*rpy.y())*expSO3(Vec::UnitX()*rpy.x());
  auto& p=planning_config_; auto& c=feedback_config_;
  pn.param("qp_planning",qp_planning_,false);
  ROS_INFO_STREAM("[RollYaw] "<<pn.getNamespace()<<"/qp_planning="<<std::boolalpha<<qp_planning_
                  <<"; manual coordinates always use direct FK [alpha, roll, yaw]");
  const bool vectors=readVector(pn,"position_lower",p.limits.lower)&&readVector(pn,"position_upper",p.limits.upper);
  const bool qp_vectors=!qp_planning_||(readVector(pn,"qp_weights",p.qp.weight)&&
    readVector(pn,"velocity_caps",p.limits.velocity)&&readVector(pn,"acceleration_caps",p.limits.acceleration));
  pn.param("planning_dt",p.dt,p.dt); pn.param("body_max_tilt",p.max_tilt,p.max_tilt);
  pn.param("orientation_tolerance",p.orientation_tolerance,p.orientation_tolerance);
  pn.param("rate_tolerance",p.rate_tolerance,p.rate_tolerance);
  pn.param("alignment_rate",p.alignment_rate,p.alignment_rate);
  pn.param("alignment_acceleration",p.alignment_acceleration,p.alignment_acceleration);
  pn.param("max_phase_duration",p.max_duration,p.max_duration);
  pn.param("scaling_attempts",p.scaling_attempts,p.scaling_attempts);
  pn.param("cog_speed_limit",p.cog_speed_limit,p.cog_speed_limit);
  pn.param("cog_acceleration_limit",p.cog_acceleration_limit,p.cog_acceleration_limit);
  pn.param("qp_regularization",p.qp.regularization,p.qp.regularization);
  pn.param("qp_smoothing",p.qp.smoothing,p.qp.smoothing); pn.param("orientation_gain",p.qp.gain,p.qp.gain);
  pn.param("min_singular_value",p.qp.min_singular_value,p.qp.min_singular_value);
  pn.param("feedback_timeout",c.timeout,c.timeout); pn.param("future_tolerance",c.future_tolerance,c.future_tolerance);
  pn.param("pivot_tolerance",c.pivot_tolerance,c.pivot_tolerance); pn.param("hinge_residual_tolerance",c.hinge_tolerance,c.hinge_tolerance);
  pn.param("tracking_tolerance",c.tracking_tolerance,c.tracking_tolerance);
  pn.param("body_tracking_tolerance",c.orientation_tolerance,c.orientation_tolerance);
  pn.param("settle_rate",c.settle_rate,c.settle_rate); pn.param("settle_time",c.settle_time,c.settle_time);
  pn.param("settle_timeout",c.settle_timeout,c.settle_timeout);
  c.require_settled_on_enable=false; // Same measured-pose capture policy as multilink.
  pn.param("feedback_sync_tolerance",synchronization_tolerance_,synchronization_tolerance_);
  pn.param("cog_model_tolerance",cog_model_tolerance_,cog_model_tolerance_);
  qp_configuration_valid_=qp_planning_&&qp_vectors&&p.valid()&&c.valid()&&c.settlingValid();
  configuration_valid_=configuration_valid_&&poseValid(cp_)&&vectors&&p.mechanismValid()&&c.feedbackConfigValid()&&
    std::isfinite(synchronization_tolerance_)&&synchronization_tolerance_>0&&
    std::isfinite(cog_model_tolerance_)&&cog_model_tolerance_>0;
  pn.param("pitch_rezero_target_pitch",rezero_pitch_,rezero_pitch_);
  pn.param("pitch_rezero_kp",rezero_kp_,rezero_kp_);
  pn.param("pitch_rezero_command_sign",rezero_sign_,rezero_sign_);
  pn.param("pitch_rezero_rate_limit",rezero_rate_,rezero_rate_);
  pn.param("pitch_rezero_max_delta",rezero_travel_,rezero_travel_);
  pn.param("pitch_rezero_pitch_tolerance",rezero_tolerance_,rezero_tolerance_);
  pn.param("pitch_rezero_rate_tolerance",rezero_rate_tolerance_,rezero_rate_tolerance_);
  pn.param("pitch_rezero_stable_duration",rezero_stable_duration_,rezero_stable_duration_);
  pn.param("pitch_rezero_timeout",rezero_timeout_,rezero_timeout_);
  pn.param("pitch_rezero_roll_joint_hold_tolerance",rezero_roll_tolerance_,rezero_roll_tolerance_);
  pn.param("pitch_rezero_yaw_joint_hold_tolerance",rezero_yaw_tolerance_,rezero_yaw_tolerance_);
  rezero_configuration_valid_=std::isfinite(rezero_pitch_)&&std::abs(rezero_pitch_)<M_PI/2;
  for(double value:{rezero_kp_,rezero_rate_,rezero_travel_,rezero_tolerance_,rezero_rate_tolerance_,
                    rezero_stable_duration_,rezero_timeout_,rezero_roll_tolerance_,rezero_yaw_tolerance_})
    rezero_configuration_valid_=rezero_configuration_valid_&&std::isfinite(value)&&value>0;
  rezero_configuration_valid_=rezero_configuration_valid_&&rezero_travel_<=M_PI&&
    rezero_timeout_>=rezero_stable_duration_&&rezero_roll_tolerance_<=.5&&rezero_yaw_tolerance_<=.5&&
    std::isfinite(rezero_sign_)&&rezero_sign_!=0;
  if(rezero_configuration_valid_) rezero_sign_=rezero_sign_<0?-1:1;
  bool underactuated=false; nh.param("controller/underactuate",underactuated,false);
  std::string controller; nh.getParam("aerial_robot_control_name",controller);
  if(underactuated||controller!="aerial_robot_control/gimbalrotor_controller") configuration_valid_=false;
  std::string joint_topic,command_topic,body_topic;
  pn.param<std::string>("joint_state_topic",joint_topic,"joint_states");
  pn.param<std::string>("joint_command_topic",command_topic,"joints_ctrl");
  pn.param<std::string>("body_state_topic",body_topic,"uav/baselink/odom");
  joints_pub_=nh.advertise<sensor_msgs::JointState>(command_topic,1);
  state_pub_=nh.advertise<std_msgs::String>(prefix_+"/state",1,true);
  session_pub_=nh.advertise<std_msgs::UInt64>(prefix_+"/session",1,true);
  details_pub_=nh.advertise<std_msgs::String>(prefix_+"/diagnostics",1);
  nominal_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/nominal_angles",1);
  position_lower_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/position_lower",1);
  position_upper_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/position_upper",1);
  angles_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/target_angles",1);
  rezero_active_pub_=nh.advertise<std_msgs::Bool>(prefix_+"/pitch_rezero_active",1,true);
  rezero_ready_pub_=nh.advertise<std_msgs::Bool>(prefix_+"/pitch_rezero_ready",1,true);
  rezero_failed_pub_=nh.advertise<std_msgs::Bool>(prefix_+"/pitch_rezero_failed",1,true);
  pitch_pub_=nh.advertise<std_msgs::Float64>("perching/commanded_pitch_delta",1);
  reference_pub_=nh.advertise<nav_msgs::Odometry>(prefix_+"/body_reference",1);
  measured_pub_=nh.advertise<nav_msgs::Odometry>(prefix_+"/body_measured",1);
  cog_pub_=nh.advertise<nav_msgs::Odometry>(prefix_+"/cog_reference",1);
  subscriptions_.push_back(nh.subscribe(joint_topic,1,&GimbalrotorRollYawPerchingNavigator::jointCallback,this));
  subscriptions_.push_back(nh.subscribe(body_topic,1,&GimbalrotorRollYawPerchingNavigator::bodyCallback,this));
  for(const auto* key:{"prepare","align","start","pause","resume","cancel"}) {
    std::string action(key);
    subscriptions_.push_back(nh.subscribe<std_msgs::UInt64>(prefix_+"/"+action,1,
      [this,action](const std_msgs::UInt64ConstPtr& m){request(action,m->data);}));
  }
  for(int axis=0;axis<3;++axis) {
    const std::string name=axis==0?"alpha_target":axis==1?"roll_target":"yaw_target";
    subscriptions_.push_back(nh.subscribe<std_msgs::Float64>(prefix_+"/"+name,1,
      [this,axis](const std_msgs::Float64ConstPtr& msg){mechanismTargetCallback(msg,axis);}));
  }
  subscriptions_.push_back(nh.subscribe(prefix_+"/target",1,
    &GimbalrotorRollYawPerchingNavigator::manualCallback,this));
  subscriptions_.push_back(nh.subscribe(prefix_+"/pitch_rezero",1,
    &GimbalrotorRollYawPerchingNavigator::pitchRezeroCallback,this));
  if(qp_planning_&&!qp_configuration_valid_) ROS_WARN("[RollYaw] Invalid QP configuration; explicit planning unavailable, FK remains usable.");
  if(!configuration_valid_) ROS_ERROR("[RollYaw] Lock disabled: require valid hinge geometry, limits and ordinary fully actuated controller.");
  diagnostics();
}
bool GimbalrotorRollYawPerchingNavigator::snapshotModel(Kinematics& out,std::string& error)
{
  if(!configuration_valid_||!robot_model_||!robot_model_->initialized()) { error="model/configuration unavailable or invalid"; return false; }
  const auto indices=robot_model_->getJointIndexMap();
  if(indices.count(roll_)!=1||indices.count(yaw_)!=1) { error="missing roll/yaw model joints"; return false; }
  const unsigned ri=indices.at(roll_),yi=indices.at(yaw_);
  auto joints=robot_model_->getJointPositions();
  if(ri>=joints.rows()||yi>=joints.rows()||!joints.data.allFinite())
  { error="invalid model joint positions/indices"; return false; }
  for(int i=1;i<3;++i) {
    const auto joint=robot_model_->getUrdfModel().getJoint(i==1?roll_:yaw_);
    if(!joint||joint->type!=urdf::Joint::REVOLUTE||joint->mimic||!joint->limits||
       !std::isfinite(joint->limits->lower)||!std::isfinite(joint->limits->upper)||
       !std::isfinite(joint->limits->velocity)||joint->limits->velocity<=0)
    { error="expected bounded URDF roll/yaw revolutes"; return false; }
    Vec axis(joint->axis.x,joint->axis.y,joint->axis.z);
    if(!axis.allFinite()||axis.norm()<1e-9||std::abs(axis.normalized().dot(i==1?Vec::UnitX():Vec::UnitZ()))<.99)
    { error="unexpected URDF roll/yaw axes"; return false; }
    planning_config_.limits.lower[i]=std::max(planning_config_.limits.lower[i],joint->limits->lower);
    planning_config_.limits.upper[i]=std::min(planning_config_.limits.upper[i],joint->limits->upper);
    planning_config_.limits.velocity[i]=std::min(planning_config_.limits.velocity[i],joint->limits->velocity);
  }
  if(!planning_config_.mechanismValid()) { error="empty URDF/configured limit intersection"; return false; }
  const auto tree=robot_model_->getTree(); const auto inertia=robot_model_->getInertiaMap();
  const auto extra=robot_model_->getExtraModuleMap(); const auto base=robot_model_->getBaselinkName();
  const auto relative=robot_model_->getCogDesireOrientation<KDL::Rotation>();
  KDL::Chain chain; if(!tree.getChain(base,contact_,chain))
  { error="missing baselink-to-contact chain"; return false; }
  // Snapshot all model data before launching the planning worker. No mutation of
  // the live robot model and no ROS access from the worker.
  auto cog=articulatedCogModel(tree,joints,inertia,extra,base,relative,ri,yi);
  return out.configure(chain,roll_,yaw_,cp_,axis_,cog,error);
}
Feedback GimbalrotorRollYawPerchingNavigator::feedback() const
{
  Feedback s=measured_; s.now=ros::Time::now().toSec(); s.hover=navi_state_==HOVER_STATE&&!force_landing_flag_;
  s.invalid_reason.clear();
  if(!joint_valid_) s.invalid_reason=joint_failure_;
  else if(!body_valid_) s.invalid_reason=body_failure_;
  else if(std::abs(s.joint_stamp-s.body_stamp)>synchronization_tolerance_)
    s.invalid_reason="joint/body feedback timestamp mismatch";
  if(estimator_) {
    for(auto axis:{State::X_BASE,State::Y_BASE,State::Z_BASE})
      if(!estimator_->getStateStatus(axis,estimate_mode_)&&s.invalid_reason.empty())
        s.invalid_reason="baselink position estimate unavailable";
  } else s.invalid_reason="estimator unavailable";
  s.valid=s.invalid_reason.empty();
  return s;
}
void GimbalrotorRollYawPerchingNavigator::jointCallback(const sensor_msgs::JointStateConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  // Other publishers may send gimbal-only messages on this shared topic.
  if(std::find(m->name.begin(),m->name.end(),roll_)==m->name.end()&&
     std::find(m->name.begin(),m->name.end(),yaw_)==m->name.end()) return;
  Vec q=measured_.eta,rate=Vec::Zero(); const double now=ros::Time::now().toSec(),stamp=m->header.stamp.toSec();
  // Match multilink/pitch-only feedback handling: an old publication must not
  // invalidate the last accepted sample or refresh its measurement timestamp.
  // feedbackValid() still expires that sample if new measurements stop arriving.
  if(stamp>0&&stamp<=measured_.joint_stamp) { ++joint_ignored_stamps_; return; }
  joint_valid_=false; joint_failure_="invalid/stale/future joint timestamp";
  if(stamp<=0||now-stamp>feedback_config_.timeout||now-stamp<-feedback_config_.future_tolerance) return;
  joint_failure_="missing/duplicate joint name or invalid position";
  for(int i=1;i<3;++i) {
    const auto& name=i==1?roll_:yaw_;
    if(std::count(m->name.begin(),m->name.end(),name)!=1) return;
    const auto index=std::distance(m->name.begin(),std::find(m->name.begin(),m->name.end(),name));
    if(static_cast<std::size_t>(index)>=m->position.size()||!std::isfinite(m->position[index])) return;
    q[i]=m->position[index];
    if(static_cast<std::size_t>(index)<m->velocity.size()) rate[i]=m->velocity[index];
    else if(measured_.joint_stamp>0&&stamp-measured_.joint_stamp>1e-4&&stamp-measured_.joint_stamp<=feedback_config_.timeout)
      rate[i]=(q[i]-measured_.eta[i])/(stamp-measured_.joint_stamp);
    else rate[i]=std::numeric_limits<double>::quiet_NaN();
  }
  measured_.eta=q; measured_.joint_rate=rate; measured_.joint_stamp=stamp; joint_valid_=rate.allFinite();
  joint_failure_=joint_valid_?"":"joint angular rates unavailable/nonfinite (need velocity or two fresh samples)";
}
void GimbalrotorRollYawPerchingNavigator::bodyCallback(const nav_msgs::OdometryConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double stamp=m->header.stamp.toSec(),now=ros::Time::now().toSec();
  // The estimator publishes on a timer using the latest IMU measurement stamp;
  // consecutive odometry publications can legitimately share that stamp.
  // Keep the accepted sample, never replace its stamp with receipt time.
  if(stamp>0&&stamp<=measured_.body_stamp) { ++body_ignored_stamps_; return; }
  body_valid_=false; body_failure_="invalid/stale/future body timestamp";
  if(stamp<=0||now-stamp>feedback_config_.timeout||now-stamp<-feedback_config_.future_tolerance) return;
  body_failure_="body odometry world/child frame mismatch";
  if(frameName(m->header.frame_id)!=frameName(world_)||frameName(m->child_frame_id)!=frameName(body_frame_)) return;
  body_failure_="invalid body quaternion";
  const auto& p=m->pose.pose; Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if(!q.coeffs().allFinite()||std::abs(q.norm()-1)>1e-3) return;
  measured_.body=Pose::Identity(); measured_.body.linear()=q.normalized().toRotationMatrix();
  measured_.body.translation()=Vec(p.position.x,p.position.y,p.position.z);
  const auto& w=m->twist.twist.angular; measured_.body_omega=Vec(w.x,w.y,w.z);
  measured_.body_stamp=stamp; body_valid_=poseValid(measured_.body)&&measured_.body_omega.allFinite();
  body_failure_=body_valid_?"":"invalid body position/angular rate";
}
bool GimbalrotorRollYawPerchingNavigator::ownsNavigationTargets() const
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  return perchingSession().mode!=perching_geometry::Mode::DISABLED;
}
void GimbalrotorRollYawPerchingNavigator::navigationTargetConflict(const char* source)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  reason_=std::string("unsupported free-flight target while perched: ")+source;
  ROS_WARN_STREAM_THROTTLE(1.0,"[RollYaw] "<<reason_);
}
void GimbalrotorRollYawPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!ownsNavigationTargets()) { GimbalrotorNavigator::naviCallback(m); return; }
  if(m->pos_xy_nav_mode==aerial_robot_msgs::FlightNav::NO_NAVIGATION&&
     m->pos_z_nav_mode==aerial_robot_msgs::FlightNav::NO_NAVIGATION&&
     m->roll_nav_mode==aerial_robot_msgs::FlightNav::NO_NAVIGATION&&
     m->yaw_nav_mode==aerial_robot_msgs::FlightNav::NO_NAVIGATION&&
     m->pitch_nav_mode==aerial_robot_msgs::FlightNav::POS_MODE)
    applyManualPitchDelta(m->target_pitch);
  else navigationTargetConflict("uav/nav: only lock-relative pitch is supported; body planning requires qp_planning=true and explicit prepare");
}
void GimbalrotorRollYawPerchingNavigator::applyManualPitchDelta(double alpha)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  Vec target=nominal_; target[0]=alpha; setNominalTarget(target,0);
}
bool GimbalrotorRollYawPerchingNavigator::tryLockPerching(const std::string& cause)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto f=feedback();
  std::string error=roll_yaw::feedbackRejection(f,feedback_config_,planning_config_);
  Kinematics model; Reference held; Vec q=f.eta; q[0]=0;
  if(perchingSession().mode!=perching_geometry::Mode::NORMAL||!error.empty()||
     !snapshotModel(model,error)||!model.lock(f.body,q)||
     !mechanismAllowed(model,q,planning_config_,error)||
     !mechanismReference(model,q,Vec::Zero(),Vec::Zero(),held)) {
    reason_="contact capture failed: "+(error.empty()?"invalid HOVER/model/FK snapshot":error);
    if(!previous_failure_.empty()) reason_+="; previous failure: "+previous_failure_;
    ROS_WARN_STREAM("[RollYaw] "<<reason_); diagnostics(); return false;
  }
  invalidatePlanner(); resetPitchRezeroState();
  model_=std::move(model); nominal_=q; final_target_=held;
  lock_valid_=true; faulted_=false; previous_failure_.clear(); neutral_=held.body.linear();
  trajectory_mode_=false; traj_generator_ptr_.reset();
  gps_waypoint_=vel_based_waypoint_=force_att_control_flag_=false;
  commitFixedContactLock(tfVector(held.cog.translation()),tfQuaternion(held.cog.linear()),
                         tfVector(model_.pivotLock().translation()));
  lock_stamp_=perchingSession().lock_stamp;
  reason_="measured contact captured by "+cause+"; independent nominal targets initialized";
  diagnostics(); return true;
}
void GimbalrotorRollYawPerchingNavigator::resetPerchingLock()
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(faulted_||pitch_rezero_failed_) {
    if(previous_failure_.empty()) previous_failure_=reason_;
    ROS_WARN_STREAM("[RollYaw] Resetting reference after: "<<previous_failure_);
  }
  GimbalrotorPerchingNavigator::resetPerchingLock();
  invalidatePlanner(); resetPitchRezeroState();
  lock_valid_=false; faulted_=false; lock_stamp_=ros::Time();
  nominal_.setZero(); // Never publish these reset values; capture must succeed first.
  manual_receipt_stamp_=ros::Time();
  // Base enable performs another reset on a rejected capture. Keep its cause.
  if(reason_.find("contact capture failed:")!=0) {
    if(previous_failure_.empty()) reason_="perching reference reset";
    else reason_="perching reference reset; previous failure: "+previous_failure_;
  }
}
void GimbalrotorRollYawPerchingNavigator::invalidatePlanner()
{
  ++producer_generation_; planner_.reset(); // Late workers hold only immutable model snapshots.
}
void GimbalrotorRollYawPerchingNavigator::stopMotion(const std::string& why)
{
  invalidatePlanner(); nominal_=final_target_.eta;
  Reference held;
  if(lock_valid_&&mechanismReference(model_,nominal_,Vec::Zero(),Vec::Zero(),held)) final_target_=held;
  reason_=why;
}
void GimbalrotorRollYawPerchingNavigator::faultHold(const std::string& why)
{
  if(!faulted_) { stopMotion(why); previous_failure_=why; }
  faulted_=true;
  if(pitch_rezero_state_!=PitchRezeroState::IDLE&&!pitch_rezero_failed_) failPitchRezero(why);
}
std::string GimbalrotorRollYawPerchingNavigator::contactRejection(const Feedback& f) const
{
  auto why=roll_yaw::feedbackRejection(f,feedback_config_,planning_config_);
  if(!why.empty()) return why;
  Estimate contact;
  if(!model_.estimate(f.body,f.eta,contact)) return "invalid passive hinge estimate";
  Vec q=f.eta; q[0]=contact.alpha;
  if((q.array()<planning_config_.limits.lower.array()).any()||
     (q.array()>planning_config_.limits.upper.array()).any()||
     f.body.linear()(2,2)<std::cos(planning_config_.max_tilt)) return "measured mechanism/body outside operating limits";
  if(contact.pivot_error>feedback_config_.pivot_tolerance||contact.hinge_residual>feedback_config_.hinge_tolerance)
    return "physical pivot moved or hinge residual exceeded";
  return "";
}
void GimbalrotorRollYawPerchingNavigator::mechanismTargetCallback(const std_msgs::Float64ConstPtr& msg,int axis)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  Vec target=nominal_; target[axis]=msg->data; // Untouched destinations survive axis changes.
  setNominalTarget(target,axis);
}
void GimbalrotorRollYawPerchingNavigator::manualCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!lock_valid_||msg->header.frame_id!="perching_lock_"+std::to_string(lock_stamp_.toNSec())) {
    reason_="manual rejected: stale/missing base lock identity"; diagnostics(); return;
  }
  // Compatibility interface for deliberate full nominal targets. Independent
  // axis topics below are preferred; publisher timestamps are informational.
  setNominalTarget(Vec(msg->vector.y,msg->vector.x,msg->vector.z));
}
bool GimbalrotorRollYawPerchingNavigator::setNominalTarget(const Vec& target,int changed_axis)
{
  // Correction details on the existing status channel let the keyboard reconcile
  // only the matching request/axis, without waiting or introducing a handshake.
  auto correction=[&]() {
    if(changed_axis<0||changed_axis>2||!lock_valid_) return;
    const char* axis=changed_axis==0?"alpha":changed_axis==1?"roll":"yaw";
    std::ostringstream detail;
    detail<<std::setprecision(17)<<" [axis="<<axis<<" lock="<<lock_stamp_.toNSec()
          <<" requested="<<target[changed_axis]<<" accepted="<<nominal_[changed_axis]<<"]";
    reason_+=detail.str();
  };
  auto reject=[&](const std::string& why) { reason_="manual rejected: "+why; correction(); diagnostics(); return false; };
  if(!lock_valid_||faulted_||!feedback().hover) return reject("require valid enabled HOVER contact");
  if(pitch_rezero_state_!=PitchRezeroState::IDLE) return reject("pitch rezero ACTIVE/HOLD; wait ready then final relock");
  if(!target.allFinite()) return reject("nonfinite mechanism target");
  const auto why=contactRejection(feedback());
  if(!why.empty()) return reject(why);
  const Vec bounded=target.cwiseMax(planning_config_.limits.lower).cwiseMin(planning_config_.limits.upper);
  std::string error;
  if(!mechanismAllowed(model_,bounded,planning_config_,error)) return reject(error);
  // Multilink semantics: accept a destination, independent of its current applied
  // or measured value. No lookahead, endpoint-tracking gate or per-key executor.
  invalidatePlanner(); nominal_=bounded; manual_receipt_stamp_=ros::Time::now();
  reason_=(bounded-target).norm()>1e-10?"manual nominal target accepted at configured limit":"manual nominal target accepted";
  if((bounded-target).norm()>1e-10) correction();
  diagnostics(); return true;
}
bool GimbalrotorRollYawPerchingNavigator::buildFinalMechanismTarget(Reference& next)
{
  const Vec bounded=nominal_.cwiseMax(planning_config_.limits.lower).cwiseMin(planning_config_.limits.upper);
  std::string error;
  if(!mechanismAllowed(model_,bounded,planning_config_,error)) { reason_="target FK rejected: "+error; return false; }
  Vec rate=Vec::Zero(),acc=Vec::Zero();
  if(planner_) { rate=planner_->target.velocity; acc=planner_->target.acceleration; }
  return mechanismReference(model_,bounded,rate,acc,next);
}
void GimbalrotorRollYawPerchingNavigator::request(const std::string& action,std::uint64_t session)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!lock_valid_||session!=lock_stamp_.toNSec()) { reason_="stale/missing base lock identity"; return; }
  if(action=="cancel") {
    if(pitch_rezero_state_==PitchRezeroState::ACTIVE) failPitchRezero("operator stopped rezero");
    else if(!faulted_) stopMotion("stopped at applied contact reference");
    diagnostics(); return;
  }
  if(!qp_planning_) { reason_="body-orientation request requires qp_planning=true"; diagnostics(); return; }
  if(!qp_configuration_valid_) { reason_="invalid QP-only configuration"; return; }
  if(faulted_||pitch_rezero_state_!=PitchRezeroState::IDLE) { reason_="QP request rejected during fault/setup"; return; }
  const auto rejection=contactRejection(feedback());
  if(!rejection.empty()) { reason_=rejection; return; }
  if(action=="prepare") {
    if(pending_.valid()) { reason_="previous QP worker still active; manual targets remain available"; return; }
    ros::NodeHandle pn(nh_,"navigation/roll_yaw_perching/request"); Request request;
    pn.param("align_only",request.align_only,false);
    const bool valid=pn.getParam("body_yaw",request.yaw)&&pn.getParam("body_roll",request.roll)&&
      pn.getParam("pitch_start",request.pitch_start)&&pn.getParam("pitch_end",request.pitch_end)&&
      pn.getParam("pitch_rate",request.rate)&&pn.getParam("pitch_acceleration",request.acceleration);
    if(!valid||!request.valid()) { reason_="invalid/incomplete body-orientation request"; return; }
    stopMotion("explicit QP preparation requested");
    planner_.reset(new Executor()); planner_->qp_planning=true;
    planner_->config=feedback_config_; planner_->planning=planning_config_;
    if(!planner_->attach(model_,final_target_,session,ros::Time::now().toSec())||!planner_->command(action,session)) {
      reason_=planner_->reason; invalidatePlanner(); return;
    }
    const auto model=model_; const auto initial=final_target_.eta; const auto neutral=neutral_; const auto config=planning_config_;
    pending_generation_=producer_generation_;
    pending_=std::async(std::launch::async,[model,initial,neutral,request,config,session]() {
      auto plan=prepare(model,initial,neutral,request,config); plan.session=session; return plan;
    });
  } else if(!planner_) { reason_="no explicit QP plan; prepare first"; return; }
  else planner_->command(action,session);
  reason_=planner_->reason; diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::resetPitchRezeroState()
{
  pitch_rezero_state_=PitchRezeroState::IDLE; pitch_rezero_ready_=pitch_rezero_failed_=false;
  rezero_frozen_.setZero(); rezero_started_=rezero_stable_since_=rezero_alpha_=0;
  rezero_previous_pitch_=rezero_previous_time_=rezero_measured_rate_=rezero_feedback_dt_=0;
  rezero_rate_valid_=false;
}
void GimbalrotorRollYawPerchingNavigator::pitchRezeroCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto f=feedback();
  if(!lock_valid_||faulted_||!f.hover||!rezero_configuration_valid_||pitch_rezero_state_!=PitchRezeroState::IDLE) {
    reason_="pitch rezero rejected: require valid enabled HOVER contact, configuration and IDLE"; diagnostics(); return;
  }
  const auto why=contactRejection(f);
  if(!why.empty()) { reason_="pitch rezero rejected: "+why; diagnostics(); return; }
  Kinematics model=model_; Vec frozen=f.eta; frozen[0]=0; Reference held;
  if(!model.lock(f.body,frozen)||!mechanismReference(model,frozen,Vec::Zero(),Vec::Zero(),held)) {
    reason_="pitch rezero rejected: invalid measured body/joint capture"; diagnostics(); return;
  }
  invalidatePlanner(); resetPitchRezeroState();
  model_=std::move(model); final_target_=held; nominal_=rezero_frozen_=frozen;
  rezero_started_=rezero_previous_time_=f.now;
  double roll,yaw; tf::Matrix3x3(tfQuaternion(f.body.linear())).getRPY(roll,rezero_previous_pitch_,yaw);
  pitch_rezero_state_=PitchRezeroState::ACTIVE;
  reason_="pitch rezero ACTIVE: signed measured-feedback control, active roll/yaw frozen";
  diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::updateBodyPitchFeedback()
{
  rezero_rate_valid_=false; rezero_feedback_dt_=0;
  const auto f=feedback();
  if(!roll_yaw::feedbackRejection(f,feedback_config_,planning_config_).empty()) {
    rezero_previous_time_=0; return;
  }
  double roll,pitch,yaw; tf::Matrix3x3(tfQuaternion(f.body.linear())).getRPY(roll,pitch,yaw);
  if(rezero_previous_time_>0) {
    rezero_feedback_dt_=f.now-rezero_previous_time_;
    if(rezero_feedback_dt_>0&&rezero_feedback_dt_<=.1) {
      rezero_measured_rate_=std::remainder(pitch-rezero_previous_pitch_,2*M_PI)/rezero_feedback_dt_;
      rezero_rate_valid_=std::isfinite(rezero_measured_rate_);
    }
  }
  rezero_previous_pitch_=pitch; rezero_previous_time_=f.now;
}
void GimbalrotorRollYawPerchingNavigator::failPitchRezero(const std::string& why)
{
  invalidatePlanner(); pitch_rezero_state_=PitchRezeroState::HOLD;
  pitch_rezero_ready_=false; pitch_rezero_failed_=true; rezero_stable_since_=0;
  reason_="pitch rezero failed: "+why+"; holding last valid pose and frozen joints";
  previous_failure_=reason_;
  ROS_WARN_STREAM("[RollYaw] "<<reason_);
}
bool GimbalrotorRollYawPerchingNavigator::verifyPitchRezeroFrozenJoints()
{
  const auto f=feedback();
  const auto why=roll_yaw::feedbackRejection(f,feedback_config_,planning_config_);
  if(!why.empty()) { if(!pitch_rezero_failed_) failPitchRezero(why); return false; }
  if(std::abs(std::remainder(f.eta[1]-rezero_frozen_[1],2*M_PI))>rezero_roll_tolerance_||
     std::abs(std::remainder(f.eta[2]-rezero_frozen_[2],2*M_PI))>rezero_yaw_tolerance_) {
    if(!pitch_rezero_failed_) failPitchRezero("active roll/yaw moved outside frozen-joint tolerance");
    return false;
  }
  return true;
}
void GimbalrotorRollYawPerchingNavigator::updatePitchRezeroCommand()
{
  const auto f=feedback(); const double elapsed=f.now-rezero_started_;
  if(!std::isfinite(elapsed)||elapsed<0||rezero_feedback_dt_<0||rezero_feedback_dt_>.5)
    return failPitchRezero("invalid or discontinuous time");
  if(elapsed>rezero_timeout_) return failPitchRezero("timeout before measured stabilization");
  double roll,pitch,yaw; tf::Matrix3x3(tfQuaternion(f.body.linear())).getRPY(roll,pitch,yaw);
  const double error=std::remainder(rezero_pitch_-pitch,2*M_PI);
  if(rezero_rate_valid_&&std::abs(error)<=rezero_tolerance_&&std::abs(rezero_measured_rate_)<=rezero_rate_tolerance_) {
    if(rezero_stable_since_==0) rezero_stable_since_=f.now;
    if(f.now-rezero_stable_since_>=rezero_stable_duration_) {
      pitch_rezero_state_=PitchRezeroState::HOLD; pitch_rezero_ready_=true; pitch_rezero_failed_=false;
      reason_="pitch rezero ready-HOLD: measured pitch/rate stable; request final relock"; return;
    }
  } else rezero_stable_since_=0;
  if(!std::isfinite(rezero_feedback_dt_)||rezero_feedback_dt_<=0) return;
  // Exact multilink signed error/rate law. Alpha rotates about R_WP * axis_,
  // including the URDF's approximately pi mounting yaw; never world Euler Y.
  const double rate=rezero_sign_*std::clamp(rezero_kp_*error,-rezero_rate_,rezero_rate_);
  const double alpha=std::clamp(rezero_alpha_+rate*std::min(rezero_feedback_dt_,.1),-rezero_travel_,rezero_travel_);
  Vec q=rezero_frozen_; q[0]=alpha;
  Reference next; std::string why;
  if(!mechanismAllowed(model_,q,planning_config_,why)||!mechanismReference(model_,q,Vec::Zero(),Vec::Zero(),next))
    return failPitchRezero("invalid/limited passive hinge target: "+why);
  // Reject a sign/geometry combination that drives measured pitch away from the
  // goal. This is a local forward response check, not an endpoint IK/controller.
  Vec lo=rezero_frozen_,hi=rezero_frozen_; Estimate estimate;
  if(!model_.estimate(f.body,f.eta,estimate)) return failPitchRezero("invalid hinge estimate");
  lo[0]=estimate.alpha-1e-5; hi[0]=estimate.alpha+1e-5;
  Pose b0,g0,b1,g1;
  if(!model_.evaluate(lo,b0,g0)||!model_.evaluate(hi,b1,g1)) return failPitchRezero("invalid hinge response FK");
  const double slope=(std::asin(std::clamp(-b1.linear()(2,0),-1.0,1.0))-
                      std::asin(std::clamp(-b0.linear()(2,0),-1.0,1.0)))/2e-5;
  if(std::abs(error)>rezero_tolerance_&&rezero_sign_*slope<=1e-6)
    return failPitchRezero("configured rezero sign does not reduce pitch error about this physical hinge");
  final_target_=next; rezero_alpha_=alpha;
}
void GimbalrotorRollYawPerchingNavigator::applyPitchRezeroTarget()
{
  const bool frozen_valid=verifyPitchRezeroFrozenJoints();
  if(pitch_rezero_state_==PitchRezeroState::ACTIVE&&frozen_valid) updatePitchRezeroCommand();
  nominal_=final_target_.eta;
  nominal_[1]=rezero_frozen_[1]; nominal_[2]=rezero_frozen_[2];
}
void GimbalrotorRollYawPerchingNavigator::commandDesiredBaselinkPose(const Reference& r)
{
  const Mat relative=r.cog.linear().transpose()*r.body.linear();
  setSynchronizedBaselinkRotationRelativeToCog(tfQuaternion(relative));
  double roll,pitch,yaw; tf::Matrix3x3(tfQuaternion(r.cog.linear())).getRPY(roll,pitch,yaw);
  setXyControlMode(POS_CONTROL_MODE); setTargetPos(tfVector(r.cog.translation()));
  setTargetVel(tfVector(r.cog_velocity)); setTargetAcc(tfVector(r.cog_acceleration));
  setTargetRPY(tf::Vector3(roll,pitch,yaw));
  setTargetOmega(tfVector(r.cog_omega)); setTargetAngAcc(tfVector(r.cog_angular_acceleration));
}
void GimbalrotorRollYawPerchingNavigator::publishJointTarget()
{
  sensor_msgs::JointState cmd; cmd.header.stamp=ros::Time::now();
  cmd.name={roll_,yaw_}; cmd.position={final_target_.eta[1],final_target_.eta[2]};
  joints_pub_.publish(cmd); // Passive alpha is never an actuator.
}
void GimbalrotorRollYawPerchingNavigator::applyActivePerchingTarget()
{
  target_applied_this_update_=true;
  if(!lock_valid_||!feedback().hover) return;
  const auto f=feedback();
  const auto why=contactRejection(f);
  if(!why.empty()) faultHold(why);
  // Preserve the articulated-model drift guard outside the two active joints.
  if(robot_model_&&robot_model_->initialized()) {
    const auto frames=robot_model_->getSegmentsTf(); const auto b=frames.find(robot_model_->getBaselinkName());
    Pose body,cog;
    if(b!=frames.end()&&model_.evaluate(f.eta,body,cog)) {
      const Vec actual=fromKdl(b->second.Inverse()*robot_model_->getCog<KDL::Frame>()).translation();
      if((actual-(body.inverse()*cog).translation()).norm()>cog_model_tolerance_)
        faultHold("articulated CoG changed beyond snapshot tolerance; recapture required");
    }
  }
  if(!faulted_) {
    if(pitch_rezero_state_!=PitchRezeroState::IDLE) applyPitchRezeroTarget();
    else {
      if(planner_) {
        planner_->tick(f);
        if(planner_->stage==Stage::FAULT_HOLD) faultHold("QP fault: "+planner_->first_fault);
        else { nominal_=planner_->target.eta; reason_=planner_->reason; }
      }
      Reference next;
      if(!faulted_&&buildFinalMechanismTarget(next)) final_target_=next;
    }
  }
  if(!poseValid(final_target_.body)||!poseValid(final_target_.cog)) { faultHold("invalid controller reference"); return; }
  commandDesiredBaselinkPose(final_target_); publishJointTarget();
}
void GimbalrotorRollYawPerchingNavigator::update()
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(force_landing_flag_&&ownsNavigationTargets()) selectPerchingMode(perching_geometry::Mode::NORMAL,false);
  if(lock_valid_&&perchingSession().lock_stamp!=lock_stamp_) faultHold("base lock identity replaced");
  updateBodyPitchFeedback(); target_applied_this_update_=false;
  GimbalrotorPerchingNavigator::update();
  // Same explicit-setup hold policy as multilink when optional base hold is off.
  if(lock_valid_&&pitch_rezero_state_!=PitchRezeroState::IDLE&&!target_applied_this_update_) applyActivePerchingTarget();
  if(pending_.valid()&&pending_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
    try {
      auto plan=pending_.get();
      if(planner_&&pending_generation_==producer_generation_&&lock_valid_&&!faulted_) {
        planner_->accept(std::move(plan)); reason_=planner_->reason;
      }
    } catch(const std::exception& e) {
      if(planner_&&pending_generation_==producer_generation_) faultHold(std::string("QP planning exception: ")+e.what());
    }
  }
  diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::diagnostics()
{
  const auto f=feedback();
  std_msgs::UInt64 session; session.data=lock_valid_?lock_stamp_.toNSec():0; session_pub_.publish(session);
  std_msgs::Bool flag;
  flag.data=pitch_rezero_state_==PitchRezeroState::ACTIVE; rezero_active_pub_.publish(flag);
  flag.data=pitch_rezero_ready_; rezero_ready_pub_.publish(flag);
  flag.data=pitch_rezero_failed_; rezero_failed_pub_.publish(flag);
  const char* state=!ownsNavigationTargets()?"FREE_FLIGHT":!lock_valid_?"LOCK_FAILED":
    pitch_rezero_failed_?"PITCH_REZERO_FAILED":faulted_?"FAULT_HOLD":
    pitch_rezero_state_==PitchRezeroState::ACTIVE?"PITCH_REZERO_ACTIVE":
    pitch_rezero_state_==PitchRezeroState::HOLD?"PITCH_REZERO_HOLD":planner_?stageName(planner_->stage):"CONTACT_LOCKED";
  std_msgs::String status; status.data=std::string(state)+": "+reason_;
  if((faulted_||pitch_rezero_failed_)&&!previous_failure_.empty()&&previous_failure_!=reason_)
    status.data+="; hold cause: "+previous_failure_;
  state_pub_.publish(status);
  if(lock_valid_) {
    geometry_msgs::Vector3Stamped angles;
    angles.header.stamp=ros::Time::now(); angles.header.frame_id="perching_lock_"+std::to_string(lock_stamp_.toNSec());
    // Effective configured/URDF intersection from snapshotModel(). Wire order
    // matches nominal_angles [roll, alpha, yaw], not internal [alpha, roll, yaw].
    const auto& limits=planning_config_.limits;
    angles.vector.x=limits.lower[1]; angles.vector.y=limits.lower[0]; angles.vector.z=limits.lower[2];
    position_lower_pub_.publish(angles);
    angles.vector.x=limits.upper[1]; angles.vector.y=limits.upper[0]; angles.vector.z=limits.upper[2];
    position_upper_pub_.publish(angles);
    angles.vector.x=nominal_[1]; angles.vector.y=nominal_[0]; angles.vector.z=nominal_[2]; nominal_pub_.publish(angles);
    angles.vector.x=final_target_.eta[1]; angles.vector.y=final_target_.eta[0]; angles.vector.z=final_target_.eta[2]; angles_pub_.publish(angles);
    std_msgs::Float64 pitch; pitch.data=nominal_[0]; pitch_pub_.publish(pitch);
    nav_msgs::Odometry body; body.header.stamp=ros::Time::now(); body.header.frame_id=world_; body.child_frame_id=body_frame_;
    body.pose.pose=poseMsg(final_target_.body);
    tf::vector3TFToMsg(tfVector(final_target_.body_omega),body.twist.twist.angular); reference_pub_.publish(body);
    body.header.stamp.fromSec(f.body_stamp); body.pose.pose=poseMsg(f.body);
    tf::vector3TFToMsg(tfVector(f.body_omega),body.twist.twist.angular); measured_pub_.publish(body);
    body.header.stamp=ros::Time::now(); body.child_frame_id="desired_cog"; body.pose.pose=poseMsg(final_target_.cog);
    tf::vector3TFToMsg(tfVector(final_target_.cog_omega),body.twist.twist.angular);
    tf::vector3TFToMsg(tfVector(final_target_.cog.linear().transpose()*final_target_.cog_velocity),body.twist.twist.linear);
    cog_pub_.publish(body);
    model_.estimate(f.body,f.eta,measured_contact_);
  }
  std::ostringstream out; out<<std::setprecision(12)
    <<"eta=[passive alpha, active roll, active yaw]; units=rad,m,s; state="<<state
    <<" lock="<<session.data<<" reason=["<<reason_<<"] qp_planning="<<qp_planning_
    <<" nominal="<<nominal_.transpose()<<" applied="<<final_target_.eta.transpose()
    <<" measured_active="<<f.eta.tail<2>().transpose()<<" measured_alpha="<<measured_contact_.alpha
    <<" joint_age="<<f.now-f.joint_stamp<<" body_age="<<f.now-f.body_stamp
    <<" joint_failure=["<<joint_failure_<<"] body_failure=["<<body_failure_<<"]"
    <<" pivot_error="<<measured_contact_.pivot_error<<" hinge_residual="<<measured_contact_.hinge_residual
    <<" rezero_alpha="<<rezero_alpha_<<" rezero_pitch="<<rezero_previous_pitch_
    <<" rezero_pitch_rate="<<rezero_measured_rate_<<" rezero_rate_valid="<<rezero_rate_valid_
    <<" rezero_sign="<<rezero_sign_<<" manual_receipt="<<manual_receipt_stamp_.toSec()
    <<" ignored_body_stamps="<<body_ignored_stamps_<<" ignored_joint_stamps="<<joint_ignored_stamps_;
  std_msgs::String detail; detail.data=out.str(); details_pub_.publish(detail);
}
}
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorRollYawPerchingNavigator,aerial_robot_navigation::BaseNavigator)
