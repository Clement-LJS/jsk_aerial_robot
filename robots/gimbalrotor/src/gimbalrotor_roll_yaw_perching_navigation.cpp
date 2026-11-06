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
  pn.param("prepare_deadline",prepare_deadline_,prepare_deadline_);
  pn.param("max_execution_step",c.max_execution_step,c.max_execution_step);
  pn.param("soft_tracking_fraction",c.soft_tracking_fraction,c.soft_tracking_fraction);
  pn.param("resume_fraction",c.resume_fraction,c.resume_fraction);
  pn.param("preparation_tolerance",c.preparation_tolerance,c.preparation_tolerance);
  pn.param("preparation_quiet_time",c.preparation_quiet_time,c.preparation_quiet_time);
  pn.param("reference_lead",c.reference_lead,c.reference_lead);
  pn.param("progress_filter_time",c.progress_filter_time,c.progress_filter_time);
  pn.param("goal_tolerance",c.goal_tolerance,c.goal_tolerance);
  pn.param("progress_timeout",c.progress_timeout,c.progress_timeout);
  ros::NodeHandle an(pn,"admittance"); auto& a=admittance_.config;
  an.param("inertia",a.inertia,a.inertia); an.param("damping",a.damping,a.damping);
  an.param("stiffness",a.stiffness,a.stiffness); an.param("torque_limit",a.torque_limit,a.torque_limit);
  an.param("angle_limit",a.angle_limit,a.angle_limit); an.param("rate_limit",a.rate_limit,a.rate_limit);
  an.param("acceleration_limit",a.acceleration_limit,a.acceleration_limit);
  an.param("reference_torque",a.reference_torque,a.reference_torque);
  an.param("filter_time",a.filter_time,a.filter_time);
  an.param("on_threshold",a.on_threshold,a.on_threshold); an.param("off_threshold",a.off_threshold,a.off_threshold);
  an.param("on_dwell",a.on_dwell,a.on_dwell); an.param("off_dwell",a.off_dwell,a.off_dwell);
  an.param("baseline_samples",a.baseline_samples,a.baseline_samples);
  an.param("observer_gain",observer_.gain,observer_.gain);
  an.param<std::string>("external_wrench_topic",interaction_topic_,"estimated_external_wrench");
  an.param<std::string>("wrench_frame",interaction_frame_,"world");
  an.param<std::string>("wrench_point",interaction_point_,"cog");
  an.param("wrench_sign",interaction_sign_,interaction_topic_=="estimated_external_wrench"?1.0:0.0);
  an.param("wrench_scale",interaction_scale_,interaction_topic_=="estimated_external_wrench"?1.0:0.0);
  interaction_configured_=a.valid()&&
    (interaction_sign_==1||interaction_sign_==-1)&&std::isfinite(interaction_scale_)&&interaction_scale_>0&&
    (interaction_frame_=="world"||interaction_frame_=="body")&&
    (interaction_point_=="cog"||interaction_point_=="body"||interaction_point_=="world_origin");
  const bool vectors=readVector(pn,"position_lower",p.limits.lower)&&readVector(pn,"position_upper",p.limits.upper);
  // Shared mechanism and trajectory bounds.
  const bool rate_vectors=readVector(pn,"velocity_caps",p.limits.velocity)&&readVector(pn,"acceleration_caps",p.limits.acceleration);
  pn.param("planning_dt",p.dt,p.dt); pn.param("body_max_tilt",p.max_tilt,p.max_tilt);
  pn.param("orientation_tolerance",p.orientation_tolerance,p.orientation_tolerance);
  pn.param("rate_tolerance",p.rate_tolerance,p.rate_tolerance);
  pn.param("alignment_rate",p.alignment_rate,p.alignment_rate);
  pn.param("alignment_acceleration",p.alignment_acceleration,p.alignment_acceleration);
  pn.param("max_phase_duration",p.max_duration,p.max_duration);
  pn.param("scaling_attempts",p.scaling_attempts,p.scaling_attempts);
  pn.param("cog_speed_limit",p.cog_speed_limit,p.cog_speed_limit);
  pn.param("cog_acceleration_limit",p.cog_acceleration_limit,p.cog_acceleration_limit);
  pn.param("min_singular_value",p.min_singular_value,p.min_singular_value);
  pn.param("feedback_timeout",c.timeout,c.timeout); pn.param("future_tolerance",c.future_tolerance,c.future_tolerance);
  pn.param("pivot_tolerance",c.pivot_tolerance,c.pivot_tolerance); pn.param("hinge_residual_tolerance",c.hinge_tolerance,c.hinge_tolerance);
  pn.param("tracking_tolerance",c.tracking_tolerance,c.tracking_tolerance);
  pn.param("body_tracking_tolerance",c.orientation_tolerance,c.orientation_tolerance);
  pn.param("settle_rate",c.settle_rate,c.settle_rate); pn.param("settle_time",c.settle_time,c.settle_time);
  pn.param("settle_timeout",c.settle_timeout,c.settle_timeout);
  pn.param("settle_use_window",c.settle_use_window,c.settle_use_window);
  pn.param("settle_body_rate",c.settle_body_rate,c.settle_body_rate);
  pn.param("settle_joint_rate",c.settle_joint_rate,c.settle_joint_rate);
  pn.param("settle_mean_rate",c.settle_mean_rate,c.settle_mean_rate);
  c.require_settled_on_enable=false; // Same measured-pose capture policy as multilink.
  pn.param("feedback_sync_tolerance",synchronization_tolerance_,synchronization_tolerance_);
  pn.param("cog_model_tolerance",cog_model_tolerance_,cog_model_tolerance_);
  planning_configuration_valid_=rate_vectors&&p.valid()&&c.governorValid(p)&&c.settlingValid()&&
    std::isfinite(prepare_deadline_)&&prepare_deadline_>0&&prepare_deadline_<=300;
  configuration_valid_=configuration_valid_&&poseValid(cp_)&&vectors&&rate_vectors&&p.limits.valid()&&p.mechanismValid()&&c.feedbackConfigValid()&&
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
  status_pub_=nh.advertise<diagnostic_msgs::DiagnosticArray>(prefix_+"/diagnostics_status",1);
  task_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/task_pitch",1);
  compliance_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/compliance",1);
  ack_pub_=nh.advertise<std_msgs::String>(prefix_+"/command_ack",1);
  subscriptions_.push_back(nh.subscribe("perching/admittance_enable",1,&GimbalrotorRollYawPerchingNavigator::admittanceCallback,this));
  subscriptions_.push_back(nh.subscribe("perching/roll_yaw/command_wrench_estimate",1,&GimbalrotorRollYawPerchingNavigator::wrenchCallback,this));
  subscriptions_.push_back(nh.subscribe(interaction_topic_,1,&GimbalrotorRollYawPerchingNavigator::externalWrenchCallback,this));
  subscriptions_.push_back(nh.subscribe(prefix_+"/admittance_tare",1,&GimbalrotorRollYawPerchingNavigator::tareCallback,this));
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
  for(const auto* key:{"prepare","start","pause","resume","stop","cancel"}) {
    std::string action(key);
    subscriptions_.push_back(nh.subscribe<std_msgs::UInt64>(prefix_+"/"+action,1,
      [this,action](const std_msgs::UInt64ConstPtr& m){
        request(action,m->data);
        std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
        std_msgs::String ack; ack.data="action="+action+" session="+std::to_string(m->data)+
          " accepted="+(last_command_accepted_?"true":"false")+" reason="+reason_; ack_pub_.publish(ack);
      }));
  }
  for(int axis=0;axis<3;++axis) {
    const std::string name=axis==0?"alpha_target":axis==1?"roll_target":"yaw_target";
    subscriptions_.push_back(nh.subscribe<std_msgs::Float64>(prefix_+"/"+name,1,
      [this,axis](const std_msgs::Float64ConstPtr& msg){mechanismTargetCallback(msg,axis);}));
  }
  for(int axis=0;axis<2;++axis) {
    subscriptions_.push_back(nh.subscribe<std_msgs::Float64>(prefix_+(axis==0?"/body_yaw_target":"/body_roll_target"),1,
      [this,axis](const std_msgs::Float64ConstPtr& msg){bodyAlignmentCallback(msg,axis);}));
  }
  subscriptions_.push_back(nh.subscribe(prefix_+"/target",1,
    &GimbalrotorRollYawPerchingNavigator::manualCallback,this));
  subscriptions_.push_back(nh.subscribe(prefix_+"/pitch_rezero",1,
    &GimbalrotorRollYawPerchingNavigator::pitchRezeroCallback,this));
  if(!planning_configuration_valid_) ROS_WARN("[RollYaw] Invalid offline configuration; manual navigation remains usable.");
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
  planning_config_.saw_point_valid=false;
  KDL::Chain saw_chain;
  if(tree.getChain(base,"saw",saw_chain)&&saw_chain.getNrOfJoints()==0) {
    KDL::TreeFkSolverPos_recursive fk(tree); KDL::Frame b,saw;
    if(fk.JntToCart(joints,b,base)>=0&&fk.JntToCart(joints,saw,"saw")>=0) {
      planning_config_.saw_point=fromKdl(b.Inverse()*saw).translation();
      planning_config_.saw_point_valid=planning_config_.saw_point.allFinite();
    }
  }
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
    { std::ostringstream why; why<<"joint/body feedback skew="<<std::abs(s.joint_stamp-s.body_stamp)
        <<" s exceeds "<<synchronization_tolerance_<<" s"; s.invalid_reason=why.str(); }
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
  const double source=m->header.stamp.toSec(),receipt=ros::Time::now().toSec();
  if(source>0&&receipt-source>=-feedback_config_.future_tolerance&&receipt-source<=feedback_config_.timeout) {
    const auto indices=robot_model_->getJointIndexMap();
    for(int rotor=1;rotor<=4;++rotor) {
      const auto name="gimbal"+std::to_string(rotor); const auto it=std::find(m->name.begin(),m->name.end(),name);
      if(it==m->name.end()) continue;
      const auto i=static_cast<std::size_t>(it-m->name.begin());
      if(std::count(m->name.begin(),m->name.end(),name)!=1||i>=m->position.size()||!std::isfinite(m->position[i]))
        articulation_samples_[name].rate=std::numeric_limits<double>::quiet_NaN();
    }
    for(std::size_t i=0;i<m->name.size()&&i<m->position.size();++i) {
      if(!indices.count(m->name[i])||!std::isfinite(m->position[i])||
         std::count(m->name.begin(),m->name.end(),m->name[i])!=1) continue;
      auto& sample=articulation_samples_[m->name[i]];
      if(source<sample.stamp&&receipt<sample.stamp-feedback_config_.future_tolerance) sample=ArticulationSample();
      if(source<=sample.stamp) continue;
      const double dt=source-sample.stamp;
      const double rate=i<m->velocity.size()?m->velocity[i]:
        (sample.stamp>0&&dt>1e-4&&dt<=feedback_config_.timeout?(m->position[i]-sample.position)/dt:
         std::numeric_limits<double>::quiet_NaN());
      sample={m->position[i],rate,source};
    }
  }
  // Other publishers may send gimbal-only messages on this shared topic.
  if(std::find(m->name.begin(),m->name.end(),roll_)==m->name.end()&&
     std::find(m->name.begin(),m->name.end(),yaw_)==m->name.end()) return;
  Vec q=measured_.eta,rate=Vec::Zero(); const double now=ros::Time::now().toSec(),stamp=m->header.stamp.toSec();
  // Match multilink/pitch-only feedback handling: an old publication must not
  // invalidate the last accepted sample or refresh its measurement timestamp.
  // feedbackValid() still expires that sample if new measurements stop arriving.
  if(stamp>0&&stamp<measured_.joint_stamp&&now<measured_.joint_stamp-feedback_config_.future_tolerance) {
    measured_.joint_stamp=0; joint_valid_=false;
    if(lock_valid_) faultHold("joint feedback clock moved backwards; recapture");
  }
  if(stamp>0&&stamp<=measured_.joint_stamp) { ++joint_ignored_stamps_; return; }
  ++joint_invalid_messages_; last_joint_message_stamp_=stamp;
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
  servo_effort_.setConstant(std::numeric_limits<double>::quiet_NaN());
  for(int i=1;i<3;++i) {
    const auto it=std::find(m->name.begin(),m->name.end(),i==1?roll_:yaw_);
    const auto index=std::distance(m->name.begin(),it);
    if(static_cast<std::size_t>(index)<m->effort.size()&&std::isfinite(m->effort[index])) servo_effort_[i]=m->effort[index];
  }
  // Passive generalized actuator input is physically absent, not a sensor zero.
  servo_effort_[0]=0; effort_stamp_=stamp; effort_receipt_=joint_receipt_=now;
  measured_.eta=q; measured_.joint_rate=rate; measured_.joint_stamp=stamp; joint_valid_=rate.allFinite();
  if(joint_valid_) --joint_invalid_messages_;
  joint_failure_=joint_valid_?"":"joint angular rates unavailable/nonfinite (need velocity or two fresh samples)";
}
void GimbalrotorRollYawPerchingNavigator::bodyCallback(const nav_msgs::OdometryConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double stamp=m->header.stamp.toSec(),now=ros::Time::now().toSec();
  // The estimator publishes on a timer using the latest IMU measurement stamp;
  // consecutive odometry publications can legitimately share that stamp.
  // Keep the accepted sample, never replace its stamp with receipt time.
  if(stamp>0&&stamp<measured_.body_stamp&&now<measured_.body_stamp-feedback_config_.future_tolerance) {
    measured_.body_stamp=0; body_valid_=false;
    if(lock_valid_) faultHold("body feedback clock moved backwards; recapture");
  }
  if(stamp>0&&stamp<=measured_.body_stamp) { ++body_ignored_stamps_; return; }
  ++body_invalid_messages_; last_body_message_stamp_=stamp;
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
  body_receipt_=now; measured_.body_stamp=stamp; body_valid_=poseValid(measured_.body)&&measured_.body_omega.allFinite();
  if(body_valid_) --body_invalid_messages_;
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
  else navigationTargetConflict("uav/nav accepts lock-relative alpha only; use body_yaw_target/body_roll_target for body alignment");
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
  stop_pending_=false; preparation_window_.reset(); preparation_stats_=SettlingStatistics(); preparation_ready_=false; preparation_clock_=0;
  manual_path_=Path(); model_=std::move(model); nominal_=q; final_target_=held; body_alignment_valid_=false;
  lock_valid_=true; faulted_=false; previous_failure_.clear(); neutral_=held.body.linear();
  trajectory_mode_=false; traj_generator_ptr_.reset();
  gps_waypoint_=vel_based_waypoint_=force_att_control_flag_=false;
  commitFixedContactLock(tfVector(held.cog.translation()),tfQuaternion(held.cog.linear()),
                         tfVector(model_.pivotLock().translation()));
  lock_stamp_=perchingSession().lock_stamp;
  resetCompliance();
  const auto indices=robot_model_->getJointIndexMap();
  dynamics_model_=std::make_shared<ArticulatedDynamics>(model_,robot_model_->getTree(),robot_model_->getInertiaMap(),
    robot_model_->getExtraModuleMap(),robot_model_->getBaselinkName(),indices.at(roll_),indices.at(yaw_),
    -Vec(robot_model_->getGravity3d()));
  planning_model_=std::make_shared<Kinematics>(model_);
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
  lock_valid_=false; faulted_=false; lock_stamp_=ros::Time(); resetCompliance(); dynamics_model_.reset();
  manual_path_=Path(); stop_pending_=false; preparation_window_.reset(); preparation_stats_=SettlingStatistics(); preparation_ready_=false; preparation_clock_=0; body_alignment_valid_=false; nominal_.setZero(); // Never publish these reset values; capture must succeed first.
  manual_receipt_stamp_=ros::Time();
  // Base enable performs another reset on a rejected capture. Keep its cause.
  if(reason_.find("contact capture failed:")!=0) {
    if(previous_failure_.empty()) reason_="perching reference reset";
    else reason_="perching reference reset; previous failure: "+previous_failure_;
  }
}
void GimbalrotorRollYawPerchingNavigator::invalidatePlanner()
{
  if(prepare_pending_) ++prepare_cancellations_;
  ++producer_generation_; prepare_worker_.cancel();
  prepare_pending_=false;
  planner_.reset(); // Late workers hold only immutable model snapshots.
}
void GimbalrotorRollYawPerchingNavigator::stopMotion(const std::string& why)
{
  body_alignment_valid_=false;
  if(prepare_pending_) { ++prepare_cancellations_; ++producer_generation_; prepare_worker_.cancel(); prepare_pending_=false; }
  if(planner_&&(planner_->stage==Stage::EXECUTING||planner_->stage==Stage::PRE_ALIGN)) {
    planner_->command("cancel",lock_stamp_.toNSec()); stop_pending_=true;
    reason_=why+"; STOPPING, retry takeover after stop complete"; return;
  }
  if(!manual_path_.knots.empty()) {
    stop_pending_=true; reason_=why+"; STOPPING along applied manual path"; return;
  }
  invalidatePlanner(); resetCompliance(); admittance_reason_="disabled: stopped; prepare and tare again"; nominal_=final_target_.eta;
  Reference held;
  if(lock_valid_&&mechanismReference(model_,nominal_,Vec::Zero(),Vec::Zero(),held)) final_target_=held;
  stop_pending_=false; reason_=why+"; stop complete, holding contact";
}
void GimbalrotorRollYawPerchingNavigator::faultHold(const std::string& why)
{
  if(!faulted_) {
    if(prepare_pending_) ++prepare_cancellations_;
    ++producer_generation_; prepare_worker_.cancel();
    prepare_pending_=false; body_alignment_valid_=false; stop_pending_=false;
    if(planner_) planner_->fault(why);
    Reference held;
    if(lock_valid_&&mechanismReference(model_,final_target_.eta,Vec::Zero(),Vec::Zero(),held)) final_target_=held;
    admittance_.enabled=false; previous_failure_=reason_=why;
  }
  preparation_window_.reset(); preparation_stats_=SettlingStatistics(); preparation_ready_=false; preparation_clock_=0;
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
    { std::ostringstream why; why<<"pivot="<<contact.pivot_error<<"/"<<feedback_config_.pivot_tolerance
      <<" m; hinge="<<contact.hinge_residual<<"/"<<feedback_config_.hinge_tolerance<<" rad"; return why.str(); }
  return "";
}
void GimbalrotorRollYawPerchingNavigator::mechanismTargetCallback(const std_msgs::Float64ConstPtr& msg,int axis)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  Vec target=nominal_; target[axis]=msg->data; // Untouched destinations survive axis changes.
  setNominalTarget(target,axis);
}
void GimbalrotorRollYawPerchingNavigator::bodyAlignmentCallback(const std_msgs::Float64ConstPtr& msg,int axis)
{
  std::uint64_t session;
  { std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); session=lock_stamp_.toNSec(); }
  request("body_align",session,axis,msg->data);
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  std_msgs::String ack; ack.data=std::string("action=")+(axis==0?"body_yaw_target":"body_roll_target")+
    " session="+std::to_string(lock_stamp_.toNSec())+" accepted="+(last_command_accepted_?"true":"false")+" reason="+reason_;
  ack_pub_.publish(ack); last_diagnostics_=0; diagnostics();
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
  auto reject=[&](const std::string& why) { reason_="manual rejected: "+why; correction(); last_diagnostics_=0; diagnostics(); return false; };
  if(!lock_valid_||faulted_||!feedback().hover) return reject("require valid enabled HOVER contact");
  if(pitch_rezero_state_!=PitchRezeroState::IDLE) return reject("pitch rezero ACTIVE/HOLD; wait ready then final relock");
  if(!target.allFinite()) return reject("nonfinite mechanism target");
  const auto why=contactRejection(feedback());
  if(!why.empty()) return reject(why);
  const Vec bounded=target.cwiseMax(planning_config_.limits.lower).cwiseMin(planning_config_.limits.upper);
  std::string error;
  if(!mechanismAllowed(model_,bounded,planning_config_,error)) return reject(error);
  if(stop_pending_) return reject("stopping, retry; command not queued");
  if(planner_&&(planner_->stage==Stage::EXECUTING||planner_->stage==Stage::PRE_ALIGN)) {
    stopMotion("manual takeover requested"); return reject("stopping, retry; command not queued");
  }
  Path transition;
  const double manual_started=steadySeconds();
  if(!mechanismPath(model_,final_target_,bounded,planning_config_,transition,error)) return reject(error);
  manual_wall_=steadySeconds()-manual_started;
  // Multilink semantics: accept a destination, independent of its current applied
  // or measured value. No lookahead, endpoint-tracking gate or per-key executor.
  // Cancel the producer at its coherent applied reference before transferring
  // ownership; bounded contains the untouched independently accepted axes.
  if(planner_||prepare_pending_) stopMotion("manual takeover");
  invalidatePlanner(); body_alignment_valid_=false; nominal_=bounded; manual_receipt_stamp_=ros::Time::now();
  manual_path_=std::move(transition); manual_time_=ros::Time::now().toSec(); manual_phase_=0; manual_speed_=1; final_target_.t=0;
  reason_=(bounded-target).norm()>1e-10?"manual nominal target accepted at configured limit":"manual nominal target accepted";
  if((bounded-target).norm()>1e-10) correction();
  diagnostics(); return true;
}
bool GimbalrotorRollYawPerchingNavigator::buildFinalMechanismTarget(Reference& next)
{
  if(!planner_&&!manual_path_.knots.empty()) {
    const double now=ros::Time::now().toSec(),dt=now-manual_time_; manual_time_=now;
    if(dt==0) { next=final_target_; return true; }
    if(dt<0||dt>feedback_config_.max_execution_step) { faultHold("manual reference clock gap"); return false; }
    const double a=.5*std::min((planning_config_.limits.acceleration.array()/planning_config_.limits.velocity.array()).minCoeff(),
      planning_config_.cog_acceleration_limit/planning_config_.cog_speed_limit);
    const double v=stop_pending_?std::max(0.0,manual_speed_-a*dt):manual_speed_;
    manual_phase_=std::min(manual_path_.duration(),manual_phase_+.5*(manual_speed_+v)*dt);
    if(!samplePath(model_,manual_path_,manual_phase_,next)) { faultHold("invalid manual interpolation"); return false; }
    scaleReference(next,v,(v-manual_speed_)/dt); manual_speed_=v;
    if(manual_phase_>=manual_path_.duration()||(stop_pending_&&v==0)) {
      manual_path_=Path(); nominal_=next.eta; stop_pending_=false;
      mechanismReference(model_,nominal_,Vec::Zero(),Vec::Zero(),next);
      reason_="stop/motion complete; holding contact";
    }
    return true;
  }
  if(planner_) { next=planner_->target; return true; }
  const Vec bounded=nominal_.cwiseMax(planning_config_.limits.lower).cwiseMin(planning_config_.limits.upper);
  std::string error;
  if(!mechanismAllowed(model_,bounded,planning_config_,error)) { reason_="target FK rejected: "+error; return false; }
  Vec rate=Vec::Zero(),acc=Vec::Zero();
  return mechanismReference(model_,bounded,rate,acc,next);
}
void GimbalrotorRollYawPerchingNavigator::request(const std::string& action,std::uint64_t session,int axis,double value)
{
  Request requested; bool request_valid=true;
  // Preserve live per-request settings without holding navigation across XMLRPC.
  if(action=="prepare") {
    ros::NodeHandle pn(nh_,"navigation/roll_yaw_perching/request");
    request_valid=pn.getParam("pitch_increment",requested.pitch_end)&&pn.getParam("pitch_rate",requested.rate)&&
      pn.getParam("pitch_acceleration",requested.acceleration);
  }
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  last_command_accepted_=false;
  if(!lock_valid_||session!=lock_stamp_.toNSec()) { reason_="stale/missing base lock identity"; return; }
  if(action=="cancel"||action=="stop") {
    last_command_accepted_=true;
    if(pitch_rezero_state_==PitchRezeroState::ACTIVE) failPitchRezero("operator stopped rezero");
    else if(!faulted_) stopMotion("stopped at applied contact reference");
    last_diagnostics_=0; diagnostics(); return;
  }
  if(stop_pending_) { reason_="stopping, retry; command not queued"; return; }
  if(!planning_configuration_valid_) { reason_="invalid offline planning configuration"; return; }
  if(faulted_||pitch_rezero_state_!=PitchRezeroState::IDLE) { reason_="orientation request rejected during fault/setup"; return; }
  const auto rejection=contactRejection(feedback());
  if(!rejection.empty()) { reason_=rejection; return; }
  if(action=="prepare"||action=="body_align") {
    const bool alignment=action=="body_align";
    if(prepare_pending_) { reason_="worker busy; retry after status changes (request not queued)"; return; }
    if(planner_&&(planner_->stage==Stage::EXECUTING||planner_->stage==Stage::PRE_ALIGN)) {
      reason_="hold or complete current motion before preparing a cut"; return;
    }
    if(!request_valid) { reason_="require pitch_increment, pitch_rate and pitch_acceleration"; return; }
    Request request=requested;
    if(alignment) {
      request=alignment_request_;
      if(!body_alignment_valid_) {
        const Mat relative=neutral_.transpose()*feedback().body.linear();
        request=Request();
        request.roll=std::asin(std::clamp(relative(2,1),-1.0,1.0));
        request.yaw=std::atan2(-relative(0,1),relative(1,1));
        request.pitch_start=std::atan2(-relative(2,0),relative(2,2));
      }
      if(axis==0) request.yaw=value; else request.roll=value;
      request.align_only=true;
    }
    if(!request.valid()) { reason_="invalid orientation request (radians, positive rate/acceleration)"; return; }
    const auto f=feedback(); Estimate estimate;
    if(!model_.estimate(f.body,f.eta,estimate)) { reason_="invalid measured start"; return; }
    Vec initial=f.eta; initial[0]=estimate.alpha;
    Reference measured_start; measured_start.eta=initial; measured_start.body=f.body;
    Reference held_nominal;
    if(!mechanismReference(model_,initial,Vec::Zero(),Vec::Zero(),held_nominal)||
       logSO3(held_nominal.body.linear().transpose()*f.body.linear()).norm()>feedback_config_.preparation_tolerance) {
      reason_="measured start inconsistent with locked hinge FK"; return;
    }
    if(!preparation_ready_||f.body_omega.norm()>=feedback_config_.settle_body_rate||
       f.joint_rate.tail<2>().norm()>=feedback_config_.settle_joint_rate) {
      reason_="wait for short fresh quiet preparation window"; return;
    }
    // Noise-tolerant rate and reference-connection gates, not a zero-rate demand.
    if((initial-final_target_.eta).cwiseAbs().maxCoeff()>feedback_config_.preparation_tolerance||
       (!alignment&&((nominal_-initial).cwiseAbs().maxCoeff()>feedback_config_.preparation_tolerance||
         f.body_omega.norm()>feedback_config_.settle_body_rate||
         f.joint_rate.tail<2>().norm()>feedback_config_.settle_joint_rate))) {
      reason_="wait for current alignment to track/settle, or hold before preparing"; return;
    }
    if(!manual_path_.knots.empty()||final_target_.velocity.norm()>1e-6) {
      stopMotion("prepare requested during manual motion"); reason_="stopping, retry prepare; not queued"; return;
    }
    // Preserve applied reference; the worker adds a bounded connection to the
    // reconciled fresh measured start. No callback-time reference jump.
    const Reference connection_start=final_target_;
    resetCompliance();
    invalidatePlanner();

    planner_.reset(new Executor());
    planner_->config=feedback_config_; planner_->planning=planning_config_;
    if(!planner_->attach(model_,connection_start,session,f.now)||!planner_->command("prepare",session)) {
      reason_=planner_->reason; invalidatePlanner(); return;
    }
    // Cut R0 is the measured controlled-body orientation now, never lock neutral.
    const Mat neutral=alignment?neutral_:Mat(f.body.linear());
    planner_->plan.goal=request; planner_->plan.neutral=neutral; planner_->plan.session=session;
    planner_->planned_pitch=request.pitch_start; planner_->task.reset(request.pitch_start);
    // Model, measured/applied starts and generation are captured under the state lock.
    const auto model_snapshot=planning_model_; const auto config=planning_config_;
    const double consistency=feedback_config_.preparation_tolerance;
    pending_generation_=producer_generation_;
    prepare_started_=steadySeconds(); prepare_result_="pending"; ++plan_id_;
    planning_status_=std::make_shared<PlanningStatus>(); const auto status=planning_status_;
    const double deadline=prepare_deadline_;
    prepare_pending_=prepare_worker_.submit([model_snapshot,initial,neutral,request,measured_start,connection_start,config=PlanningConfig(config),session,status,deadline,consistency](const std::function<bool()>& cancel) mutable {
      const double started=steadySeconds();
      config.cancelled=[cancel,started,deadline]{return cancel()||steadySeconds()-started>deadline;};
      config.report=[status,last=started,was_validation=false](int i,int n,bool validation) mutable {
        const double now=steadySeconds(); auto& elapsed=was_validation?status->validation_seconds:status->solve_seconds;
        elapsed.store(elapsed.load()+now-last); last=now; was_validation=validation;
        status->interval=i; status->total=n; status->validating=validation;
      };
      const auto model=*model_snapshot;
      // Physical consistency is distinct from the numerical IK tolerance.
      Vec reconciled; std::string error;
      Plan plan;
      if(!inverseOrientation(model,measured_start.body.linear(),initial,config,reconciled,error)||
         (reconciled-initial).cwiseAbs().maxCoeff()>consistency||
         (reconciled-connection_start.eta).cwiseAbs().maxCoeff()>consistency)
        plan.reason="measured orientation/joints cannot be reconciled within contact tolerance: "+error;
      else {
        initial=reconciled;
        plan=prepare(model,request.align_only?connection_start.eta:initial,neutral,request,config);
      }
      plan.session=session;
      plan.submission=measured_start; plan.applied_start=connection_start; plan.submission_valid=true;
      if(plan.ok) {
        Path connection; std::string why;
        if(!request.align_only) {
          if(!mechanismPath(model,connection_start,initial,config,connection,why)) { plan.ok=false; plan.reason="start connection: "+why; }
          else plan.align=std::move(connection);
        }
      }
      plan.wall_seconds=steadySeconds()-started;
      if(cancel()||plan.wall_seconds>deadline) { plan.ok=false; plan.reason="offline cancelled/deadline exceeded"; }
      return plan;
    });
    if(!prepare_pending_) { planner_->hold("planner worker busy/cancelling; retry prepare"); }
    else { planner_->reason="JOB_ACCEPTED: preparing; motion not yet accepted"; }
    last_command_accepted_=prepare_pending_;
  } else if(!planner_) { reason_="no prepared path; prepare first"; return; }
  else {
    if((action=="start"&&planner_->stage!=Stage::READY)||(action=="resume"&&planner_->stage!=Stage::PAUSED)) {
      reason_="start requires measured READY; resume requires PAUSED; wait/retry (not queued)"; return;
    }
    if(action=="start") {
      const auto f=feedback(); Estimate e;
      if(!planner_->plan.ok||planner_->plan.cut.knots.empty()||!model_.estimate(f.body,f.eta,e)) { reason_="prepare first"; return; }
      Vec q=f.eta; q[0]=e.alpha;
      if((q-(planner_->plan.measured_start_valid?planner_->plan.measured_start.eta:planner_->plan.cut.knots.front().eta)).cwiseAbs().maxCoeff()>feedback_config_.goal_tolerance||
         logSO3(f.body.linear().transpose()*planner_->plan.neutral).norm()>feedback_config_.preparation_tolerance||
         (planner_->plan.measured_start_valid&&logSO3(f.body.linear().transpose()*planner_->plan.measured_start.body.linear()).norm()>feedback_config_.goal_tolerance)) {
        stopMotion("prepared alignment moved; prepare again"); return;
      }
    }
    if(action=="start"||action=="resume") {
      const auto current=feedback();
      if(!preparation_ready_||current.body_omega.norm()>=feedback_config_.settle_body_rate||
         current.joint_rate.tail<2>().norm()>=feedback_config_.settle_joint_rate) {
        reason_="motion denied: wait for fresh quiet body/joint window"; return;
      }
      if(admittance_.enabled&&(!interaction_valid_||!admittance_.baseline_ready||
          ros::Time::now().toSec()-interaction_stamp_>feedback_config_.timeout||
          steadySeconds()-interaction_receipt_>feedback_config_.timeout)) {
        reason_="motion denied: enabled admittance input/baseline unavailable; restore source or explicitly disable"; return;
      }
    }
    last_command_accepted_=planner_->command(action,session);
    if(action=="start"&&last_command_accepted_) body_alignment_valid_=false;
  }
  reason_=planner_->reason; diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::resetCompliance()
{
  dynamics_worker_.cancel(); dynamics_pending_=false;
  observer_.reset(); admittance_.reset(); torque_valid_=false;
  last_dynamics_stamp_=torque_stamp_=torque_receipt_=0;
  observer_started_=steadySeconds();
  external_wrench_=geometry_msgs::WrenchStamped();
  tare_requested_=false; interaction_valid_=false; interaction_stamp_=0; interaction_receipt_=0; compliance_time_=0;
  admittance_reason_="disabled: prepare to measured READY, then no-cut tare with a fresh external wrench";
}
void GimbalrotorRollYawPerchingNavigator::admittanceCallback(const std_msgs::BoolConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!msg->data) {
    admittance_.enabled=false;
    admittance_reason_="disabled; bounded correction release along prepared path";
  } else if(!interaction_configured_||!admittance_.config.valid()) {
    admittance_reason_="unavailable: invalid admittance dynamics or wrench frame/point/sign (+/-1)/scale (>0) configuration";
  } else if(!planner_||!planner_->plan.ok||planner_->plan.goal.align_only||planner_->stage!=Stage::READY||
            !contactRejection(feedback()).empty()) {
    admittance_reason_="unavailable: require a prepared cut at measured READY with valid contact feedback";
  } else if(!admittance_.baseline_ready) {
    admittance_reason_="unavailable: no completed baseline; tare at quiet no-cut READY with a fresh synchronized wrench";
  } else if(!interaction_valid_||interaction_stamp_<=0||ros::Time::now().toSec()-interaction_stamp_>feedback_config_.timeout||
            steadySeconds()-interaction_receipt_>feedback_config_.timeout||
            std::abs(interaction_stamp_-feedback().body_stamp)>synchronization_tolerance_) {
    admittance_reason_="unavailable: external wrench invalid, stale or unsynchronized with body feedback";
  } else {
    admittance_.enabled=true; admittance_reason_="enabled/ready: approximate path interaction; assumed no-cut tare";
  }
  ROS_INFO_STREAM("[RollYaw] "<<admittance_reason_); diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::tareCallback(const std_msgs::EmptyConstPtr&)
{
  std::uint64_t session,generation;
  { std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); session=lock_stamp_.toNSec(); generation=producer_generation_; }
  ros::NodeHandle an(nh_,"navigation/roll_yaw_perching/admittance");
  double sign=0,scale=0; std::string frame,point;
  an.param("wrench_sign",sign,interaction_topic_=="estimated_external_wrench"?1.0:0.0);
  an.param("wrench_scale",scale,interaction_topic_=="estimated_external_wrench"?1.0:0.0);
  an.param<std::string>("wrench_frame",frame,"world"); an.param<std::string>("wrench_point",point,"cog");
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(session!=lock_stamp_.toNSec()||generation!=producer_generation_||admittance_.enabled||
     !planner_||planner_->stage!=Stage::READY||std::abs(admittance_.offset)>1e-8) {
    admittance_reason_="tare rejected: disable, wait for zero correction and measured READY cut; retry after state change"; return;
  }
  admittance_.reset(); tare_requested_=false; interaction_valid_=false;
  interaction_sign_=sign; interaction_scale_=scale; interaction_frame_=frame; interaction_point_=point;
  interaction_configured_=admittance_.config.valid()&&(interaction_sign_==1||interaction_sign_==-1)&&
    std::isfinite(interaction_scale_)&&interaction_scale_>0&&
    (interaction_frame_=="world"||interaction_frame_=="body")&&
    (interaction_point_=="cog"||interaction_point_=="body"||interaction_point_=="world_origin");
  if(!interaction_configured_) {
    admittance_reason_="tare rejected: invalid admittance dynamics or wrench frame/point/sign (+/-1)/scale (>0) configuration"; return;
  }
  if(!planner_||!planner_->plan.ok||planner_->plan.goal.align_only||
     planner_->stage!=Stage::READY||admittance_.enabled||std::abs(admittance_.offset)>1e-8) {
    admittance_reason_="tare rejected: require READY cut, disabled and zero correction; remove cutting contact"; return;
  }
  admittance_.reset(); tare_requested_=true; interaction_stamp_=external_wrench_.header.stamp.toSec(); interaction_receipt_=0;
  admittance_reason_="taring: operator asserts no cutting contact; collecting fresh quiet samples";
}
void GimbalrotorRollYawPerchingNavigator::externalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double t=msg->header.stamp.toSec(),now=ros::Time::now().toSec();
  const auto& force=msg->wrench.force; const auto& moment=msg->wrench.torque;
  const std::string expected=interaction_frame_=="world"?world_:body_frame_;
  if(!std::isfinite(t)||t<=0||now-t>feedback_config_.timeout||now-t<-feedback_config_.future_tolerance||
     frameName(msg->header.frame_id)!=frameName(expected)||!Vec(force.x,force.y,force.z).allFinite()||
     !Vec(moment.x,moment.y,moment.z).allFinite()) {
    interaction_receipt_=0; interaction_valid_=false; admittance_reason_="unavailable: external wrench frame/stamp/values invalid"; return;
  }
  if(t<=external_wrench_.header.stamp.toSec()) return;
  external_wrench_=*msg; interaction_receipt_=steadySeconds();
}
void GimbalrotorRollYawPerchingNavigator::updateInteraction(const Feedback& f)
{
  const double dt=compliance_time_>0?f.now-compliance_time_:0; compliance_time_=f.now;
  if(!planner_||!planner_->plan.ok||planner_->plan.goal.align_only) return;
  const double stamp=external_wrench_.header.stamp.toSec();
  bool fresh=interaction_configured_&&interaction_receipt_>0&&steadySeconds()-interaction_receipt_<=feedback_config_.timeout&&
    contactRejection(f).empty()&&stamp>0&&f.now-stamp>=-feedback_config_.future_tolerance&&f.now-stamp<=feedback_config_.timeout&&
    std::abs(stamp-f.body_stamp)<=synchronization_tolerance_;
  const bool quiet=f.body_omega.norm()<feedback_config_.settle_body_rate&&
    f.joint_rate.tail<2>().norm()<feedback_config_.settle_joint_rate;
  if(tare_requested_&&!quiet) { admittance_.reset(); admittance_reason_="tare waiting for quiet no-cut posture"; }
  if(fresh&&stamp==interaction_stamp_) fresh=interaction_valid_;
  if(fresh&&stamp>interaction_stamp_) {
    Estimate e; Vec q=f.eta; Pose body,cog;
    fresh=model_.estimate(f.body,q,e); q[0]=e.alpha;
    if(fresh) fresh=model_.evaluate(q,body,cog);
    if(fresh) {
      const double direction=planner_->plan.goal.pitch_end>=planner_->plan.goal.pitch_start?1:-1;
      const auto& w=external_wrench_.wrench;
      Vec force(w.force.x,w.force.y,w.force.z),moment(w.torque.x,w.torque.y,w.torque.z);
      if(interaction_frame_=="body") { force=f.body.linear()*force; moment=f.body.linear()*moment; }
      Vec source=Vec::Zero();
      if(interaction_point_=="body") source=f.body.translation();
      if(interaction_point_=="cog") source=f.body*(body.inverse()*cog).translation();
      fresh=pathInteractionTorque(model_,q,direction,planning_config_.min_singular_value,
                                  force,moment,source,interaction_torque_);
      interaction_torque_*=interaction_sign_*interaction_scale_;
      fresh=fresh&&std::isfinite(interaction_torque_);
      if(fresh) {
          admittance_.observe(interaction_torque_,stamp,tare_requested_&&quiet&&planner_->stage==Stage::READY);
          interaction_stamp_=stamp;
          if(tare_requested_&&admittance_.baseline_ready) { tare_requested_=false; admittance_reason_="disabled/ready: no-cut baseline complete; enable explicitly"; }
      }
    }
  }
  if(tare_requested_&&!fresh) { admittance_.reset(); admittance_reason_="tare waiting for a continuous fresh synchronized no-cut source"; }
  if(admittance_.enabled&&!fresh) admittance_reason_="stale/unavailable interaction: bounded path stop, correction braking/held; original goal retained";
  else if(admittance_.enabled) admittance_reason_="enabled/ready: approximate external work, resistance reduces penetration";
  if(dt>0&&dt<=feedback_config_.max_execution_step) {
    if(!admittance_.step(dt,fresh,true)) { fresh=false; admittance_reason_="unavailable: scalar integration failed; holding"; }
  } else if(admittance_.enabled&&(dt<0||dt>feedback_config_.max_execution_step)) fresh=false;
  interaction_valid_=fresh;
  planner_->relief=std::min(0.0,admittance_.offset);
  planner_->interaction_hold=admittance_.enabled&&!fresh;
}
void GimbalrotorRollYawPerchingNavigator::wrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const double stamp=msg->header.stamp.toSec(),now=ros::Time::now().toSec();
  const auto& f=msg->wrench.force; const auto& t=msg->wrench.torque;
  if(stamp<=input_wrench_.header.stamp.toSec()) return;
  if(frameName(msg->header.frame_id)!=frameName(world_)||stamp<=0||now-stamp>feedback_config_.timeout||
     now-stamp<-feedback_config_.future_tolerance||!Vec(f.x,f.y,f.z).allFinite()||!Vec(t.x,t.y,t.z).allFinite()) {
    load_reason_="invalid command-derived wrench frame/stamp/value"; torque_valid_=false; return;
  }
  input_wrench_=*msg; wrench_receipt_=steadySeconds();
}
void GimbalrotorRollYawPerchingNavigator::updateDynamics(const Feedback& f)
{
  const double now=steadySeconds();
  if(auto result=dynamics_worker_.take()) {
    dynamics_pending_=false; dynamics_wall_=result->wall;
    if(result->session==lock_stamp_.toNSec()&&now-result->submitted<=feedback_config_.timeout&&
       f.now-result->stamp<=feedback_config_.timeout&&result->terms.ok) {
      const bool observed=observer_.update(result->terms,result->stamp,feedback_config_.timeout);
      if(!observed) observer_started_=now;
      torque_valid_=observed&&now-observer_started_>=1;
      if(torque_valid_) {
        torque_stamp_=result->stamp; torque_receipt_=now;
        // Diagnostic only; raw servo current scaling and command-derived
        // wrench do not establish an external cutting interaction estimate.
      }
      load_reason_=torque_valid_?result->reason:"diagnostic observer initializing/recovering; not an interaction input";
    } else { torque_valid_=false; load_reason_="rejected dynamics: "+result->reason; }
  }
  if(now-torque_receipt_>feedback_config_.timeout) torque_valid_=false;
  if(dynamics_pending_||now-last_dynamics_<.05||f.body_stamp<=last_dynamics_stamp_) return;
  const double source=input_wrench_.header.stamp.toSec();
  auto fresh=[&](double stamp){return stamp>0&&f.now-stamp>=-feedback_config_.future_tolerance&&f.now-stamp<=feedback_config_.timeout;};
  if(!dynamics_model_||!contactRejection(f).empty()||!servo_effort_.allFinite()||!fresh(effort_stamp_)||
     !fresh(source)||now-wrench_receipt_>feedback_config_.timeout||
     std::abs(source-f.body_stamp)>synchronization_tolerance_||
     std::abs(effort_stamp_-f.body_stamp)>synchronization_tolerance_) {
    load_reason_="no computable dynamics: require synchronized body, active effort and command-wrench samples";
    torque_valid_=false; return;
  }
  Estimate estimate; Mat j;
  if(!model_.estimate(f.body,f.eta,estimate)) return;
  Vec q=f.eta,v=f.joint_rate; q[0]=estimate.alpha;
  if(!model_.jacobian(q,j)) return;
  v[0]=j.col(0).dot(f.body_omega-j.col(1)*v[1]-j.col(2)*v[2]);
  auto joints=robot_model_->getJointPositions(); KDL::JntArray rates(joints.rows()); rates.data.setZero();
  unsigned missing=0;
  for(const auto& joint:robot_model_->getJointIndexMap()) {
    if(joint.second>=rates.rows()) continue;
    const auto found=articulation_samples_.find(joint.first);
    if(found!=articulation_samples_.end()&&fresh(found->second.stamp)&&std::abs(found->second.stamp-f.body_stamp)<=synchronization_tolerance_&&std::isfinite(found->second.rate)) {
      joints(joint.second)=found->second.position; rates(joint.second)=found->second.rate;
    }
    else if(joint.first!=roll_&&joint.first!=yaw_) ++missing;
  }
  const auto& force=input_wrench_.wrench.force; const auto& torque=input_wrench_.wrench.torque;
  const Vec fw(force.x,force.y,force.z),tw(torque.x,torque.y,torque.z),effort=servo_effort_;
  const auto model=dynamics_model_; const auto session=lock_stamp_.toNSec();
  const double stamp=f.body_stamp,sigma=planning_config_.min_singular_value;
  dynamics_pending_=dynamics_worker_.submit([model,q,v,joints,rates,fw,tw,effort,session,stamp,now,sigma,missing](const std::function<bool()>& cancelled) {
    DynamicsResult out; out.session=session; out.stamp=stamp; out.submitted=now;
    const double began=steadySeconds();
    out.terms=model->evaluate(q,v,joints,rates,fw,tw,effort,sigma,[cancelled,began]{return cancelled()||steadySeconds()-began>.1;});
    out.reason=out.terms.reason+"; unmeasured other joint rates frozen="+std::to_string(missing);
    out.wall=steadySeconds()-began; return out;
  });
  if(dynamics_pending_) { last_dynamics_=now; last_dynamics_stamp_=f.body_stamp; }
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
  const bool clock_valid=preparation_clock_==0||f.now>=preparation_clock_;
  preparation_clock_=f.now;
  if(!clock_valid) { faultHold("feedback clock moved backwards; recapture contact"); }
  if(why.empty()&&!faulted_) {
    // A faster source can change while min(body,joint) repeats. Its excursion
    // must invalidate quiet history, not disappear before the next joint sample.
    if(f.body_omega.norm()>=feedback_config_.settle_body_rate||
       f.joint_rate.tail<2>().norm()>=feedback_config_.settle_joint_rate) preparation_window_.reset();
    preparation_stats_=preparation_window_.observe(std::min(f.body_stamp,f.joint_stamp),
      {{f.body_omega.x(),f.body_omega.y(),f.body_omega.z()}},{{f.joint_rate[1],f.joint_rate[2]}},
      feedback_config_.preparation_quiet_time,feedback_config_.timeout);
    preparation_ready_=preparation_stats_.qualifies(feedback_config_.settle_body_rate,
      feedback_config_.settle_joint_rate,feedback_config_.settle_mean_rate);
  } else { preparation_window_.reset(); preparation_stats_=SettlingStatistics(); preparation_ready_=false; preparation_clock_=0; }
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
  if(!faulted_&&pitch_rezero_state_==PitchRezeroState::IDLE&&
     ((f.eta.tail<2>()-final_target_.eta.tail<2>()).cwiseAbs().maxCoeff()>feedback_config_.tracking_tolerance||
      logSO3(f.body.linear().transpose()*final_target_.body.linear()).norm()>feedback_config_.orientation_tolerance))
    faultHold("body/active-joint reference tracking tolerance exceeded");
  if(!faulted_) {
    if(pitch_rezero_state_!=PitchRezeroState::IDLE) applyPitchRezeroTarget();
    else {
      if(planner_&&!faulted_) {
        updateInteraction(f);
        planner_->applied=final_target_; planner_->applied_valid=true;
        planner_->tick(f);
        if(planner_->stage==Stage::FAULT_HOLD) faultHold("trajectory fault: "+planner_->first_fault);
        else { nominal_=planner_->target.eta; reason_=planner_->reason; }
      }
      Reference next;
      if(!faulted_&&buildFinalMechanismTarget(next)) {
        final_target_=next;
        if(stop_pending_&&planner_&&planner_->stage==Stage::COMPLETED_HOLD) {
          stop_pending_=false; invalidatePlanner(); nominal_=final_target_.eta; body_alignment_valid_=false;
          resetCompliance(); admittance_reason_="disabled: stopped; prepare and tare again"; reason_="stop complete; holding contact; takeover may be retried";
        }
        if(planner_&&planner_->plan.ok) {
          double residual;
          taskPitch(cuttingOrientation(planner_->plan.neutral,planner_->plan.goal,0,0,0).rotation,
                    final_target_.body.linear(),control_pitch_,control_pitch_,residual);
        }
      }
    }
  }
  if(!poseValid(final_target_.body)||!poseValid(final_target_.cog)) { faultHold("invalid controller reference"); return; }
  commandDesiredBaselinkPose(final_target_); publishJointTarget();
}
void GimbalrotorRollYawPerchingNavigator::update()
{
  const double update_started=steadySeconds();
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(force_landing_flag_&&ownsNavigationTargets()) selectPerchingMode(perching_geometry::Mode::NORMAL,false);
  if(lock_valid_&&perchingSession().lock_stamp!=lock_stamp_) faultHold("base lock identity replaced");
  updateBodyPitchFeedback(); target_applied_this_update_=false;
  GimbalrotorPerchingNavigator::update();
  // Roll-yaw owns coherent body/joint output throughout an enabled lock.
  if(lock_valid_&&!target_applied_this_update_) applyActivePerchingTarget();
  if(auto plan=prepare_worker_.take()) {
    const bool expected=prepare_pending_;
    prepare_pending_=false; prepare_wall_=plan->wall_seconds;
    if(expected) prepare_result_=plan->reason;
    if(expected&&planner_&&pending_generation_==producer_generation_&&lock_valid_&&!faulted_) {
      if(plan->session!=lock_stamp_.toNSec()) faultHold("invalid prepare worker result: "+plan->reason);
      else {
        const auto f=feedback();
        if(!contactRejection(f).empty()||!preparation_ready_||
           f.body_omega.norm()>=feedback_config_.settle_body_rate||f.joint_rate.tail<2>().norm()>=feedback_config_.settle_joint_rate||
           !planner_->submissionMatches(*plan,f,final_target_)) {
          planner_->hold("measurement or applied start changed during prepare; retry prepare/body target"); planner_->plan=Plan();
        } else if(planner_->accept(std::move(*plan))) {
          if(planner_->command("align",lock_stamp_.toNSec())) {
            if(planner_->plan.goal.align_only) { alignment_request_=planner_->plan.goal; body_alignment_valid_=true; }
            planner_->reason="PLAN_READY: geometric connection moving; wait measured READY";
          } else {
            planner_->hold("automatic alignment denied: "+planner_->reason+"; retry prepare/body target");
            planner_->plan=Plan(); // no unreachable private align command or false plan-ready acknowledgement
          }
        }
      }
      reason_=planner_->reason; prepare_result_=reason_;
      std_msgs::String result_ack;
      result_ack.data="action=prepare_result session="+std::to_string(lock_stamp_.toNSec())+
        " plan_ready="+(planner_->plan.ok?"true":"false")+
        " motion_active="+(planner_->stage==Stage::PRE_ALIGN?"true":"false")+" reason="+reason_;
      ack_pub_.publish(result_ack);
    }
  }
  if(lock_valid_&&!faulted_&&pitch_rezero_state_==PitchRezeroState::IDLE) {
    updateDynamics(feedback());
    if(!planner_) updateInteraction(feedback());
  }
  diagnostics();
  navigation_wall_=steadySeconds()-update_started;
}
void GimbalrotorRollYawPerchingNavigator::diagnostics()
{
  const auto f=feedback();
  const double now=steadySeconds();
  const std::string transition=!ownsNavigationTargets()?"FREE_FLIGHT":faulted_?"FAULT_HOLD":
    pitch_rezero_state_==PitchRezeroState::ACTIVE?"PITCH_REZERO_ACTIVE":
    pitch_rezero_state_==PitchRezeroState::HOLD?"PITCH_REZERO_HOLD":planner_?stageName(planner_->stage):"CONTACT_LOCKED";
  const bool changed=transition!=last_status_;
  if(changed) {
    if(faulted_) ROS_ERROR_STREAM("[RollYaw] "<<transition<<": "<<previous_failure_);
    else ROS_INFO_STREAM("[RollYaw] "<<transition<<": "<<reason_);
    last_status_=transition;
  }
  if(!changed&&now-last_diagnostics_<.2) return;
  last_diagnostics_=now;
  if(lock_valid_) ROS_INFO_STREAM_THROTTLE(2.0,"[RollYaw] "<<transition<<": "<<reason_
    <<"; diagnostic observer: "<<load_reason_<<"; "<<admittance_reason_);
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
    <<" lock="<<session.data<<" reason=["<<reason_<<"]"
    <<" nominal="<<nominal_.transpose()<<" applied="<<final_target_.eta.transpose()
    <<" measured_active="<<f.eta.tail<2>().transpose()<<" measured_alpha="<<measured_contact_.alpha
    <<" joint_age="<<f.now-f.joint_stamp<<" body_age="<<f.now-f.body_stamp
    <<" joint_failure=["<<joint_failure_<<"] body_failure=["<<body_failure_<<"]"
    <<" pivot_error="<<measured_contact_.pivot_error<<" hinge_residual="<<measured_contact_.hinge_residual
    <<" rezero_alpha="<<rezero_alpha_<<" rezero_pitch="<<rezero_previous_pitch_
    <<" rezero_pitch_rate="<<rezero_measured_rate_<<" rezero_rate_valid="<<rezero_rate_valid_
    <<" rezero_sign="<<rezero_sign_<<" manual_receipt="<<manual_receipt_stamp_.toSec()
    <<" ignored_body_stamps="<<body_ignored_stamps_<<" ignored_joint_stamps="<<joint_ignored_stamps_;
  out<<" pivot_limit="<<feedback_config_.pivot_tolerance<<" hinge_limit="<<feedback_config_.hinge_tolerance
     <<" first_fault=["<<previous_failure_<<"] admittance=["<<admittance_reason_<<"]";
  if(planner_) out<<" phase_s="<<planner_->progress<<" planned_pitch="<<planner_->planned_pitch
    <<" measured_pitch="<<planner_->task.raw<<" measured_progress_rad="<<planner_->task.distance
    <<" original_goal="<<planner_->plan.goal.pitch_end<<" goal_error="<<planner_->goal_error
    <<" control_pitch="<<control_pitch_<<" goal_reached="<<planner_->goal_reached;
  std_msgs::String detail; detail.data=out.str(); details_pub_.publish(detail);
  diagnostic_msgs::DiagnosticArray array; array.header.stamp=ros::Time::now();
  diagnostic_msgs::DiagnosticStatus diagnostic;
  diagnostic.name="gimbalrotor/roll_yaw_perching"; diagnostic.hardware_id="perching_lock_"+std::to_string(session.data);
  diagnostic.level=faulted_?diagnostic_msgs::DiagnosticStatus::ERROR:
    (prepare_pending_||(planner_&&planner_->progress_lag))?diagnostic_msgs::DiagnosticStatus::WARN:diagnostic_msgs::DiagnosticStatus::OK;
  diagnostic.message=std::string(state)+": "+reason_;
  auto field=[&](const std::string& name,const auto& value) {
    diagnostic_msgs::KeyValue item; item.key=name; std::ostringstream text; text<<std::setprecision(17)<<value;
    item.value=text.str(); diagnostic.values.push_back(std::move(item));
  };
  const double nan=std::numeric_limits<double>::quiet_NaN();
  field("lock_id",session.data); field("session_id",session.data); field("plan_id",plan_id_);
  field("producer_generation",producer_generation_); field("first_fault",previous_failure_);
  field("stopping",stop_pending_); field("manual_callback_wall_s",manual_wall_);
  field("geometry_prepared",planner_&&planner_->plan.ok);
  field("cut_duration_s",planner_&&planner_->plan.ok?planner_->plan.cut.duration():nan);
  field("time_scale",planner_&&planner_->plan.ok?std::pow(2.0,planner_->plan.scale_attempt):nan);
  field("saw_proxy_valid",planning_config_.saw_point_valid);
  field("saw_proxy_drop_m",planner_&&planner_->plan.ok?planner_->plan.proxy_drop:nan);
  field("body_alignment_pending",prepare_pending_&&planner_&&planner_->plan.goal.align_only);
  field("body_alignment_pending_yaw_rad",prepare_pending_&&planner_&&planner_->plan.goal.align_only?planner_->plan.goal.yaw:nan);
  field("body_alignment_pending_roll_rad",prepare_pending_&&planner_&&planner_->plan.goal.align_only?planner_->plan.goal.roll:nan);
  field("body_alignment_yaw_rad",body_alignment_valid_?alignment_request_.yaw:nan);
  field("body_alignment_roll_rad",body_alignment_valid_?alignment_request_.roll:nan);
  field("prepare_cancellations",prepare_cancellations_); field("prepare_result",prepare_result_); field("prepare_generation",pending_generation_);
  field("preparation_ready",preparation_ready_); field("preparation_window_s",preparation_stats_.observed);
  field("prepare_pending",prepare_pending_); field("prepare_elapsed_s",prepare_pending_?now-prepare_started_:prepare_wall_);
  field("prepare_solve_wall_s",planning_status_->solve_seconds.load());
  field("prepare_validation_wall_s",planning_status_->validation_seconds.load());
  field("prepare_deadline_s",prepare_deadline_); field("prepare_interval",planning_status_->interval.load());
  field("prepare_intervals",planning_status_->total.load()); field("prepare_validating",planning_status_->validating.load());
  field("body_stamp_s",f.body_stamp); field("joint_stamp_s",f.joint_stamp);
  field("body_age_s",f.now-f.body_stamp); field("joint_age_s",f.now-f.joint_stamp);
  field("body_receipt_age_s",f.now-body_receipt_); field("joint_receipt_age_s",f.now-joint_receipt_);
  field("feedback_skew_s",std::abs(f.body_stamp-f.joint_stamp)); field("feedback_skew_limit_s",synchronization_tolerance_);
  field("body_invalid_messages",body_invalid_messages_); field("joint_invalid_messages",joint_invalid_messages_);
  field("last_body_message_stamp_s",last_body_message_stamp_); field("last_joint_message_stamp_s",last_joint_message_stamp_);
  field("feedback_timeout_s",feedback_config_.timeout); field("body_ignored_stamps",body_ignored_stamps_);
  field("joint_ignored_stamps",joint_ignored_stamps_); field("body_failure",body_failure_); field("joint_failure",joint_failure_);
  field("pivot_error_m",lock_valid_?measured_contact_.pivot_error:nan); field("pivot_limit_m",feedback_config_.pivot_tolerance);
  field("hinge_residual_rad",lock_valid_?measured_contact_.hinge_residual:nan); field("hinge_limit_rad",feedback_config_.hinge_tolerance);
  field("effective_body_tracking_error_rad",lock_valid_?logSO3(f.body.linear().transpose()*final_target_.body.linear()).norm():nan);
  field("effective_body_tracking_limit_rad",feedback_config_.orientation_tolerance);
  field("effective_joint_tracking_error_rad",lock_valid_?(f.eta.tail<2>()-final_target_.eta.tail<2>()).cwiseAbs().maxCoeff():nan);
  field("effective_joint_tracking_limit_rad",feedback_config_.tracking_tolerance);
  const bool goal_valid=planner_&&planner_->plan.goal.valid();
  const bool measured_task_valid=goal_valid&&planner_->task.valid&&!faulted_&&
    f.now-planner_->task.stamp<=feedback_config_.timeout&&contactRejection(f).empty();
  const bool fresh_torque=interaction_valid_&&interaction_stamp_>0&&!faulted_&&f.now-interaction_stamp_<=feedback_config_.timeout&&now-interaction_receipt_<=feedback_config_.timeout;
  field("goal_valid",goal_valid); field("original_goal_rad",goal_valid?planner_->plan.goal.pitch_end:nan);
  field("planned_pitch_rad",goal_valid?planner_->planned_pitch:nan); field("nominal_phase_s",planner_?planner_->nominal_progress:nan);
  field("effective_phase_s",planner_?planner_->progress:nan);
  field("measured_task_valid",measured_task_valid);
  field("measured_pitch_rad",measured_task_valid?planner_->task.raw:nan);
  field("filtered_pitch_rad",measured_task_valid?planner_->task.filtered:nan);
  field("physical_progress_rad",measured_task_valid?planner_->task.distance:nan);
  field("cross_track_rad",measured_task_valid?planner_->task.cross_track:nan);
  field("goal_error_rad",measured_task_valid?planner_->goal_error:nan);
  field("goal_orientation_error_rad",measured_task_valid?planner_->goal_orientation_error:nan);
  field("control_pitch_rad",goal_valid?control_pitch_:nan);
  field("path_end_commanded",planner_&&planner_->path_end_commanded); field("goal_reached",planner_&&planner_->goal_reached);
  field("goal_pending",goal_valid&&!planner_->goal_reached); field("progress_lag",planner_&&planner_->progress_lag);
  field("reference_lead_rad",feedback_config_.reference_lead); field("goal_tolerance_rad",feedback_config_.goal_tolerance);
  field("current_body_rate_rad_s",body_valid_?f.body_omega.norm():nan);
  field("current_joint_rate_rad_s",joint_valid_?f.joint_rate.tail<2>().norm():nan);
  field("settling_valid",planner_&&planner_->settling.complete);
  field("settling_elapsed_s",planner_?planner_->settling_elapsed:nan);
  field("settling_window_s",planner_?planner_->settling.observed:nan);
  field("settling_body_rms_rad_s",planner_&&planner_->settling.complete?planner_->settling.body_rms:nan);
  field("settling_joint_rms_rad_s",planner_&&planner_->settling.complete?planner_->settling.joint_rms:nan);
  field("settling_body_mean_rad_s",planner_&&planner_->settling.complete?planner_->settling.body_mean:nan);
  field("settling_joint_mean_rad_s",planner_&&planner_->settling.complete?planner_->settling.joint_mean:nan);
  field("settling_body_limit_rad_s",feedback_config_.settle_body_rate); field("settling_joint_limit_rad_s",feedback_config_.settle_joint_rate);
  field("settling_mean_limit_rad_s",feedback_config_.settle_mean_rate); field("settling_timeout_s",feedback_config_.settle_timeout);
  field("admittance_enabled",admittance_.enabled); field("admittance_status",admittance_reason_);
  field("contact",admittance_.contact); field("baseline_ready",admittance_.baseline_ready); field("baseline_samples",admittance_.samples);
  field("bias_Nm",admittance_.baseline_ready?admittance_.bias:nan); field("task_torque_valid",fresh_torque);
  field("task_torque_Nm",fresh_torque?interaction_torque_:nan); field("torque_stamp_s",interaction_stamp_);
  field("interaction_source",interaction_topic_); field("interaction_frame",interaction_frame_); field("interaction_point",interaction_point_);
  field("interaction_sign",interaction_sign_); field("interaction_scale",interaction_scale_);
  field("command_observer_torque_UNVERIFIED_Nm",torque_valid_?observer_.torque:nan);
  field("torque_age_s",interaction_stamp_>0?f.now-interaction_stamp_:nan); field("reference_torque_Nm",admittance_.config.reference_torque);
  field("filtered_contact_torque_Nm",admittance_.baseline_ready?admittance_.filtered:nan);
  double effective_offset=nan;
  if(goal_valid&&!planner_->plan.goal.align_only&&planner_->plan.ok) {
    double p=0,cross=0;
    taskPitch(planner_->plan.neutral,planner_->plan.cut.desired(planner_->nominal_progress).rotation,0,p,cross);
    const double direction=planner_->plan.goal.pitch_end>=0?1:-1;
    effective_offset=direction*(planner_->planned_pitch-p);
  }
  field("effective_offset_rad",effective_offset);
  field("offset_rad",admittance_.offset); field("offset_rate_rad_s",admittance_.rate);
  field("offset_acceleration_rad_s2",admittance_.acceleration); field("load_source",load_reason_);
  field("servo_roll_effort_Nm_estimate",f.now-effort_stamp_<=feedback_config_.timeout?servo_effort_[1]:nan);
  field("servo_yaw_effort_Nm_estimate",f.now-effort_stamp_<=feedback_config_.timeout?servo_effort_[2]:nan);
  field("effort_stamp_s",effort_stamp_); field("command_wrench_stamp_s",input_wrench_.header.stamp.toSec());
  field("dynamics_wall_s",dynamics_wall_); field("navigation_wall_s",navigation_wall_);
  array.status.push_back(std::move(diagnostic)); status_pub_.publish(array);
  geometry_msgs::Vector3Stamped plot; plot.header=array.header;
  plot.header.frame_id="perching_lock_"+std::to_string(session.data);
  plot.vector.x=goal_valid?planner_->planned_pitch:nan;
  plot.vector.y=measured_task_valid?planner_->task.raw:nan;
  plot.vector.z=goal_valid?control_pitch_:nan; task_pub_.publish(plot);
  plot.vector.x=admittance_.offset; plot.vector.y=admittance_.rate;
  plot.vector.z=fresh_torque?interaction_torque_:nan; compliance_pub_.publish(plot);
}
}
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorRollYawPerchingNavigator,aerial_robot_navigation::BaseNavigator)
