#include <gimbalrotor/gimbalrotor_roll_yaw_perching_navigation.h>
#include <kdl/treefksolverpos_recursive.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_eigen.h>
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
  auto& p=core_.planning; auto& c=core_.config;
  bool vectors=readVector(pn,"position_lower",p.limits.lower)&&readVector(pn,"position_upper",p.limits.upper)&&
    readVector(pn,"velocity_caps",p.limits.velocity)&&readVector(pn,"acceleration_caps",p.limits.acceleration)&&
    readVector(pn,"qp_weights",p.qp.weight);
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
  pn.param("require_settled_on_enable",c.require_settled_on_enable,false);
  pn.param("manual_max_step",manual_max_step_,manual_max_step_);
  pn.param("manual_command_timeout",manual_timeout_,manual_timeout_);
  pn.param("manual_tracking_tolerance",manual_tracking_tolerance_,manual_tracking_tolerance_);
  pn.param("feedback_sync_tolerance",synchronization_tolerance_,synchronization_tolerance_);
  pn.param("cog_model_tolerance",cog_model_tolerance_,cog_model_tolerance_);
  configuration_valid_=configuration_valid_&&poseValid(cp_)&&vectors&&p.valid()&&c.valid()&&
    std::isfinite(synchronization_tolerance_)&&synchronization_tolerance_>0&&
    std::isfinite(cog_model_tolerance_)&&cog_model_tolerance_>0&&
    std::isfinite(manual_max_step_)&&manual_max_step_>0&&manual_max_step_<=M_PI/180&&
    std::isfinite(manual_timeout_)&&manual_timeout_>0&&manual_timeout_<=1&&
    std::isfinite(manual_tracking_tolerance_)&&manual_tracking_tolerance_>0&&
    manual_tracking_tolerance_<std::min(c.tracking_tolerance,c.orientation_tolerance);
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
  angles_pub_=nh.advertise<geometry_msgs::Vector3Stamped>(prefix_+"/target_angles",1);
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
  subscriptions_.push_back(nh.subscribe(prefix_+"/manual_delta",1,
    &GimbalrotorRollYawPerchingNavigator::manualCallback,this));
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
    core_.planning.limits.lower[i]=std::max(core_.planning.limits.lower[i],joint->limits->lower);
    core_.planning.limits.upper[i]=std::min(core_.planning.limits.upper[i],joint->limits->upper);
    core_.planning.limits.velocity[i]=std::min(core_.planning.limits.velocity[i],joint->limits->velocity);
  }
  if(!core_.planning.valid()) { error="empty URDF/configured limit intersection"; return false; }
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
  if(stamp<=0||now-stamp>core_.config.timeout||now-stamp<-core_.config.future_tolerance) return;
  joint_failure_="missing/duplicate joint name or invalid position";
  for(int i=1;i<3;++i) {
    const auto& name=i==1?roll_:yaw_;
    if(std::count(m->name.begin(),m->name.end(),name)!=1) return;
    const auto index=std::distance(m->name.begin(),std::find(m->name.begin(),m->name.end(),name));
    if(static_cast<std::size_t>(index)>=m->position.size()||!std::isfinite(m->position[index])) return;
    q[i]=m->position[index];
    if(static_cast<std::size_t>(index)<m->velocity.size()) rate[i]=m->velocity[index];
    else if(measured_.joint_stamp>0&&stamp-measured_.joint_stamp>1e-4&&stamp-measured_.joint_stamp<=core_.config.timeout)
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
  if(stamp<=0||now-stamp>core_.config.timeout||now-stamp<-core_.config.future_tolerance) return;
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
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); return core_.owns(); }
void GimbalrotorRollYawPerchingNavigator::navigationTargetConflict(const char* source)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); core_.fault(std::string("conflicting navigation target: ")+source); }
void GimbalrotorRollYawPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& m)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); if(acceptNavigationTarget("uav/nav")) GimbalrotorNavigator::naviCallback(m); }
void GimbalrotorRollYawPerchingNavigator::applyManualPitchDelta(double)
{ navigationTargetConflict("legacy manual_pitch_delta; use session-stamped roll_yaw/manual_delta"); }
bool GimbalrotorRollYawPerchingNavigator::tryLockPerching(const std::string&)
{
  lock_failure_.clear();
  Kinematics model; std::string error;
  if(!snapshotModel(model,error)) { lock_failure_=core_.reason=error; return false; }
  const auto sample=feedback();
  if(!core_.capture(sample,model)) {
    std::ostringstream why;
    why<<core_.reason<<"; joint_valid="<<joint_valid_<<" body_valid="<<body_valid_
       <<" body_frame="<<body_frame_<<" hover="<<sample.hover
       <<" joint_age="<<sample.now-sample.joint_stamp<<" body_age="<<sample.now-sample.body_stamp
       <<" joint_rate="<<sample.joint_rate.tail<2>().norm()<<" body_rate="<<sample.body_omega.norm()
       <<" settle_rate="<<core_.config.settle_rate
       <<" require_settled_on_enable="<<core_.config.require_settled_on_enable;
    lock_failure_=core_.reason=why.str(); return false;
  }
  output_inhibited_=false; neutral_=core_.target.body.linear(); trajectory_mode_=false; traj_generator_ptr_.reset();
  gps_waypoint_=vel_based_waypoint_=force_att_control_flag_=false;
  commitFixedContactLock(tfVector(core_.target.cog.translation()),tfQuaternion(core_.target.cog.linear()),
                         tfVector(core_.model.pivotLock().translation()));
  lock_stamp_=perchingSession().lock_stamp;
  applyActivePerchingTarget(); return true;
}
void GimbalrotorRollYawPerchingNavigator::resetPerchingLock()
{
  core_.reset(); disable_pending_=false; manual_stamp_=ros::Time::now();
  GimbalrotorPerchingNavigator::resetPerchingLock();
  if(!lock_failure_.empty()) core_.reason=lock_failure_;
}
void GimbalrotorRollYawPerchingNavigator::selectPerchingMode(perching_geometry::Mode mode,bool enable)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(enable&&core_.owns()) {
    core_.reason="existing contact retained; explicitly disable before capturing another lock"; return;
  }
  if(!enable&&mode==perching_geometry::Mode::NORMAL&&
     (core_.stage==Stage::PRE_ALIGN||core_.stage==Stage::EXECUTING||core_.stage==Stage::MANUAL)) {
    core_.command("cancel",core_.session); disable_pending_=true;
    core_.reason="disable requested; decelerating before release"; diagnostics(); return;
  }
  lock_failure_.clear();
  GimbalrotorPerchingNavigator::selectPerchingMode(mode,enable);
  if(enable&&!core_.owns()&&lock_failure_.empty())
    lock_failure_=supportsPerchingMode(mode)?"enable requires HOVER without forced landing":"unsupported perching mode";
  if(enable&&!lock_failure_.empty()) core_.reason=lock_failure_;
  diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  // Empty legacy messages cannot authorize a new reference while attached.
  core_.reason="relock requires explicit disable then enable with fresh valid HOVER feedback";
}
void GimbalrotorRollYawPerchingNavigator::resetCallback(const std_msgs::EmptyConstPtr&)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); core_.fault("explicit reset: holding contact; disable/enable to recapture"); }
void GimbalrotorRollYawPerchingNavigator::request(const std::string& action,std::uint64_t session)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(output_inhibited_) { core_.reason="outputs inhibited after flight-state/lock loss; disable and recapture"; return; }
  if(disable_pending_) { core_.reason="disable pending; motion rejected"; return; }
  const auto rejection=core_.feedbackRejection(feedback());
  if(!rejection.empty()) { core_.reason=rejection; return; }
  if(action=="prepare"&&pending_.valid()) { core_.reason="previous planning worker still active"; return; }
  if(!core_.command(action,session)) return;
  if(action=="prepare") {
    ros::NodeHandle pn(nh_,"navigation/roll_yaw_perching/request"); Request r;
    pn.param("align_only",r.align_only,false);
    bool valid=pn.getParam("body_yaw",r.yaw)&&pn.getParam("body_roll",r.roll)&&pn.getParam("pitch_start",r.pitch_start)&&
      pn.getParam("pitch_end",r.pitch_end)&&pn.getParam("pitch_rate",r.rate)&&pn.getParam("pitch_acceleration",r.acceleration);
    if(!valid||!r.valid()) { core_.stage=Stage::CONTACT_LOCKED; core_.reason="invalid/incomplete request parameters (radians, seconds)"; return; }
    const auto model=core_.model; const auto q=core_.target.eta; const auto neutral=neutral_; const auto config=core_.planning;
    planning_started_=ros::Time::now().toSec();
    pending_=std::async(std::launch::async,[model,q,neutral,r,config,session]() {
      auto result=prepare(model,q,neutral,r,config); result.session=session; return result;
    });
  }
  diagnostics();
}
std::string GimbalrotorRollYawPerchingNavigator::manualTrackingRejection(
  const Feedback& f,const Reference& reference) const
{
  const auto why=core_.feedbackRejection(f);
  if(!why.empty()) return why;
  Estimate measured;
  if(!core_.model.estimate(f.body,f.eta,measured)) return "invalid hinge estimate";
  Vec q=f.eta; q[0]=measured.alpha;
  if((q.array()<core_.planning.limits.lower.array()).any()||
     (q.array()>core_.planning.limits.upper.array()).any()||
     f.body.linear()(2,2)<std::cos(core_.planning.max_tilt))
    return "measured mechanism/body outside operating range";
  if(measured.pivot_error>core_.config.pivot_tolerance||measured.hinge_residual>core_.config.hinge_tolerance)
    return "physical pivot moved or hinge residual exceeded";
  const double joint_error=(q-reference.eta).cwiseAbs().maxCoeff();
  const double body_error=logSO3(f.body.linear().transpose()*reference.body.linear()).norm();
  if(!std::isfinite(joint_error)||!std::isfinite(body_error)) return "invalid tracking error";
  if(joint_error>manual_tracking_tolerance_||body_error>manual_tracking_tolerance_) {
    std::ostringstream why;
    why<<"manual tracking bound exceeded [rad]: mechanism="<<joint_error
       <<" body_SO3="<<body_error<<" limit="<<manual_tracking_tolerance_
       <<"; holding last reference, no increment queued";
    return why.str();
  }
  return "";
}
void GimbalrotorRollYawPerchingNavigator::manualCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto f=feedback();
  auto reject=[&](const std::string& why){core_.reason="manual rejected: "+why; diagnostics();};
  // frame_id is an exact session token, NOT a TF frame to transform. Timestamp
  // ordering survives keyboard restart and rejects duplicates, even while busy.
  if(!core_.owns()||msg->header.frame_id!="perching_lock_"+std::to_string(core_.session))
  { reject("stale/missing session"); return; }
  const double age=f.now-msg->header.stamp.toSec();
  if(msg->header.stamp.isZero()||msg->header.stamp<=manual_stamp_||
     age<0||age>manual_timeout_)
  { reject("stale/future/duplicate command stamp"); return; }
  manual_stamp_=msg->header.stamp;
  const Vec delta(msg->vector.x,msg->vector.y,msg->vector.z); // roll, local pitch, yaw [rad]
  if(!delta.allFinite()) { reject("nonfinite increment"); return; }
  if(delta.isZero(0)) {
    // Stop never changes the lock or returns to its initial angles. A cancelled
    // planning result cannot be accepted after REZERO is left.
    if(core_.stage!=Stage::FAULT_HOLD)
      core_.command("cancel",core_.session);
    diagnostics(); return;
  }
  if(output_inhibited_||disable_pending_) { reject("outputs inhibited/disable pending"); return; }
  const auto why=core_.feedbackRejection(f);
  if(!why.empty()) { reject(why); return; }
  if(pending_.valid()||(core_.stage!=Stage::CONTACT_LOCKED&&core_.stage!=Stage::COMPLETED_HOLD)||
     (core_.stage==Stage::CONTACT_LOCKED&&core_.plan.ok))
  { reject("busy; no increment queued (cancel any automatic plan first)"); return; }
  if((delta.array()!=0).count()!=1||delta.cwiseAbs().maxCoeff()>manual_max_step_)
  { reject("require one axis and a step within manual_max_step [rad]"); return; }
  const auto tracking=manualTrackingRejection(f,core_.target);
  if(!tracking.empty()) { reject(tracking); return; }
  // Intrinsic Z-X-Y: R_WB = N * Rz(yaw) * Rx(roll) * Ry(local pitch).
  // Derive from the held FK reference, never from a drifting measured pose.
  const Mat relative=neutral_.transpose()*core_.target.body.linear();
  Request r; r.align_only=true;
  r.roll=std::asin(std::max(-1.0,std::min(1.0,relative(2,1))))+delta.x();
  r.pitch_start=r.pitch_end=std::atan2(-relative(2,0),relative(2,2))+delta.y();
  r.yaw=std::remainder(std::atan2(-relative(0,1),relative(1,1))+delta.z(),2*M_PI);
  if(!r.valid()) { reject("requested contact-relative angle outside supported range"); return; }
  auto config=core_.planning;
  // A subdegree request needs tighter IK accuracy than a large automatic move.
  config.orientation_tolerance=std::min(config.orientation_tolerance,delta.norm()*.05);
  // Discrete bounded motion, never a latched rate: loss of keyboard traffic
  // finishes at most this increment. No stream/heartbeat is needed to stop it.
  config.max_duration=std::min(config.max_duration,5.0);
  if(!core_.command("prepare",core_.session)) return;
  core_.reason="planning one manual increment; additional input is discarded";
  const auto model=core_.model; const auto initial=core_.target.eta; const auto neutral=neutral_;
  const auto session=core_.session;
  planning_started_=f.now;
  pending_=std::async(std::launch::async,[model,initial,neutral,r,config,session]() {
    auto result=prepare(model,initial,neutral,r,config);
    result.session=session; result.manual=true;
    if(result.ok) {
      Pose body,cog;
      if(!model.evaluate(result.align.knots.back().eta,body,cog)) {
        result.ok=false; result.reason="invalid manual endpoint FK";
      } else {
        const Mat relative=neutral.transpose()*body.linear();
        std::ostringstream accepted;
        accepted<<"manual increment accepted; endpoint in lock frame [rad]: roll="
          <<std::asin(std::max(-1.0,std::min(1.0,relative(2,1))))
          <<" local_pitch="<<std::atan2(-relative(2,0),relative(2,2))
          <<" yaw="<<std::atan2(-relative(0,1),relative(1,1));
        result.reason=accepted.str();
      }
    }
    return result;
  });
  diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::applyActivePerchingTarget()
{
  if(!core_.owns()||output_inhibited_||navi_state_!=HOVER_STATE||force_landing_flag_) return;
  const auto& r=core_.target;
  if(!poseValid(r.body)||!poseValid(r.cog)) { core_.fault("invalid controller reference"); return; }
  const Mat relative=r.cog.linear().transpose()*r.body.linear();
  setSynchronizedBaselinkRotationRelativeToCog(tfQuaternion(relative));
  double roll,pitch,yaw; tf::Matrix3x3(tfQuaternion(r.cog.linear())).getRPY(roll,pitch,yaw);
  setXyControlMode(POS_CONTROL_MODE); setTargetPos(tfVector(r.cog.translation()));
  setTargetVel(tfVector(r.cog_velocity)); setTargetAcc(tfVector(r.cog_acceleration)); setTargetRPY(tf::Vector3(roll,pitch,yaw));
  setTargetOmega(tfVector(r.cog_omega)); setTargetAngAcc(tfVector(r.cog_angular_acceleration));
  sensor_msgs::JointState cmd; cmd.header.stamp=ros::Time::now(); cmd.name={roll_,yaw_}; cmd.position={r.eta[1],r.eta[2]};
  joints_pub_.publish(cmd); // Position-only interface; never publish passive alpha.
}
void GimbalrotorRollYawPerchingNavigator::update()
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  GimbalrotorNavigator::update(); // retain battery, halt and landing behavior
  if(core_.owns()) {
    const auto session=perchingSession();
    if(session.mode!=perching_geometry::Mode::NORMAL||session.lock_stamp!=lock_stamp_)
      { output_inhibited_=true; core_.fault("perching lock lost or replaced"); }
    if(navi_state_!=HOVER_STATE||force_landing_flag_) output_inhibited_=true;
    core_.tick(feedback());
    // Detect drift of the non-planned articulated model (e.g. gimbals/extra mass).
    if(robot_model_&&robot_model_->initialized()) {
      const auto bg=robot_model_->getCog<KDL::Frame>();
      const auto frames=robot_model_->getSegmentsTf(); const auto b=frames.find(robot_model_->getBaselinkName());
      Pose expected_body,expected_cog;
      if(b!=frames.end()&&core_.model.evaluate(measured_.eta,expected_body,expected_cog)) {
        const Vec actual=fromKdl(b->second.Inverse()*bg).translation();
        const Vec expected=(expected_body.inverse()*expected_cog).translation();
        if((actual-expected).norm()>cog_model_tolerance_) core_.fault("articulated CoG changed beyond snapshot tolerance; replan required");
      }
    }
  }
  if(pending_.valid()&&pending_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
    try {
      auto plan=pending_.get();
      const double planning_age=ros::Time::now().toSec()-planning_started_;
      if(plan.manual&&(planning_age<0||planning_age>2.0)) {
        plan.ok=false; plan.reason="manual planning timed out; increment discarded";
      }
      if(plan.manual&&plan.ok&&core_.stage==Stage::REZERO&&plan.session==core_.session) {
        // Recheck after asynchronous planning, using the actual FK endpoint.
        // A stationary/lagging mechanism cannot accumulate an unbounded series
        // of accepted steps, even when each individual increment is small.
        Reference endpoint;
        if(!samplePath(core_.model,plan.align,plan.align.duration(),endpoint)) {
          plan.ok=false; plan.reason="invalid manual endpoint reference";
        } else {
          const auto rejection=manualTrackingRejection(feedback(),endpoint);
          if(!rejection.empty()) { plan.ok=false; plan.reason="manual endpoint rejected: "+rejection; }
        }
      }
      core_.accept(std::move(plan));
    }
    catch(const std::exception& e) { core_.fault(std::string("planning exception: ")+e.what()); }
  }
  if(disable_pending_&&(core_.stage==Stage::COMPLETED_HOLD||core_.stage==Stage::FAULT_HOLD))
    selectPerchingMode(perching_geometry::Mode::NORMAL,false);
  applyActivePerchingTarget(); diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::diagnostics()
{
  std_msgs::UInt64 id; id.data=core_.session; session_pub_.publish(id);
  std_msgs::String state; state.data=std::string(stageName(core_.stage))+": "+
    (core_.stage==Stage::FAULT_HOLD?core_.first_fault:core_.reason); state_pub_.publish(state);
  const auto& r=core_.target; const auto f=feedback();
  const bool publishing=core_.owns()&&!output_inhibited_&&f.hover&&poseValid(r.body)&&poseValid(r.cog);
  if(core_.owns()) {
    // Intrinsic Z-X-Y coordinates relative to the frozen contact orientation N.
    // These are body targets, not motor positions or world-frame Euler angles.
    const Mat relative=neutral_.transpose()*r.body.linear();
    geometry_msgs::Vector3Stamped angles;
    angles.header.stamp=ros::Time::now();
    angles.header.frame_id="perching_lock_"+std::to_string(core_.session);
    angles.vector.x=std::asin(std::max(-1.0,std::min(1.0,relative(2,1))));
    angles.vector.y=std::atan2(-relative(2,0),relative(2,2));
    angles.vector.z=std::atan2(-relative(0,1),relative(1,1));
    angles_pub_.publish(angles);
    nav_msgs::Odometry body; body.header.stamp=ros::Time::now(); body.header.frame_id=world_; body.child_frame_id=body_frame_;
    body.pose.pose=poseMsg(r.body); tf::vector3TFToMsg(tfVector(r.body_omega),body.twist.twist.angular); reference_pub_.publish(body);
    body.header.stamp.fromSec(f.body_stamp); body.pose.pose=poseMsg(f.body);
    tf::vector3TFToMsg(tfVector(f.body_omega),body.twist.twist.angular); measured_pub_.publish(body);
    body.header.stamp=ros::Time::now(); body.child_frame_id="desired_cog"; body.pose.pose=poseMsg(r.cog);
    tf::vector3TFToMsg(tfVector(r.cog_omega),body.twist.twist.angular);
    tf::vector3TFToMsg(tfVector(r.cog.linear().transpose()*r.cog_velocity),body.twist.twist.linear); cog_pub_.publish(body);
  }
  Mat j; double alpha_rate=std::numeric_limits<double>::quiet_NaN();
  Vec q=f.eta; q[0]=core_.measured.alpha;
  if(core_.owns()&&core_.feedbackValid(f)&&core_.model.jacobian(q,j))
    alpha_rate=j.col(0).dot(f.body_omega-j.col(1)*f.joint_rate[1]-j.col(2)*f.joint_rate[2]);
  const auto yaw=[](const Mat& rotation){return std::atan2(rotation(1,0),rotation(0,0));};
  const double nan=std::numeric_limits<double>::quiet_NaN();
  Vec error=Vec::Constant(nan);
  if(core_.owns()&&body_valid_) error=logSO3(f.body.linear().transpose()*r.body.linear());
  std::ostringstream out; out<<std::setprecision(12)
    <<"units: rad, rad/s, rad/s^2, m, s; eta=[passive,roll,yaw]; poses=W; omega=B/G; "
    <<"state="<<stageName(core_.stage)<<" session="<<core_.session
    <<" enable_rejection=["<<lock_failure_<<"] reason=["<<core_.reason<<"]"
    <<" first_fault=["<<core_.first_fault<<"]"
    <<" ignored_body_stamps="<<body_ignored_stamps_<<" ignored_joint_stamps="<<joint_ignored_stamps_
    <<" manual_tracking_bound_rad="<<manual_tracking_tolerance_
    <<" body_yaw_measured_W_rad="<<(body_valid_?yaw(f.body.linear()):nan)
    <<" body_yaw_reference_W_rad="<<(core_.owns()?yaw(r.body.linear()):nan)
    <<" joint_yaw_measured_rad="<<(joint_valid_?f.eta[2]:nan)
    <<" joint_yaw_command_rad="<<(publishing?std::to_string(r.eta[2]):"absent")
    <<" body_rate_B_rad_s="<<f.body_omega.transpose()
    <<" joint_rates_rad_s="<<f.joint_rate.tail<2>().transpose()
    <<" body_rate_valid="<<(body_valid_&&f.now-f.body_stamp<=core_.config.timeout&&f.now-f.body_stamp>=-core_.config.future_tolerance)
    <<" joint_rate_valid="<<(joint_valid_&&f.now-f.joint_stamp<=core_.config.timeout&&f.now-f.joint_stamp>=-core_.config.future_tolerance)
    <<" orientation_error_measured_B_rad="<<error.transpose()
    <<" joint_failure=["<<joint_failure_<<"] body_failure=["<<body_failure_<<"]"
    <<" manual_last_stamp_s="<<manual_stamp_.toSec()
    <<" progress_s="<<core_.progress<<" eta_des="<<r.eta.transpose()
    <<" eta_measured="<<q.transpose()<<" eta_rate_des="<<r.velocity.transpose()
    <<" passive_rate_est="<<alpha_rate<<" joint_age="<<f.now-f.joint_stamp<<" body_age="<<f.now-f.body_stamp
    <<" output_inhibited="<<output_inhibited_<<" feedback_valid="<<core_.feedbackValid(f)<<" pivot_error_m="<<core_.measured.pivot_error
    <<" hinge_residual_rad="<<core_.measured.hinge_residual<<" orientation_residual="<<r.orientation_error
    <<" qp_plan_ok="<<core_.plan.ok<<" scale_attempt="<<core_.plan.scale_attempt
    <<" omega_body_ideal="<<r.desired.omega.transpose()<<" body_acc="<<r.body_acceleration.transpose()
    <<" cog_acc_W="<<r.cog_acceleration.transpose()<<" cog_angular_acc_G="<<r.cog_angular_acceleration.transpose()
    <<" desired_body_quaternion_xyzw="<<Eigen::Quaterniond(r.desired.rotation).coeffs().transpose();
  std_msgs::String debug; debug.data=out.str(); details_pub_.publish(debug);
}
}
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorRollYawPerchingNavigator,aerial_robot_navigation::BaseNavigator)
