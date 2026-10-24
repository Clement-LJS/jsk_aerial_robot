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
  pn.param<std::string>("body_frame",body_frame_,"gimbalrotor/base_link");
  bool calibrated=false; pn.param("calibration_confirmed",calibrated,false);
  Vec xyz=Vec::Zero(),rpy=Vec::Zero();
  calibration_valid_=calibrated&&readVector(pn,"pivot_offset",xyz,true)&&readVector(pn,"pivot_rpy",rpy,true)&&
    readVector(pn,"hinge_axis",axis_,true)&&std::abs(axis_.norm()-1)<1e-7;
  cp_.translation()=xyz; cp_.linear()=expSO3(Vec::UnitZ()*rpy.z())*expSO3(Vec::UnitY()*rpy.y())*expSO3(Vec::UnitX()*rpy.x());
  auto& p=core_.planning; auto& c=core_.config;
  bool vectors=readVector(pn,"position_lower",p.limits.lower)&&readVector(pn,"position_upper",p.limits.upper)&&
    readVector(pn,"velocity_caps",p.limits.velocity)&&readVector(pn,"acceleration_caps",p.limits.acceleration)&&
    readVector(pn,"qp_weights",p.qp.weight);
  pn.param("planning_dt",p.dt,p.dt); pn.param("body_max_tilt",p.max_tilt,p.max_tilt);
  double general_tilt=.2; nh.getParam("navigation/max_target_tilt_angle",general_tilt);
  p.max_tilt=std::min(p.max_tilt,general_tilt); // no implicit safety-envelope expansion
  if(!std::isfinite(general_tilt)||general_tilt<=0) vectors=false;
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
  pn.param("feedback_sync_tolerance",synchronization_tolerance_,synchronization_tolerance_);
  pn.param("cog_model_tolerance",cog_model_tolerance_,cog_model_tolerance_);
  calibration_valid_=calibration_valid_&&vectors&&p.valid()&&c.valid()&&
    std::isfinite(synchronization_tolerance_)&&synchronization_tolerance_>0&&
    std::isfinite(cog_model_tolerance_)&&cog_model_tolerance_>0;
  bool underactuated=false; nh.param("controller/underactuate",underactuated,false);
  bool allocation_in_fc=true; nh.param("controller/gimbal_calc_in_fc",allocation_in_fc,true);
  std::string controller; nh.getParam("aerial_robot_control_name",controller);
  if(underactuated||allocation_in_fc||controller!="aerial_robot_control/gimbalrotor_controller") calibration_valid_=false;
  std::string joint_topic,command_topic,body_topic;
  pn.param<std::string>("joint_state_topic",joint_topic,"joint_states");
  pn.param<std::string>("joint_command_topic",command_topic,"joints_ctrl");
  pn.param<std::string>("body_state_topic",body_topic,"uav/baselink/odom");
  joints_pub_=nh.advertise<sensor_msgs::JointState>(command_topic,1);
  state_pub_=nh.advertise<std_msgs::String>(prefix_+"/state",1,true);
  session_pub_=nh.advertise<std_msgs::UInt64>(prefix_+"/session",1,true);
  details_pub_=nh.advertise<std_msgs::String>(prefix_+"/diagnostics",1);
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
  if(!calibration_valid_) ROS_ERROR("[RollYaw] Lock disabled: require confirmed physical hinge calibration, valid limits and ordinary fully actuated controller.");
  diagnostics();
}
bool GimbalrotorRollYawPerchingNavigator::snapshotModel(Kinematics& out,std::string& error)
{
  if(!calibration_valid_||!robot_model_||!robot_model_->initialized()) { error="model/calibration unavailable"; return false; }
  const auto indices=robot_model_->getJointIndexMap();
  if(indices.count(roll_)!=1||indices.count(yaw_)!=1) { error="missing roll/yaw model joints"; return false; }
  const unsigned ri=indices.at(roll_),yi=indices.at(yaw_);
  auto joints=robot_model_->getJointPositions();
  if(ri>=joints.rows()||yi>=joints.rows()||!joints.data.allFinite()) return false;
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
  KDL::Chain chain; if(!tree.getChain(base,contact_,chain)) return false;
  // Snapshot all model data before launching the planning worker. No mutation of
  // the live robot model and no ROS access from the worker.
  auto cog=articulatedCogModel(tree,joints,inertia,extra,base,relative,ri,yi);
  return out.configure(chain,roll_,yaw_,cp_,axis_,cog,error);
}
Feedback GimbalrotorRollYawPerchingNavigator::feedback() const
{
  Feedback s=measured_; s.now=ros::Time::now().toSec(); s.hover=navi_state_==HOVER_STATE&&!force_landing_flag_;
  s.valid=joint_valid_&&body_valid_&&std::abs(s.joint_stamp-s.body_stamp)<=synchronization_tolerance_;
  if(estimator_) for(auto axis:{State::X_BASE,State::Y_BASE,State::Z_BASE})
    s.valid=s.valid&&estimator_->getStateStatus(axis,estimate_mode_);
  else s.valid=false;
  return s;
}
void GimbalrotorRollYawPerchingNavigator::jointCallback(const sensor_msgs::JointStateConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  // Other publishers may send gimbal-only messages on this shared topic.
  if(std::find(m->name.begin(),m->name.end(),roll_)==m->name.end()&&
     std::find(m->name.begin(),m->name.end(),yaw_)==m->name.end()) return;
  Vec q=measured_.eta,rate=Vec::Zero(); const double now=ros::Time::now().toSec(),stamp=m->header.stamp.toSec();
  joint_valid_=false;
  if(stamp<=0||stamp<=measured_.joint_stamp||now-stamp>core_.config.timeout||now-stamp<-core_.config.future_tolerance) return;
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
}
void GimbalrotorRollYawPerchingNavigator::bodyCallback(const nav_msgs::OdometryConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  body_valid_=false;
  const double stamp=m->header.stamp.toSec(),now=ros::Time::now().toSec();
  if(stamp<=0||stamp<=measured_.body_stamp||now-stamp>core_.config.timeout||now-stamp<-core_.config.future_tolerance||
     frameName(m->header.frame_id)!=frameName(world_)||frameName(m->child_frame_id)!=frameName(body_frame_)) return;
  const auto& p=m->pose.pose; Eigen::Quaterniond q(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z);
  if(!q.coeffs().allFinite()||std::abs(q.norm()-1)>1e-3) return;
  measured_.body=Pose::Identity(); measured_.body.linear()=q.normalized().toRotationMatrix();
  measured_.body.translation()=Vec(p.position.x,p.position.y,p.position.z);
  const auto& w=m->twist.twist.angular; measured_.body_omega=Vec(w.x,w.y,w.z);
  measured_.body_stamp=stamp; body_valid_=poseValid(measured_.body)&&measured_.body_omega.allFinite();
}
bool GimbalrotorRollYawPerchingNavigator::ownsNavigationTargets() const
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); return core_.owns(); }
void GimbalrotorRollYawPerchingNavigator::navigationTargetConflict(const char* source)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); core_.fault(std::string("conflicting navigation target: ")+source); }
void GimbalrotorRollYawPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& m)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); if(acceptNavigationTarget("uav/nav")) GimbalrotorNavigator::naviCallback(m); }
void GimbalrotorRollYawPerchingNavigator::applyManualPitchDelta(double)
{ navigationTargetConflict("legacy manual_pitch_delta; use prepared stroke"); }
bool GimbalrotorRollYawPerchingNavigator::tryLockPerching(const std::string&)
{
  Kinematics model; std::string error;
  if(!snapshotModel(model,error)) { core_.reason=error; return false; }
  if(!core_.capture(feedback(),model)) return false;
  output_inhibited_=false; neutral_=core_.target.body.linear(); trajectory_mode_=false; traj_generator_ptr_.reset();
  gps_waypoint_=vel_based_waypoint_=force_att_control_flag_=false;
  commitFixedContactLock(tfVector(core_.target.cog.translation()),tfQuaternion(core_.target.cog.linear()),
                         tfVector(core_.model.pivotLock().translation()));
  lock_stamp_=perchingSession().lock_stamp;
  applyActivePerchingTarget(); return true;
}
void GimbalrotorRollYawPerchingNavigator::resetPerchingLock()
{ core_.reset(); GimbalrotorPerchingNavigator::resetPerchingLock(); }
void GimbalrotorRollYawPerchingNavigator::selectPerchingMode(perching_geometry::Mode mode,bool enable)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(enable&&core_.owns()) {
    core_.reason="existing contact retained; explicitly disable before capturing another lock"; return;
  }
  if(core_.stage==Stage::PRE_ALIGN||core_.stage==Stage::EXECUTING||core_.stage==Stage::REZERO) {
    core_.reason="cancel and settle before changing contact lock"; return;
  }
  GimbalrotorPerchingNavigator::selectPerchingMode(mode,enable); diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  // Empty legacy messages cannot authorize a new reference while attached.
  core_.reason="relock requires explicit disable then enable with fresh settled feedback";
}
void GimbalrotorRollYawPerchingNavigator::resetCallback(const std_msgs::EmptyConstPtr&)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); core_.fault("explicit reset: holding contact; disable/enable to recapture"); }
void GimbalrotorRollYawPerchingNavigator::request(const std::string& action,std::uint64_t session)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(output_inhibited_) { core_.reason="outputs inhibited after flight-state/lock loss; disable and recapture"; return; }
  if(!core_.feedbackValid(feedback())) { core_.reason="request requires fresh settled/valid feedback"; return; }
  if(action=="prepare"&&pending_.valid()) { core_.reason="previous planning worker still active"; return; }
  if(!core_.command(action,session)) return;
  if(action=="prepare") {
    ros::NodeHandle pn(nh_,"navigation/roll_yaw_perching/request"); Request r;
    bool valid=pn.getParam("body_yaw",r.yaw)&&pn.getParam("body_roll",r.roll)&&pn.getParam("pitch_start",r.pitch_start)&&
      pn.getParam("pitch_end",r.pitch_end)&&pn.getParam("pitch_rate",r.rate)&&pn.getParam("pitch_acceleration",r.acceleration);
    if(!valid||!r.valid()) { core_.stage=Stage::CONTACT_LOCKED; core_.reason="invalid/incomplete request parameters (radians, seconds)"; return; }
    const auto model=core_.model; const auto q=core_.target.eta; const auto neutral=neutral_; const auto config=core_.planning;
    pending_=std::async(std::launch::async,[model,q,neutral,r,config,session]() {
      auto result=prepare(model,q,neutral,r,config); result.session=session; return result;
    });
  }
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
    try { core_.accept(pending_.get()); }
    catch(const std::exception& e) { core_.fault(std::string("planning exception: ")+e.what()); }
  }
  applyActivePerchingTarget(); diagnostics();
}
void GimbalrotorRollYawPerchingNavigator::diagnostics()
{
  std_msgs::UInt64 id; id.data=core_.session; session_pub_.publish(id);
  std_msgs::String state; state.data=std::string(stageName(core_.stage))+": "+core_.reason; state_pub_.publish(state);
  if(!core_.owns()) return;
  const auto& r=core_.target; const auto f=feedback();
  nav_msgs::Odometry body; body.header.stamp=ros::Time::now(); body.header.frame_id=world_; body.child_frame_id=body_frame_;
  body.pose.pose=poseMsg(r.body); tf::vector3TFToMsg(tfVector(r.body_omega),body.twist.twist.angular); reference_pub_.publish(body);
  body.header.stamp.fromSec(f.body_stamp); body.pose.pose=poseMsg(f.body);
  tf::vector3TFToMsg(tfVector(f.body_omega),body.twist.twist.angular); measured_pub_.publish(body);
  body.header.stamp=ros::Time::now(); body.child_frame_id="desired_cog"; body.pose.pose=poseMsg(r.cog);
  tf::vector3TFToMsg(tfVector(r.cog_omega),body.twist.twist.angular);
  tf::vector3TFToMsg(tfVector(r.cog.linear().transpose()*r.cog_velocity),body.twist.twist.linear); cog_pub_.publish(body);
  Mat j; double alpha_rate=std::numeric_limits<double>::quiet_NaN();
  Vec q=f.eta; q[0]=core_.measured.alpha;
  if(core_.feedbackValid(f)&&core_.model.jacobian(q,j))
    alpha_rate=j.col(0).dot(f.body_omega-j.col(1)*f.joint_rate[1]-j.col(2)*f.joint_rate[2]);
  std::ostringstream out; out<<std::setprecision(12)
    <<"units: rad, rad/s, rad/s^2, m, s; eta=[passive,roll,yaw]; poses=W; omega=B/G; "
    <<"session="<<core_.session<<" progress_s="<<core_.progress<<" eta_des="<<r.eta.transpose()
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
