#include <gimbalrotor/gimbalrotor_pitch_only_perching_navigation.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <iomanip>

using namespace aerial_robot_navigation;
using namespace aerial_robot_model;
using namespace pitch_only;

void GimbalrotorPitchOnlyPerchingNavigator::initialize(ros::NodeHandle nh,ros::NodeHandle nhp,
  boost::shared_ptr<RobotModel> model,boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,double dt)
{
  GimbalrotorPerchingNavigator::initialize(nh,nhp,model,estimator,dt);
  ros::NodeHandle pn(nh,"navigation/pitch_only_perching");
  pn.param<std::string>("pitch_joint_name",joint_name_,"joint_pitch_only");
  pn.param<std::string>("contact_link_name",contact_link_,"pitch_joint_only_hand_link");
  pn.param<std::string>("world_frame",world_frame_,"world");
  pn.param<std::string>("topic_prefix",prefix_,"perching/pitch_only");
  pn.param("pitch_command_sign",pitch_command_sign_,1.0);
  pitch_command_sign_=std::isfinite(pitch_command_sign_)&&pitch_command_sign_<0?-1:1;
  pn.param("pitch_axis_alignment_threshold",pitch_axis_alignment_threshold_,.90);
  pn.getParam("pitch_lower_limit",configured_lower_);
  pn.getParam("pitch_upper_limit",configured_upper_);
  pn.getParam("pitch_rate_limit",configured_rate_);
  auto& c=core_.config;
  pn.param("body_pitch_lower_limit",c.body_pitch_min,c.body_pitch_min);
  pn.param("body_pitch_upper_limit",c.body_pitch_max,c.body_pitch_max);
  pn.param("joint_state_timeout",c.feedback_timeout,c.feedback_timeout);
  pn.param("joint_state_future_tolerance",c.future_tolerance,c.future_tolerance);
  pn.param("pitch_zero_tolerance",c.q_zero_tolerance,c.q_zero_tolerance);
  pn.param("pitch_position_tolerance",c.q_hold_error,c.q_hold_error);
  pn.param("pitch_velocity_tolerance",c.joint_settle_rate,c.joint_settle_rate);
  pn.param("pitch_rezero_rate_tolerance",c.rezero_rate_tolerance,c.rezero_rate_tolerance);
  pn.param("pitch_rezero_stable_duration",c.settle_duration,c.settle_duration);
  pn.param("pitch_rezero_target_pitch",c.rezero_pitch,c.rezero_pitch);
  pn.param("pitch_rezero_command_sign",c.rezero_sign,c.rezero_sign);
  c.rezero_sign=std::isfinite(c.rezero_sign)&&c.rezero_sign<0?-1:1;
  pn.param("pitch_rezero_kp",c.rezero_kp,c.rezero_kp);
  pn.param("pitch_rezero_rate_limit",c.rezero_rate,c.rezero_rate);
  pn.param("pitch_rezero_max_delta",c.rezero_range,c.rezero_range);
  pn.param("pitch_rezero_pitch_tolerance",c.rezero_pitch_tolerance,c.rezero_pitch_tolerance);
  pn.param("pitch_rezero_timeout",c.rezero_timeout,c.rezero_timeout);
  pn.param("passive_lower_limit",c.passive_min,c.passive_min);
  pn.param("passive_upper_limit",c.passive_max,c.passive_max);
  pn.param("passive_rate_limit",c.passive_rate,c.passive_rate);
  auto vector=[&pn](const std::string& key,KDL::Vector& v) {
    if(!pn.hasParam(key)) return;
    std::vector<double> a;
    if(pn.getParam(key,a)&&a.size()==3) v=KDL::Vector(a[0],a[1],a[2]);
    else v=KDL::Vector(unknown,unknown,unknown);
  };
  vector("pivot_offset",c.pivot_offset); vector("hinge_axis",c.hinge_axis);
  if(finite(c.hinge_axis)&&c.hinge_axis.Norm()>1e-6) c.hinge_axis=c.hinge_axis/c.hinge_axis.Norm();
  model_resolution_attempted_=robot_model_&&robot_model_->initialized();
  if(model_resolution_attempted_) model_valid_=resolveModel();
  core_.model=[this](double q,KDL::Frame& bc,KDL::Vector& g) { return modelAt(q,bc,g); };
  std::string command_topic,joint_topic;
  pn.param<std::string>("joint_command_topic",command_topic,"joints_ctrl");
  pn.param<std::string>("joint_state_topic",joint_topic,"joint_states");
  joints_pub_=nh.advertise<sensor_msgs::JointState>(command_topic,1);
  pose_pub_=nh.advertise<geometry_msgs::PoseStamped>(prefix_+"/target_body_pose",1);
  pivot_pub_=nh.advertise<geometry_msgs::PointStamped>(prefix_+"/locked_pivot",1);
  state_pub_=nh.advertise<std_msgs::String>(prefix_+"/state",1);
  for(const auto* key:{"model_valid","contact_valid","rezero_active","rezero_ready","rezero_failed","servo_frozen","cutting_enabled","feedback_valid"})
    bool_pubs_[key]=nh.advertise<std_msgs::Bool>(prefix_+"/"+key,1);
  for(const auto* key:{"servo_measured","servo_target_nominal","servo_hold_error","body_pitch","body_pitch_rate","passive_alpha_cmd","passive_alpha_measured","cut_alpha_min","cut_alpha_max","pivot_error","lock_generation"})
    number_pubs_[key]=nh.advertise<std_msgs::Float64>(prefix_+"/"+key,1);
  subscribers_.push_back(nh.subscribe(joint_topic,1,&GimbalrotorPitchOnlyPerchingNavigator::jointCallback,this));
  for(const auto* key:{"confirm_detached_zero","pitch_rezero","freeze","unfreeze","prepare_unperch","confirm_released","recover"}) {
    const std::string action(key);
    subscribers_.push_back(nh.subscribe<std_msgs::Empty>(prefix_+"/"+action,1,
      [this,action](const std_msgs::EmptyConstPtr&) { request(action); }));
  }
  subscribers_.push_back(nh.subscribe<std_msgs::Float64>(prefix_+"/servo_command",1,
    [this](const std_msgs::Float64ConstPtr& m) {
      std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
      core_.servo(pitch_command_sign_*m->data,sample());
    }));
  if(model_resolution_attempted_&&!model_valid_)
    ROS_ERROR("[PitchOnly] Invalid mechanism/parameters; check joint limits, pivot axis and positive rates/timeouts.");
}

bool GimbalrotorPitchOnlyPerchingNavigator::resolveModel()
{
  if(!robot_model_||!robot_model_->initialized()||contact_link_.empty()) return false;
  const auto j=robot_model_->getUrdfModel().getJoint(joint_name_);
  if(!j||(j->type!=urdf::Joint::REVOLUTE&&j->type!=urdf::Joint::CONTINUOUS)||j->mimic) return false;
  KDL::Vector a(j->axis.x,j->axis.y,j->axis.z);
  if(!finite(a)||a.Norm()<1e-6||!std::isfinite(pitch_axis_alignment_threshold_)||
     pitch_axis_alignment_threshold_<=0||pitch_axis_alignment_threshold_>1) return false;
  a=a/a.Norm();
  if(std::abs(a.y())<pitch_axis_alignment_threshold_) return false;
  const auto& indices=robot_model_->getJointIndexMap();
  const auto it=indices.find(joint_name_);
  if(it==indices.end()||it->second>=robot_model_->getJointPositions().rows()) return false;
  joint_index_=it->second;
  KDL::Chain chain;
  if(!robot_model_->getTree().getChain(robot_model_->getBaselinkName(),contact_link_,chain)||chain.getNrOfJoints()!=1) return false;
  for(unsigned i=0;i<chain.getNrOfSegments();++i) {
    const auto& k=chain.getSegment(i).getJoint();
    if(k.getType()!=KDL::Joint::None&&k.getName()!=joint_name_) return false;
  }
  auto& c=core_.config;
  c.q_min=configured_lower_; c.q_max=configured_upper_;
  if(j->type==urdf::Joint::REVOLUTE&&j->limits) {
    c.q_min=std::isnan(configured_lower_)?j->limits->lower:std::max(j->limits->lower,configured_lower_);
    c.q_max=std::isnan(configured_upper_)?j->limits->upper:std::min(j->limits->upper,configured_upper_);
  }
  // Preserve the URDF velocity cap; an optional YAML value can reduce it.
  c.q_rate=j->limits&&std::isfinite(j->limits->velocity)&&j->limits->velocity>0?j->limits->velocity:configured_rate_;
  if(!std::isnan(configured_rate_)) c.q_rate=std::min(c.q_rate,configured_rate_);
  if(!c.valid()) return false;
  ROS_INFO("[PitchOnly] pitch limits [%.4f, %.4f] rad, rate %.4f rad/s, command sign %.0f",
           c.q_min,c.q_max,c.q_rate,pitch_command_sign_);
  return true;
}

bool GimbalrotorPitchOnlyPerchingNavigator::modelAt(double q,KDL::Frame& bc,KDL::Vector& cog) const
{
  if(!model_valid_||!std::isfinite(q)) return false;
  auto joints=robot_model_->getJointPositions();
  if(joint_index_>=joints.rows()) return false;
  joints(joint_index_)=q;
  const auto frames=robot_model_->fullForwardKinematics(joints);
  const auto b=frames.find(robot_model_->getBaselinkName()),c=frames.find(contact_link_);
  if(b==frames.end()||c==frames.end()||!finite(b->second)||!finite(c->second)) return false;
  bc=b->second.Inverse()*c->second;
  KDL::RigidBodyInertia total=KDL::RigidBodyInertia::Zero();
  for(const auto& item:robot_model_->getInertiaMap()) {
    auto f=frames.find(item.first);
    if(f==frames.end()||!finite(f->second)) return false;
    total=total+f->second*item.second;
    for(const auto& extra:robot_model_->getExtraModuleMap())
      if(extra.second.getName()==item.first) total=total+f->second*(extra.second.getFrameToTip()*extra.second.getInertia());
  }
  if(!std::isfinite(total.getMass())||total.getMass()<=1e-9) return false;
  cog=b->second.Inverse()*total.getCOG();
  return finite(bc)&&finite(cog);
}

Sample GimbalrotorPitchOnlyPerchingNavigator::sample() const
{
  Sample s=feedback_; s.now=ros::Time::now().toSec(); s.hover=navi_state_==HOVER_STATE;
  const double receive_age=s.now-joint_receive_stamp_;
  s.model_valid=model_valid_&&joint_receive_stamp_>0&&receive_age>=0&&receive_age<=core_.config.feedback_timeout;
  if(!estimator_) { s.model_valid=false; return s; }
  // Same estimator source and frame convention as multilink; no extra odometry subscription.
  const auto p=estimator_->getPos(Frame::BASELINK,estimate_mode_);
  const auto rotation=estimator_->getOrientation(Frame::BASELINK,estimate_mode_);
  tf::Quaternion q; rotation.getRotation(q);
  if(!std::isfinite(q.length2())||q.length2()<1e-12) { s.model_valid=false; return s; }
  q.normalize();
  s.body=KDL::Frame(KDL::Rotation::Quaternion(q.x(),q.y(),q.z(),q.w()),KDL::Vector(p.x(),p.y(),p.z()));
  const auto omega=estimator_->getAngularVel(Frame::BASELINK,estimate_mode_);
  s.angular_speed=omega.length();
  s.pitch_rate=pitchRate(s.body,KDL::Vector(omega.x(),omega.y(),omega.z()));
  return s;
}
void GimbalrotorPitchOnlyPerchingNavigator::jointCallback(const sensor_msgs::JointStateConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto it=std::find(m->name.begin(),m->name.end(),joint_name_);
  if(it==m->name.end()||std::count(m->name.begin(),m->name.end(),joint_name_)!=1) return;
  const auto index=std::distance(m->name.begin(),it);
  if(static_cast<std::size_t>(index)>=m->position.size()||!std::isfinite(m->position[index])) return;
  const double now=ros::Time::now().toSec();
  const double stamp=m->header.stamp.isZero()?now:m->header.stamp.toSec();
  if(!core_.fresh(now,stamp)||stamp<=feedback_.joint_stamp) return;
  const double q=m->position[index];
  double v=unknown;
  if(static_cast<std::size_t>(index)<m->velocity.size()&&std::isfinite(m->velocity[index])) v=m->velocity[index];
  else if(feedback_.joint_stamp>0 && stamp-feedback_.joint_stamp>1e-6 && stamp-feedback_.joint_stamp<=.5)
    v=(q-feedback_.q)/(stamp-feedback_.joint_stamp);
  feedback_.q=q; feedback_.qdot=v; feedback_.joint_stamp=stamp; joint_receive_stamp_=now;
}

void GimbalrotorPitchOnlyPerchingNavigator::request(const std::string& action)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  const auto s=sample(); const auto generation=core_.generation;
  if(action=="confirm_detached_zero") core_.armZero(s);
  else if(action=="pitch_rezero") core_.rezero(s);
  else if(action=="freeze") core_.freeze(s);
  else if(action=="unfreeze") core_.unfreeze(s);
  else if(action=="prepare_unperch") core_.prepare(s);
  else if(action=="confirm_released") core_.released(s);
  else if(action=="recover") core_.recover(s);
  if(core_.generation!=generation&&core_.ownsPose()) commitLock();
  diagnostics();
}
void GimbalrotorPitchOnlyPerchingNavigator::selectPerchingMode(perching_geometry::Mode mode,bool enable)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  // An optional pre-contact zero hold can exist while perching is disabled.
  if(mode==perching_geometry::Mode::NORMAL&&!enable) core_.zero_hold_armed=false;
  GimbalrotorPerchingNavigator::selectPerchingMode(mode,enable);
}
void GimbalrotorPitchOnlyPerchingNavigator::resetPerchingLock()
{
  core_.reset();
  GimbalrotorPerchingNavigator::resetPerchingLock();
}
bool GimbalrotorPitchOnlyPerchingNavigator::tryLockPerching(const std::string&)
{
  if(!core_.enable(sample())) return false;
  commitLock(); return true;
}
void GimbalrotorPitchOnlyPerchingNavigator::relockCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(core_.relock(sample())) commitLock();
  diagnostics();
}
void GimbalrotorPitchOnlyPerchingNavigator::applyManualPitchDelta(double delta)
{ std::lock_guard<std::recursive_mutex> l(perchingStateMutex()); core_.cut(delta,sample()); }
void GimbalrotorPitchOnlyPerchingNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& m)
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!core_.ownsPose()) GimbalrotorNavigator::naviCallback(m);
}
void GimbalrotorPitchOnlyPerchingNavigator::commitLock()
{
  const auto p=core_.target*core_.cog;
  double x,y,z,w; core_.target.M.GetQuaternion(x,y,z,w);
  const tf::Quaternion body(x,y,z,w);
  const auto cog_orientation=body*getCommandedBaselinkRotationRelativeToCog().inverse();
  commitFixedContactLock(tf::Vector3(p.x(),p.y(),p.z()),cog_orientation,
    tf::Vector3(core_.pivot.x(),core_.pivot.y(),core_.pivot.z()));
}

bool GimbalrotorPitchOnlyPerchingNavigator::commandPose()
{
  if(!finite(core_.target)||!finite(core_.cog)) return false;
  double x,y,z,w; core_.target.M.GetQuaternion(x,y,z,w);
  setBaselinkRotationTargetRelativeToCog(tf::Quaternion(0,0,0,1));
  const auto relative=getCommandedBaselinkRotationRelativeToCog();
  if(!std::isfinite(relative.length2())||relative.length2()<1e-12) return false;
  auto orientation=tf::Quaternion(x,y,z,w)*relative.inverse(); orientation.normalize();
  double r,p,yaw; tf::Matrix3x3(orientation).getRPY(r,p,yaw);
  const auto position=core_.target*core_.cog;
  setXyControlMode(POS_CONTROL_MODE);
  setTargetPos(tf::Vector3(position.x(),position.y(),position.z()));
  setTargetVel(0,0,0); setTargetAcc(0,0,0); setTargetRPY(tf::Vector3(r,p,yaw));
  setTargetOmega(0,0,0); setTargetAngAcc(0,0,0);
  return true;
}
void GimbalrotorPitchOnlyPerchingNavigator::applyActivePerchingTarget()
{
  if(core_.ownsPose() && !commandPose()) core_.fault("controller target invalid");
  if(!model_valid_||(!core_.ownsPose()&&!core_.zero_hold_armed)) return;
  sensor_msgs::JointState cmd; cmd.header.stamp=ros::Time::now(); cmd.name={joint_name_}; cmd.position={core_.q_target};
  joints_pub_.publish(cmd); // Joint positions are model radians; Servo.yaml performs motor conversion.
}
void GimbalrotorPitchOnlyPerchingNavigator::update()
{
  std::lock_guard<std::recursive_mutex> l(perchingStateMutex());
  if(!model_resolution_attempted_&&robot_model_&&robot_model_->initialized()) {
    model_resolution_attempted_=true; model_valid_=resolveModel();
    if(!model_valid_) ROS_ERROR("[PitchOnly] Unable to resolve pitch mechanism/parameters.");
  }
  GimbalrotorNavigator::update();
  // Match multilink: leaving HOVER disables perching instead of retaining authority.
  if(navi_state_!=HOVER_STATE) {
    if(perchingMode()!=perching_geometry::Mode::DISABLED)
      GimbalrotorPerchingNavigator::selectPerchingMode(perching_geometry::Mode::NORMAL,false);
    core_.zero_hold_armed=false;
    last_update_=ros::Time(); diagnostics(); return;
  }
  if(perchingMode()==perching_geometry::Mode::NORMAL&&!core_.ownsPose()) tryLockPerching("active update");
  const auto now=ros::Time::now();
  const double dt=last_update_.isZero()?0.025:(now-last_update_).toSec(); last_update_=now;
  core_.tick(sample(),dt);
  applyActivePerchingTarget(); diagnostics();
}
void GimbalrotorPitchOnlyPerchingNavigator::diagnostics()
{
  const auto s=sample();
  auto boolean=[this](const std::string& key,bool v) { std_msgs::Bool m; m.data=v; bool_pubs_.at(key).publish(m); };
  auto number=[this](const std::string& key,double v) { std_msgs::Float64 m; m.data=v; number_pubs_.at(key).publish(m); };
  const bool valid=core_.feedback(s);
  boolean("model_valid",model_valid_); boolean("feedback_valid",valid);
  boolean("contact_valid",core_.attached()&&valid);
  boolean("rezero_active",core_.stage==Stage::PITCH_REZERO_ACTIVE);
  boolean("rezero_ready",core_.rezeroReady(s));
  boolean("rezero_failed",core_.stage==Stage::FAULT_HOLD&&!core_.rezero_completed);
  boolean("servo_frozen",core_.stage==Stage::PASSIVE_CUTTING);
  boolean("cutting_enabled",core_.stage==Stage::PASSIVE_CUTTING&&valid);
  number("servo_measured",s.q); number("servo_target_nominal",core_.q_target);
  number("servo_hold_error",s.q-core_.q_target); number("body_pitch",pitch(s.body));
  number("body_pitch_rate",s.pitch_rate);
  number("passive_alpha_cmd",core_.alpha); number("passive_alpha_measured",core_.ownsPose()?core_.measuredAlpha(s):unknown);
  number("cut_alpha_min",core_.alpha_min); number("cut_alpha_max",core_.alpha_max); number("lock_generation",core_.generation);
  KDL::Frame bc; KDL::Vector g;
  const double error=core_.ownsPose()&&modelAt(s.q,bc,g)?(s.body*bc*core_.config.pivot_offset-core_.pivot).Norm():unknown;
  number("pivot_error",error);
  const char* feedback_error=core_.feedbackError(s);
  // Preserve the 15-field snapshot layout for RViz; old saw-feedback slot is reserved.
  std::ostringstream out; out<<std::setprecision(17)<<name(core_.stage)<<"|"<<core_.generation<<"|"<<s.now<<"|"
    <<s.q<<"|"<<core_.q_target<<"|"<<pitch(s.body)<<"|"<<core_.alpha<<"|"<<core_.measuredAlpha(s)<<"|"
    <<core_.alpha_min<<"|"<<core_.alpha_max<<"|"<<valid<<"|0|"<<error<<"|"<<core_.zero_hold_armed<<"|"
    <<(feedback_error?feedback_error:core_.status);
  std_msgs::String state; state.data=out.str(); state_pub_.publish(state);
  if(core_.ownsPose()) {
    geometry_msgs::PoseStamped pose; pose.header.stamp=ros::Time::now(); pose.header.frame_id=world_frame_;
    pose.pose.position.x=core_.target.p.x(); pose.pose.position.y=core_.target.p.y(); pose.pose.position.z=core_.target.p.z();
    core_.target.M.GetQuaternion(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    pose_pub_.publish(pose);
    geometry_msgs::PointStamped pivot; pivot.header=pose.header;
    pivot.point.x=core_.pivot.x(); pivot.point.y=core_.pivot.y(); pivot.point.z=core_.pivot.z(); pivot_pub_.publish(pivot);
  }
}
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorPitchOnlyPerchingNavigator,aerial_robot_navigation::BaseNavigator);
