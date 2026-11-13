#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <cmath>

namespace gazebo
{
// An ideal passive contact fixture for Gazebo, not a contact detector. A
// successful navigator lock attaches the configured hinge to the world. The
// body and roll/yaw joints remain dynamic; no trajectory is imposed by this plugin.
class RollYawPerchingContact : public ModelPlugin
{
public:
  ~RollYawPerchingContact() override
  {
    update_.reset();
    if(hinge_) { hinge_->Detach(); hinge_->Fini(); }
    if(nh_) nh_->shutdown();
    queue_.clear();
  }

  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override
  {
    if(!ros::isInitialized()) { gzerr << "RollYaw contact requires gazebo_ros\n"; return; }
    model_=model;
    const auto ns=sdf->Get<std::string>("robotNamespace");
    nh_.reset(new ros::NodeHandle(ns)); nh_->setCallbackQueue(&queue_);
    ros::NodeHandle pn(*nh_,"navigation/roll_yaw_perching");
    std::string contact,prefix;
    pn.param<std::string>("contact_link",contact,"roll_hand_link");
    pn.param<std::string>("debug_prefix",prefix,"perching/roll_yaw");
    pn.param<std::string>("world_frame",world_frame_,"world");
    if(!world_frame_.empty()&&world_frame_.front()=='/') world_frame_.erase(0,1);
    link_=model_->GetLink(contact);
    ignition::math::Vector3d rpy,lower,upper;
    if(!link_||!read(pn,"pivot_offset",offset_)||!read(pn,"pivot_rpy",rpy)||
       !read(pn,"hinge_axis",axis_)||std::abs(axis_.Length()-1)>1e-7||
       !read(pn,"position_lower",lower)||!read(pn,"position_upper",upper)||
       lower.X()>=0||upper.X()<=0)
    { ROS_ERROR("[RollYaw Gazebo] Invalid contact link/hinge configuration"); return; }
    pivot_rotation_=ignition::math::Quaterniond(rpy);
    lower_=lower.X(); upper_=upper.X();
    pn.param("pivot_tolerance",tolerance_,0.01);
    if(!std::isfinite(tolerance_)||tolerance_<=0) return;
    status_pub_=nh_->advertise<std_msgs::String>(prefix+"/contact",1,true);
    pivot_sub_=nh_->subscribe<geometry_msgs::PointStamped>("perching/locked_pivot",1,
      [this](const geometry_msgs::PointStampedConstPtr& p) {
        std::string frame=p->header.frame_id;
        if(!frame.empty()&&frame.front()=='/') frame.erase(0,1);
        if(frame!=world_frame_||p->header.stamp<=last_lock_) return;
        requested_={p->point.x,p->point.y,p->point.z};
        if(!requested_.IsFinite()) return;
        last_lock_=p->header.stamp; pending_=true;
      });
    state_sub_=nh_->subscribe<std_msgs::String>(prefix+"/state",1,
      [this](const std_msgs::StringConstPtr& s) {
        attached_requested_=s->data.rfind("FREE_FLIGHT:",0)!=0;
      });
    update_=event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo&) { update(); });
    publish("detached: ideal passive contact available on successful perching lock");
  }

private:
  static bool read(ros::NodeHandle& nh,const std::string& key,ignition::math::Vector3d& v)
  {
    std::vector<double> data;
    if(!nh.getParam(key,data)||data.size()!=3) return false;
    v={data[0],data[1],data[2]}; return v.IsFinite();
  }
  void publish(const std::string& text)
  { std_msgs::String s; s.data=text; status_pub_.publish(s); }
  void update()
  {
    // Process ROS messages and manipulate joints only on the physics thread.
    queue_.callAvailable(ros::WallDuration(0));
    if(!attached_requested_&&hinge_)
    {
      hinge_->Detach(); hinge_->Fini(); hinge_.reset(); pending_=false;
      publish("detached: perching disabled");
    }
    if(!attached_requested_||hinge_||!pending_) return;
    pending_=false;
    const auto anchor=link_->WorldPose().Pos()+link_->WorldPose().Rot().RotateVector(offset_);
    if(anchor.Distance(requested_)>tolerance_)
    { publish("rejected: measured lock does not match physical hinge"); return; }
    hinge_=model_->GetWorld()->Physics()->CreateJoint("revolute",model_);
    if(!hinge_) { publish("rejected: physics engine cannot create hinge"); return; }
    hinge_->SetName("roll_yaw_passive_contact");
    hinge_->Load(physics::LinkPtr(),link_,ignition::math::Pose3d(offset_,pivot_rotation_));
    hinge_->Init();
    hinge_->SetAxis(0,axis_);
    hinge_->SetLowerLimit(0,lower_); hinge_->SetUpperLimit(0,upper_);
    publish("attached: passive world hinge; roll/yaw and body remain dynamic");
  }
  ros::CallbackQueue queue_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::Subscriber pivot_sub_,state_sub_;
  ros::Publisher status_pub_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr hinge_;
  event::ConnectionPtr update_;
  ignition::math::Vector3d offset_,axis_,requested_;
  ignition::math::Quaterniond pivot_rotation_;
  ros::Time last_lock_;
  std::string world_frame_;
  double lower_=0,upper_=0,tolerance_=0.01;
  bool pending_=false,attached_requested_=false;
};
GZ_REGISTER_MODEL_PLUGIN(RollYawPerchingContact)
}
