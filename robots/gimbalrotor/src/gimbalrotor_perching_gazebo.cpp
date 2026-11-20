#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/String.h>
#include <cmath>
#include <urdf/model.h>

namespace gazebo
{
// An ideal passive contact fixture for Gazebo, not a contact detector. A
// successful navigator lock attaches the configured hinge to the world. The
// body and active joints joints remain dynamic; no trajectory is imposed by this plugin.
class PerchingContact : public ModelPlugin
{
public:
  ~PerchingContact() override
  {
    update_.reset();
    if(hinge_) { hinge_->Detach(); hinge_->Fini(); }
    if(nh_) nh_->shutdown();
    queue_.clear();
  }

  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override
  {
    if(!ros::isInitialized()) { gzerr << "Perching contact requires gazebo_ros\n"; return; }
    model_=model;
    const auto ns=sdf->Get<std::string>("robotNamespace");
    nh_.reset(new ros::NodeHandle(ns)); nh_->setCallbackQueue(&queue_);
    ros::NodeHandle pn(*nh_,"navigation/perching");
    std::string prefix, description;
    pn.param<std::string>("debug_prefix",prefix,"perching");
    pn.param<std::string>("world_frame",world_frame_,"world");
    if(!world_frame_.empty()&&world_frame_.front()=='/') world_frame_.erase(0,1);
    urdf::Model description_model;
    if(!nh_->getParam("robot_description", description) || !description_model.initString(description)) return;
    auto marker = description_model.getLink("perching_pivot");
    ignition::math::Pose3d surviving_marker = ignition::math::Pose3d::Zero;
    // Gazebo may lump the massless marker and any fixed ancestors into a link.
    while(marker && !(link_ = model_->GetLink(marker->name)))
    {
      const auto joint = marker->parent_joint;
      if(!joint || joint->type != urdf::Joint::FIXED) break;
      const auto& origin = joint->parent_to_joint_origin_transform;
      const ignition::math::Pose3d parent_child(
          ignition::math::Vector3d(origin.position.x, origin.position.y, origin.position.z),
          ignition::math::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z));
      surviving_marker = parent_child * surviving_marker;
      marker = description_model.getLink(joint->parent_link_name);
    }
    std::vector<double> lower{-M_PI}, upper{M_PI};
    pn.getParam("position_lower",lower);
    pn.getParam("position_upper",upper);
    if(!link_ || lower.empty() || upper.size()!=lower.size() ||
       !std::isfinite(lower[0]) || !std::isfinite(upper[0]) || lower[0]>=0 || upper[0]<=0)
    { ROS_ERROR("[Perching Gazebo] Invalid URDF marker or hinge limits"); return; }
    offset_ = surviving_marker.Pos();
    pivot_rotation_ = surviving_marker.Rot();
    axis_ = ignition::math::Vector3d(0, 1, 0);
    lower_=lower[0]; upper_=upper[0];
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
    if(!attached_requested_ || !pending_) return;
    if(hinge_) { hinge_->Detach(); hinge_->Fini(); hinge_.reset(); }
    pending_=false;
    const auto anchor=link_->WorldPose().Pos()+link_->WorldPose().Rot().RotateVector(offset_);
    if(anchor.Distance(requested_)>tolerance_)
    { publish("rejected: measured lock does not match physical hinge"); return; }
    hinge_=model_->GetWorld()->Physics()->CreateJoint("revolute",model_);
    if(!hinge_) { publish("rejected: physics engine cannot create hinge"); return; }
    hinge_->SetName("perching_passive_contact");
    hinge_->Load(physics::LinkPtr(),link_,ignition::math::Pose3d(offset_,pivot_rotation_));
    hinge_->Init();
    hinge_->SetAxis(0, axis_); // Gazebo expects the axis in the joint frame.
    hinge_->SetLowerLimit(0,lower_); hinge_->SetUpperLimit(0,upper_);
    publish("attached: passive world hinge; active joints and body remain dynamic");
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
GZ_REGISTER_MODEL_PLUGIN(PerchingContact)
}
