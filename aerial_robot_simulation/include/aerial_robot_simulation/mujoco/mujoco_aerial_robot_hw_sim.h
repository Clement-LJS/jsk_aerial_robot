#pragma once

#include <aerial_robot_simulation/mujoco/mujoco_cutting_force_model.h>
#include <aerial_robot_simulation/mujoco/mujoco_spinal_interface.h>
#include <aerial_robot_simulation/noise_model.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mujoco_ros_control/mujoco_default_robot_hw_sim.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>

namespace mujoco_ros_control
{
  class AerialRobotHWSim : public mujoco_ros_control::DefaultRobotHWSim
  {
  public:
    AerialRobotHWSim() {};
    ~AerialRobotHWSim() {}

    bool init(const std::string& robot_namespace,
              ros::NodeHandle model_nh,
              mjModel* mujoco_model,
              mjData* mujoco_data
              ) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

  protected:
    struct CuttingState
    {
      bool enabled = false;
      bool model_ready = false;
      bool valid = false;
      bool contact = false;
      bool completed = false;
      bool perching_enabled = false;
      bool cutting_active = false;
      bool cutting_active_prev = false;
      bool has_locked_pivot = false;
      bool reset_requested = false;
      bool has_reference_radius = false;
      int base_body_id = -1;
      int saw_body_id = -1;
      int contact_site_id = -1;
      int constraint_1_id = -1;
      int constraint_2_id = -1;
      double cutting_direction_sign = 1.0;
      double locked_pivot_match_tolerance = 0.002;
      double last_signed_travel = 0.0;
      tf::Vector3 configured_pivot_world;
      tf::Vector3 pivot_axis_world;
      tf::Vector3 locked_pivot_world;
      tf::Vector3 reference_radius_world;
      aerial_robot_simulation::MujocoCuttingForceModel force_model;
    };

    hardware_interface::MujocoSpinalInterface spinal_interface_;

    std::vector<std::string> rotor_list_;
    ros::Publisher ground_truth_pub_;
    ros::Publisher mocap_pub_;
    double ground_truth_pub_rate_;
    double mocap_pub_rate_;
    double mocap_rot_noise_, mocap_pos_noise_;
    double ground_truth_pos_noise_, ground_truth_vel_noise_, ground_truth_rot_noise_, ground_truth_angular_noise_;
    double ground_truth_rot_drift_, ground_truth_vel_drift_, ground_truth_angular_drift_;
    double ground_truth_rot_drift_frequency_, ground_truth_vel_drift_frequency_, ground_truth_angular_drift_frequency_;
    double joint_state_pub_rate_ = 0.02;

    ros::Time last_ground_truth_time_;
    ros::Time last_mocap_time_;

    CuttingState cutting_state_;
    ros::Subscriber perching_enable_sub_;
    ros::Subscriber cutting_active_sub_;
    ros::Subscriber locked_pivot_sub_;
    ros::Subscriber cutting_reset_sub_;
    ros::Publisher cutting_model_ready_pub_;
    ros::Publisher cutting_valid_pub_;
    ros::Publisher cutting_wrench_cog_pub_;
    ros::Publisher cutting_wrench_pivot_pub_;
    ros::Publisher cutting_force_magnitude_pub_;
    ros::Publisher cutting_pivot_torque_y_pub_;
    ros::Publisher cutting_penetration_pub_;
    ros::Publisher cutting_penetration_velocity_pub_;
    ros::Publisher cutting_arc_angle_pub_;
    ros::Publisher cutting_layer_index_pub_;
    ros::Publisher cutting_contact_pub_;
    ros::Publisher cutting_completed_pub_;
    ros::Publisher cutting_locked_pivot_error_pub_;

    void perchingEnableCallback(const std_msgs::Bool::ConstPtr& msg);
    void cuttingActiveCallback(const std_msgs::Bool::ConstPtr& msg);
    void lockedPivotCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void cuttingResetCallback(const std_msgs::Empty::ConstPtr& msg);
    bool initCutting(ros::NodeHandle model_nh);
    void resetCuttingState();
    void publishCuttingTopics(const ros::Time& time,
                              const tf::Vector3& force_world,
                              const tf::Vector3& torque_cog_world,
                              const tf::Vector3& torque_pivot_world,
                              const aerial_robot_simulation::MujocoCuttingForceModel::Output& model_output,
                              double arc_angle,
                              double locked_pivot_error);
  };
}
