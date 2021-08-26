#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <pluginlib/class_loader.h>

namespace sampling_kinematics_plugin
{
using kinematics::KinematicsResult;

struct SolWithCost
{
  std::vector<double> value;
  double cost;

  bool operator<(const SolWithCost& a) const
  {
    return cost < a.cost;
  }
};

/**
 * @brief Kinematics plugin implementation for robots mounted on external axes
 */
class ExternalAxisSamplingKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
   *  @brief Default constructor
   */
  ExternalAxisSamplingKinematicsPlugin();

  virtual bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const override;

  virtual bool initialize(const moveit::core::RobotModel &robot_model,
                          const std::string& group_name,
                          const std::string& base_frame,
                          const std::vector<std::string>& tip_frames,
                          double search_discretization) override;

  virtual bool
  getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                std::vector<std::vector<double>>& solutions, KinematicsResult& result,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const inline std::vector<std::string>& getJointNames() const override
  {
    return ik_group_info_.joint_names;
  }

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const inline std::vector<std::string>& getLinkNames() const override
  {
    return ik_group_info_.link_names;
  }

protected:
  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, const IKCallbackFn& solution_callback,
                   moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                   double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                   const robot_state::RobotState* reference_state = nullptr) const override;

  bool getFreeJointLimits(double& min, double& max) const;

  /**
   * @brief Calculates the transform to the robot base frame at the input external axis joint position
   * @param joint_position - external axis joint position
   * @return
   */
  Eigen::Isometry3d getRobotBaseTransform(const double joint_position) const;

  /**
   * @brief Calculates the target IK pose in the robot base frame at the input external axis joint position
   * @param pose - target IK pose
   * @param joint_position - external axis joint position
   * @return
   */
  geometry_msgs::Pose getUpdatedPose(const geometry_msgs::Pose& pose,
                                     const double joint_position) const;

  std::string robot_base_frame_;                            /** @brief Name of the robot base frame */
  moveit_msgs::KinematicSolverInfo ik_group_info_;          /** @brief Solver information */
  robot_state::RobotStatePtr robot_state_;                  /** @brief MoveIt robot state */
  const robot_model::JointModelGroup* joint_model_group_;   /** @brief MoveIt joint model group */
  pluginlib::ClassLoader<KinematicsBase> loader_;           /** @brief Kinematics plugin loader for the robot kinematics plugin */
  boost::shared_ptr<kinematics::KinematicsBase> solver_;                    /** @brief Kinematics solver plugin for the robot */
  std::vector<double> cost_weights_;                        /** @brief Weights to be applied to each joint when evaluating the cost of a joint pose solution */
};
}  // namespace sampling_kinematics_plugin
