#include <sampling_kinematics_plugin/external_axis_sampling_kinematics_plugin.h>

// URDF, SRDF
#include <srdfdom/model.h>
#include <urdf_model/model.h>
#include <moveit/rdf_loader/rdf_loader.h>

// Eigen
#include <tf2_eigen/tf2_eigen.h>

static const std::string LOG_NAMESPACE = "sampling_kinematics_plugin";

namespace
{
double distance(const std::vector<double> &a,
                const std::vector<double> &b,
                const std::vector<double> &weights)
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += weights.at(i) * std::abs(b.at(i) - a.at(i));
  return cost;
}

} // namespace anonymous

namespace sampling_kinematics_plugin
{
using kinematics::KinematicsResult;

ExternalAxisSamplingKinematicsPlugin::ExternalAxisSamplingKinematicsPlugin()
  : kinematics::KinematicsBase ()
  , loader_("moveit_core", "kinematics::KinematicsBase")
{
}

bool ExternalAxisSamplingKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                                      const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                                      double search_discretization)
{
  ROS_INFO_STREAM_NAMED(LOG_NAMESPACE, "ExternalAxisSamplingKinematicsPlugin initializing");

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  robot_state_.reset(new robot_state::RobotState(robot_model_));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getActiveJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getActiveJointModelNames()[i]);
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if (!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED(LOG_NAMESPACE, "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
                      group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Get the robot planning group (planning group for only the robot joints, unlike the input group which includes external axis joints)
  const std::string robot_group_name_param = "robot_group";
  std::string robot_group_name;
  if (!lookupParam<std::string>(robot_group_name_param, robot_group_name, ""))
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Failed to get " << robot_group_name_param << "' parameter");
    return false;
  }
  auto robot_joint_model_group = robot_model_->getJointModelGroup(robot_group_name);
  if (!robot_joint_model_group)
    return false;

  robot_base_frame_
    = robot_joint_model_group->getActiveJointModels().front()->getParentLinkModel()->getName();

  // Perform checks on the joints in both specified planning groups
  {
    std::vector<std::string> joint_names = joint_model_group_->getActiveJointModelNames();
    std::vector<std::string> robot_joint_names = robot_joint_model_group
                                                          ->getActiveJointModelNames();

    // Ensure all the joints in the robot planning group exist in this planning group
    std::sort(joint_names.begin(), joint_names.end());
    std::sort(robot_joint_names.begin(), robot_joint_names.end());
    if (!std::includes(joint_names.begin(),
                       joint_names.end(),
                       robot_joint_names.begin(),
                       robot_joint_names.end()))
    {
      ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE,
                             "Planning group '"
                               << group_name
                               << "' does not contain all joints present in robot planning group '"
                               << robot_group_name << "'");
      return false;
    }

    // Get the difference between the input planning group and the specified robot group, which represents the free joint(s)
    std::vector<std::string> active_joint_diff;
    std::set_difference(joint_names.begin(),
                        joint_names.end(),
                        robot_joint_names.begin(),
                        robot_joint_names.end(),
                        std::inserter(active_joint_diff, active_joint_diff.begin()));

    if (active_joint_diff.size() != 1)
    {
      ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "The ExternalAxisSamplingKinematicsPlugin requires only 1 free joint (" << active_joint_diff.size() << " detected)");
      return false;
    }
  }

  // Get the name of the plugin to laod
  const std::string robot_solver_type_param = robot_group_name + "/kinematics_solver";
  std::string robot_solver_type;
  if (!lookupParam<std::string>(robot_solver_type_param, robot_solver_type, ""))
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Failed to get '" << robot_solver_type_param << "'");
    return false;
  }

  try
  {
    solver_ = loader_.createInstance(robot_solver_type);
  }
  catch (const pluginlib::ClassLoaderException& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Failed to load child IK solver plugin");
    return false;
  }

  // Initialize the plugin with the group that represents the manipulator (the input joint group should have additional joints)
  if(!solver_->initialize(robot_model, robot_group_name, robot_base_frame_, tip_frames, search_discretization))
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Failed to initialize robot kinematics solver plugin");
    return false;
  }

  // Get the cost weights
  const std::string weight_param = "weights";
  const std::size_t n = joint_model_group_->getActiveJointModelNames().size();
  std::vector<double> tmp_weights;
  if (!lookupParam(weight_param, tmp_weights, std::vector<double>(n, 1.0)))
  {
    ROS_WARN_STREAM_NAMED(LOG_NAMESPACE, "Using default weight vector (" << n << " x 1.0)");
  }
  else if (tmp_weights.size() != n)
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE,
                           "Incorrect number of weights (" << tmp_weights.size() << " provided, "
                                                           << n << " expected)");
    return false;
  }
  cost_weights_ = std::move(tmp_weights);

  return true;
}

bool ExternalAxisSamplingKinematicsPlugin::getFreeJointLimits(double& min, double& max) const
{
  try
  {
    auto bounds_vec = joint_model_group_->getActiveJointModelsBounds();
    if (bounds_vec.at(0)->at(0).position_bounded_)
    {
      max = bounds_vec.at(0)->at(0).max_position_;
      min = bounds_vec.at(0)->at(0).min_position_;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Free joint is not position bounded");
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, ex.what());
    return false;
  }

  return true;
}

Eigen::Isometry3d ExternalAxisSamplingKinematicsPlugin::getRobotBaseTransform(const double joint_position) const
{
  robot_state_->setToDefaultValues();
  std::vector<double> joint_vals;
  robot_state_->copyJointGroupPositions(group_name_, joint_vals);
  joint_vals[0] = joint_position;
  robot_state_->setJointGroupPositions(group_name_, joint_vals);

  // Lookup the transform to the robot base link
  return Eigen::Isometry3d(robot_state_->getFrameTransform(robot_base_frame_).matrix());
}

geometry_msgs::Pose ExternalAxisSamplingKinematicsPlugin::getUpdatedPose(const geometry_msgs::Pose& ik_pose,
                                                                       const double joint_position) const
{
  // Get the transform to the robot base
  Eigen::Isometry3d root_to_robot_base = getRobotBaseTransform(joint_position);

  // Get the transform to the kinematic base link
  Eigen::Isometry3d root_to_kin_base(robot_state_->getFrameTransform(base_frame_).matrix());

  // Convert the target pose
  Eigen::Isometry3d pose;
  tf2::fromMsg(ik_pose, pose);

  // Get the pose relative to the robot base frame (it comes in relative to the kinematic base frame, i.e. external axis base)
  Eigen::Isometry3d new_pose = (root_to_kin_base.inverse() * root_to_robot_base).inverse() * pose;

  return tf2::toMsg(new_pose);
}

bool ExternalAxisSamplingKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool ExternalAxisSamplingKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool ExternalAxisSamplingKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool ExternalAxisSamplingKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool ExternalAxisSamplingKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool ExternalAxisSamplingKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                       const std::vector<double>& ik_seed_state, double timeout,
                                                       std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                       moveit_msgs::MoveItErrorCodes& error_code,
                                                       const std::vector<double>& consistency_limits,
                                                       const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool ExternalAxisSamplingKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                          const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                          const std::vector<double>& /*consistency_limits*/,
                                                          std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                          moveit_msgs::MoveItErrorCodes& error_code,
                                                          const kinematics::KinematicsQueryOptions& /*options*/,
                                                          const robot_state::RobotState* /*reference_state*/) const
{
  // Pass through to the getPositionIK method to get all IK solutions in order of lowest cost
  std::vector<std::vector<double>> sols;
  KinematicsResult tmp_result;
  if (getPositionIK(ik_poses, ik_seed_state, sols, tmp_result, kinematics::KinematicsQueryOptions()))
  {
    // Check the full solution using the provided callback
    if (solution_callback)
    {
      // Iterate through the solutions, and return the first solution that passes the callback
      for (const auto &sol : sols)
      {
        moveit_msgs::MoveItErrorCodes tmp_error;
        solution_callback(ik_poses.front(), sol, tmp_error);
        if (tmp_error.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
          solution = sol;
          error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
          return true;
        }
      }
      ROS_DEBUG_STREAM_NAMED(LOG_NAMESPACE, "IK solutions passed the validity callback");
    }
    else
    {
      // Return the first solution if no validity callback was defined
      solution = sols.front();
      error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
      return true;
    }
  }

  // IK failed
  error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  return false;
}

bool ExternalAxisSamplingKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                         const std::vector<double>& ik_seed_state,
                                                         std::vector<std::vector<double>>& solutions, KinematicsResult& /*result*/,
                                                         const kinematics::KinematicsQueryOptions& options) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "You can only get all solutions for a single pose.");
    return false;
  }

  // Get the limits of the free joint
  double min, max;
  if (!getFreeJointLimits(min, max))
    return false;

  // Get the number of discrete iterations that will be tested
  std::size_t count = static_cast<std::size_t>(std::floor((max - min) / search_discretization_));

  // Create an IK seed state without the additional joints
  std::vector<double> seed_state(ik_seed_state.begin() + 1, ik_seed_state.end());

  std::vector<SolWithCost> all_sols;
  // Pre-allocate memory for the solutions.
  // Take a reasonable guess at the total number of solutions there might be (8 per discretization for spherical wrist industrial robots)
  all_sols.reserve(count * 8);

  for (std::size_t i = 0; i < count; ++i)
  {
    double joint_position = min + (i * search_discretization_);
    geometry_msgs::Pose new_ik_pose = getUpdatedPose(ik_poses.front(), joint_position);

    std::vector<std::vector<double>> sols;
    KinematicsResult tmp_result;
    if (solver_->getPositionIK({new_ik_pose}, seed_state, sols, tmp_result, options))
    {
      for (auto &&s : sols)
      {
        SolWithCost sol;
        sol.value = s;

        // Add the external axis joint value to the beginning of the solution
        sol.value.insert(sol.value.begin(), joint_position);

        // Calculate the cost of the solution
        sol.cost = distance(sol.value, ik_seed_state, cost_weights_);
        all_sols.push_back(std::move(sol));
      }
    }
  }

  if (all_sols.empty())
  {
    ROS_DEBUG_STREAM_NAMED(LOG_NAMESPACE, "No IK solutions found");
    return false;
  }

  // Sort the solutions by lowest joint distance from the solution
  std::sort(all_sols.begin(), all_sols.end());

  solutions.clear();
  solutions.reserve(all_sols.size());
  for (const auto& sol : all_sols)
  {
    solutions.push_back(sol.value);
  }

  return true;
}

bool ExternalAxisSamplingKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                                         const std::vector<double>& joint_angles,
                                                         std::vector<geometry_msgs::Pose>& poses) const
{
  std::vector<geometry_msgs::Pose> robot_poses;
  std::vector<double> robot_joints(joint_angles.begin() + 1, joint_angles.end());
  if (!solver_->getPositionFK(link_names, robot_joints, robot_poses))
  {
    return false;
  }

  Eigen::Isometry3d root_to_robot_base = getRobotBaseTransform(joint_angles[0]);

  poses.clear();
  poses.reserve(robot_poses.size());

  for(const auto& robot_pose : robot_poses)
  {
    Eigen::Isometry3d robot_base_to_tip;
    tf2::fromMsg(robot_pose, robot_base_to_tip);
    Eigen::Isometry3d root_to_tip = root_to_robot_base * robot_base_to_tip;
    poses.push_back(tf2::toMsg(root_to_tip));
  }

  return true;
}

}  // namespace sampling_kinematics_plugin

// register as a KinematicsBase implementation
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(sampling_kinematics_plugin::ExternalAxisSamplingKinematicsPlugin, kinematics::KinematicsBase)
