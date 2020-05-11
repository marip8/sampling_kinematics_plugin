#include <gtest/gtest.h>
#include <ros/ros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.h>
#include <tf2_eigen/tf2_eigen.h>

// Names to read values from the parameter server
const std::string GROUP_PARAM = "/group";
const std::string TIP_LINK_PARAM = "/tip_link";
const std::string ROOT_LINK_PARAM = "/root_link";
const std::string SOLVER_PARAM = "/robot_description_kinematics/rail_manipulator/kinematics_solver";
const std::string ROBOT_DESCRIPTION = "robot_description";

class TestPlugin : public ::testing::Test
{
  public:
  TestPlugin()
    : ::testing::Test()
    , loader_("moveit_core", "kinematics::KinematicsBase")
  {
  }

  std::vector<double> getTestJoints()
  {
    const std::size_t n = plugin_->getJointNames().size();
    std::vector<double> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
    {
      out.push_back((i + 1) * 0.1);
    }
    return out;
  }

  protected:
  void SetUp() override
  {
    ros::NodeHandle nh;
    std::string group_name, root_link, tip_link, solver;
    ASSERT_TRUE(nh.getParam(GROUP_PARAM, group_name));
    ASSERT_TRUE(nh.getParam(ROOT_LINK_PARAM, root_link));
    ASSERT_TRUE(nh.getParam(TIP_LINK_PARAM, tip_link));
    ASSERT_TRUE(nh.getParam(SOLVER_PARAM, solver));
    ASSERT_NO_THROW(plugin_.reset(loader_.createUnmanagedInstance(solver)));
    ASSERT_TRUE(plugin_->initialize(ROBOT_DESCRIPTION, group_name, root_link, tip_link, 0.1));
  }

  kinematics::KinematicsBasePtr plugin_;
  pluginlib::ClassLoader<kinematics::KinematicsBase> loader_;
};

TEST_F(TestPlugin, InitOk)
{
  ASSERT_FALSE(this->plugin_->getGroupName().empty());
}

TEST_F(TestPlugin, CompareIKAndFK)
{
  // find reachable pose
  const std::vector<double> joints = this->getTestJoints();
  geometry_msgs::Pose fk_pose;
  {
    std::vector<geometry_msgs::Pose> poses_out;
    ASSERT_TRUE(this->plugin_->getPositionFK(this->plugin_->getLinkNames(), joints, poses_out));
    ASSERT_GT(poses_out.size(), 0);
    fk_pose = poses_out.front();
  }

  // calculate all ik solutions for this pose
  std::vector<std::vector<double>> solutions;
  {
    kinematics::KinematicsResult result;
    kinematics::KinematicsQueryOptions options;
    ASSERT_TRUE(this->plugin_->getPositionIK({fk_pose}, joints, solutions, result, options));
  }

  // check if fk for all this solutions gives the same pose
  Eigen::Isometry3d actual, desired;
  tf2::fromMsg(fk_pose, desired);
  for (auto js : solutions)
  {
    std::vector<geometry_msgs::Pose> poses_out;
    ASSERT_TRUE(this->plugin_->getPositionFK(this->plugin_->getLinkNames(), js, poses_out));
    tf2::fromMsg(poses_out.front(), actual);
    ASSERT_TRUE(actual.isApprox(desired));
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sampling_kinematics_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
