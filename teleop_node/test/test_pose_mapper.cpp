// Unit tests for the core: no ROS, no robot.
#include <gtest/gtest.h>

#include <cmath>

#include "teleop_manipulators/core/pose_mapper.hpp"

using teleop::PoseMapper;

TEST(PoseMapper, NoTargetWhenNotEngaged) {
  PoseMapper mapper({});
  EXPECT_FALSE(mapper.target(Eigen::Isometry3d::Identity()).has_value());
}

TEST(PoseMapper, TranslationIsRotatedAndScaled) {
  PoseMapper::Params p;
  p.R_base_optitrack = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.translation_scale = 0.5;
  PoseMapper mapper(p);

  Eigen::Isometry3d tcp0 = Eigen::Isometry3d::Identity();
  tcp0.translation() = Eigen::Vector3d(0.4, 0.0, 0.3);
  mapper.engage(Eigen::Isometry3d::Identity(), tcp0);

  Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
  tool.translation() = Eigen::Vector3d(0.2, 0.0, 0.0);  // +X in OptiTrack

  const auto target = mapper.target(tool);
  ASSERT_TRUE(target.has_value());
  // +X OptiTrack -> +Y base, scaled by 0.5
  EXPECT_TRUE(target->translation().isApprox(Eigen::Vector3d(0.4, 0.1, 0.3), 1e-9));
}

TEST(PoseMapper, NoMotionGivesInitialTcp) {
  PoseMapper mapper({});
  Eigen::Isometry3d tcp0 = Eigen::Isometry3d::Identity();
  tcp0.translation() = Eigen::Vector3d(0.1, 0.2, 0.3);
  Eigen::Isometry3d tool0 = Eigen::Isometry3d::Identity();
  tool0.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  mapper.engage(tool0, tcp0);
  EXPECT_TRUE(mapper.target(tool0)->isApprox(tcp0, 1e-9));
}
