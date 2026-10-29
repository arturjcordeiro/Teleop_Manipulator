// Pruning a mobile base down to the manipulator: no ROS, no robot.
#include <gtest/gtest.h>

#include <string>

#include "teleop_manipulators/core/diff_ik.hpp"

using teleop::DiffIK;

namespace {

// base -> wheel (side branch), base -> lift -> shoulder -> elbow -> tool0
const char* kMobileArmUrdf = R"(
<robot name="mobile_arm">
  <link name="world"/>
  <joint name="world_fixed" type="fixed">
    <parent link="world"/><child link="base"/>
  </joint>
  <link name="base"/>
  <joint name="wheel" type="continuous">
    <parent link="base"/><child link="wheel_link"/>
    <axis xyz="0 1 0"/>
  </joint>
  <link name="wheel_link"/>
  <joint name="lift" type="prismatic">
    <parent link="base"/><child link="lift_link"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="1" effort="1" velocity="0.1"/>
  </joint>
  <link name="lift_link"/>
  <joint name="shoulder" type="revolute">
    <parent link="lift_link"/><child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <link name="link1"/>
  <joint name="elbow" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
  <link name="link2"/>
  <joint name="tcp_joint" type="fixed">
    <parent link="link2"/><child link="tool0"/>
  </joint>
  <link name="tool0"/>
</robot>
)";

DiffIK::Params makeParams(const std::string& base_frame) {
  DiffIK::Params p;
  p.urdf_xml = kMobileArmUrdf;
  p.tcp_frame = "tool0";
  p.base_frame = base_frame;
  return p;
}

}  // namespace

TEST(DiffIK, LocksSideBranchesWhenNoBaseFrameGiven) {
  DiffIK ik(makeParams(""));
  EXPECT_EQ(ik.dof(), 3u);  // lift + shoulder + elbow, wheel is locked
  EXPECT_EQ(ik.lowerLimits().size(), 3);
  EXPECT_EQ(ik.upperLimits().size(), 3);
}

TEST(DiffIK, BaseFrameDropsMobileBaseJoints) {
  DiffIK ik(makeParams("lift_link"));
  EXPECT_EQ(ik.dof(), 2u);  // shoulder + elbow only
  EXPECT_EQ(ik.lowerLimits().size(), 2);
  EXPECT_EQ(ik.velocityLimits().size(), 2);
}

TEST(DiffIK, UnknownBaseFrameThrows) {
  EXPECT_THROW(DiffIK(makeParams("nope")), std::invalid_argument);
}
