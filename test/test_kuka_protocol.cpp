#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>

#include "teleop_manipulators/backends/kuka_protocol.hpp"

using namespace teleop::kuka;

namespace {

void putDouble(std::uint8_t *out, double value) {
  std::uint64_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  raw = swap64(raw);
  std::memcpy(out, &raw, sizeof(raw));
}

double readDouble(const std::uint8_t *in) { return getBe(in); }

} // namespace

TEST(KukaProtocol, IsBigEndian) {
  std::uint8_t buf[8];
  putBe(buf, 1.0);
  // IEEE-754 1.0 in big-endian is 3F F0 00 00 00 00 00 00.
  EXPECT_EQ(buf[0], 0x3F);
  EXPECT_EQ(buf[1], 0xF0);
  for (int i = 2; i < 8; ++i) {
    EXPECT_EQ(buf[i], 0x00);
  }
}

TEST(KukaProtocol, JointPacketLayout) {
  const std::array<double, kDof> q{1.5, -2.25, 0.0, 0.25, -0.5, 3.0, -1.0};
  std::uint8_t buf[kJointPacketSize];
  encodeJoint(q, 0.75, 0.5, buf);

  for (std::size_t i = 0; i < kDof; ++i) {
    EXPECT_DOUBLE_EQ(readDouble(buf + i * 8), q[i]);
  }
  EXPECT_DOUBLE_EQ(readDouble(buf + 56), 0.75);
  EXPECT_DOUBLE_EQ(readDouble(buf + 64), 0.5);
}

TEST(KukaProtocol, CartesianPacketLayout) {
  std::uint8_t buf[kCartesianPacketSize];
  encodeCartesian(400.0, -100.0, 250.0, 0.1, -0.2, 0.3, 200.0, 50.0, buf);

  EXPECT_DOUBLE_EQ(readDouble(buf + 0), 400.0);
  EXPECT_DOUBLE_EQ(readDouble(buf + 8), -100.0);
  EXPECT_DOUBLE_EQ(readDouble(buf + 16), 250.0);
  EXPECT_DOUBLE_EQ(readDouble(buf + 24), 0.1);
  EXPECT_DOUBLE_EQ(readDouble(buf + 32), -0.2);
  EXPECT_DOUBLE_EQ(readDouble(buf + 40), 0.3);
  EXPECT_DOUBLE_EQ(readDouble(buf + 48), 200.0);
  EXPECT_DOUBLE_EQ(readDouble(buf + 56), 50.0);
}

TEST(KukaProtocol, DecodeReply) {
  const std::array<double, kDof> q{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  const std::array<double, 3> force{1.0, 2.0, 3.0};
  const std::array<double, 3> torque{-1.0, -2.0, -3.0};

  std::uint8_t buf[kReplySize]{};
  buf[0] = static_cast<std::uint8_t>(Status::Reached);
  for (std::size_t i = 0; i < kDof; ++i) {
    putDouble(buf + 1 + i * 8, q[i]);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    putDouble(buf + 57 + i * 8, force[i]);
    putDouble(buf + 81 + i * 8, torque[i]);
  }

  Reply reply;
  ASSERT_TRUE(decodeReply(buf, kReplySize, reply));
  EXPECT_EQ(reply.status, Status::Reached);
  for (std::size_t i = 0; i < kDof; ++i) {
    EXPECT_DOUBLE_EQ(reply.q[i], q[i]);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_DOUBLE_EQ(reply.force[i], force[i]);
    EXPECT_DOUBLE_EQ(reply.torque[i], torque[i]);
  }

  EXPECT_FALSE(decodeReply(buf, kReplySize - 1, reply));
}
