// Wire format for SmartServoPTP.java on the KUKA cabinet (iiwa 14 R820).
//
// Commands (mode auto-selected by packet size):
//   72 B -> joint PTP:   [J1..J7 (rad), speed_rel (0..1), accel_rel (0..1)]
//   64 B -> cartesian LIN:[X,Y,Z (mm), A,B,C (rad), speed (mm/s), accel
//   (mm/s^2)]
//    1 B -> ping (latch the client endpoint, no motion)
//
// Reply (105 B, streamed at 50 Hz and after every command):
//   [status (1 B)] + [J1..J7 (7 x BE double)] + [Fx Fy Fz] + [Tx Ty Tz]
//
// Everything is big-endian. Header-only so the packet layout can be unit
// tested.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace teleop::kuka {

inline constexpr std::size_t kDof = 7;
inline constexpr std::size_t kJointPacketSize = 72;
inline constexpr std::size_t kCartesianPacketSize = 64;
inline constexpr std::size_t kReplySize = 1 + kDof * 8 + 6 * 8; // 105

enum class Status : std::uint8_t {
  Moving = 0x00,
  Reached = 0x01,
  Rejected = 0x02,
  Idle = 0x03,
  Unknown = 0xFF,
};

struct Reply {
  Status status{Status::Unknown};
  std::array<double, kDof> q{};
  std::array<double, 3> force{};
  std::array<double, 3> torque{};
};

inline std::uint64_t swap64(std::uint64_t v) {
  v = ((v << 8) & 0xFF00FF00FF00FF00ULL) | ((v >> 8) & 0x00FF00FF00FF00FFULL);
  v = ((v << 16) & 0xFFFF0000FFFF0000ULL) | ((v >> 16) & 0x0000FFFF0000FFFFULL);
  return (v << 32) | (v >> 32);
}

inline void putBe(std::uint8_t *out, double value) {
  std::uint64_t raw;
  std::memcpy(&raw, &value, sizeof(raw));
  raw = swap64(raw);
  std::memcpy(out, &raw, sizeof(raw));
}

inline double getBe(const std::uint8_t *in) {
  std::uint64_t raw;
  std::memcpy(&raw, in, sizeof(raw));
  raw = swap64(raw);
  double value;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

inline void encodeJoint(const std::array<double, kDof> &q, double speed_rel,
                        double accel_rel, std::uint8_t out[kJointPacketSize]) {
  for (std::size_t i = 0; i < kDof; ++i) {
    putBe(out + i * 8, q[i]);
  }
  putBe(out + 56, speed_rel);
  putBe(out + 64, accel_rel);
}

inline void encodeCartesian(double x_mm, double y_mm, double z_mm, double a_rad,
                            double b_rad, double c_rad, double speed_mm_s,
                            double accel_mm_s2,
                            std::uint8_t out[kCartesianPacketSize]) {
  const double values[8] = {x_mm,  y_mm,  z_mm,       a_rad,
                            b_rad, c_rad, speed_mm_s, accel_mm_s2};
  for (std::size_t i = 0; i < 8; ++i) {
    putBe(out + i * 8, values[i]);
  }
}

inline bool decodeReply(const std::uint8_t *buf, std::size_t len, Reply &out) {
  if (len != kReplySize) {
    return false;
  }
  switch (buf[0]) {
  case 0x00:
    out.status = Status::Moving;
    break;
  case 0x01:
    out.status = Status::Reached;
    break;
  case 0x02:
    out.status = Status::Rejected;
    break;
  case 0x03:
    out.status = Status::Idle;
    break;
  default:
    out.status = Status::Unknown;
    break;
  }
  for (std::size_t i = 0; i < kDof; ++i) {
    out.q[i] = getBe(buf + 1 + i * 8);
  }
  for (std::size_t i = 0; i < 3; ++i) {
    out.force[i] = getBe(buf + 57 + i * 8);
    out.torque[i] = getBe(buf + 81 + i * 8);
  }
  return true;
}

} // namespace teleop::kuka
