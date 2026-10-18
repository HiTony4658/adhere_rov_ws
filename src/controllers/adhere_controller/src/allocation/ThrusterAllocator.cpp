#include "adhere_controller/allocation/ThrusterAllocator.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace adhere_controller
{

Mat68 buildThrusterAllocationMatrix(
  const std::array<Vec3, 8> & pos, const std::array<Vec3, 8> & dir)
{
  Mat68 b = Mat68::Zero();
  for (int i = 0; i < 8; ++i) {
    b.block<3, 1>(0, i) = dir[i];
    b.block<3, 1>(3, i) = pos[i].cross(dir[i]);
  }
  return b;
}

Mat86 buildThrusterAllocationPseudoInverse(const Mat68 & b, double lambda_reg)
{
  const Mat6 gram = b * b.transpose() + lambda_reg * Mat6::Identity();
  return b.transpose() * gram.inverse();
}

Mat34 buildPadNormalMap(const std::array<Vec3, 4> & pad_pos)
{
  Mat34 g = Mat34::Zero();
  for (int i = 0; i < 4; ++i) {
    g(0, i) = 1.0;
    g(1, i) = -pad_pos[i].z();
    g(2, i) = pad_pos[i].y();
  }
  return g;
}

Mat43 buildPadNormalMapPseudoInverse(const Mat34 & g, double lambda_reg)
{
  const Mat3 gram = g * g.transpose() + lambda_reg * Mat3::Identity();
  return g.transpose() * gram.inverse();
}

Mat48 buildContactMap(const Mat43 & g_pinv, const Mat68 & b)
{
  Mat38 b_sub;
  b_sub.row(0) = b.row(0);
  b_sub.row(1) = b.row(4);
  b_sub.row(2) = b.row(5);
  return -g_pinv * b_sub;
}

std::pair<double, double> padHalfSpans(const std::array<Vec3, 4> & pad_pos)
{
  double l_y = 0.0;
  double l_z = 0.0;
  for (const auto & p : pad_pos) {
    l_y = std::max(l_y, std::abs(p.y()));
    l_z = std::max(l_z, std::abs(p.z()));
  }
  return {l_y, l_z};
}

Mat48 buildKernConstraintMatrix(const Mat68 & b, double l_y, double l_z, double eps_margin)
{
  const Eigen::Matrix<double, 1, 8> b_fx = b.row(0);
  const Eigen::Matrix<double, 1, 8> b_ty = b.row(4);
  const Eigen::Matrix<double, 1, 8> b_tz = b.row(5);
  constexpr std::array<std::pair<double, double>, 4> kSigns = {
    {{1.0, 1.0}, {1.0, -1.0}, {-1.0, 1.0}, {-1.0, -1.0}}};

  Mat48 c = Mat48::Zero();
  for (size_t m = 0; m < kSigns.size(); ++m) {
    const double s_y = kSigns[m].first;
    const double s_z = kSigns[m].second;
    c.row(m) = (s_y / l_y) * b_tz - (s_z / l_z) * b_ty + (1.0 - eps_margin) * b_fx;
  }
  return c;
}

}  // namespace adhere_controller
