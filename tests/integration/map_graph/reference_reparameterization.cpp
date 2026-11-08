// Exercise the actual Ceres residual under a change of scan reference frame.
// This is a first-order covariance identity, not an exact nonlinear cost identity.
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "solvers/ceres_utils.h"

struct Pose { Eigen::Vector2d t; double a; };
Pose compose(const Pose &x, const Pose &y) {
  return {x.t + RotationMatrix2D(x.a) * y.t, NormalizeAngle(x.a + y.a)};
}
Pose inverse(const Pose &x) {
  return {-RotationMatrix2D(x.a).transpose() * x.t, -x.a};
}
Eigen::Vector3d residual(const Pose &a, const Pose &b, const Pose &z) {
  Eigen::Vector3d r;
  PoseGraph2dErrorTerm f(z.t.x(), z.t.y(), z.a, Eigen::Matrix3d::Identity());
  f(&a.t.x(), &a.t.y(), &a.a, &b.t.x(), &b.t.y(), &b.a, r.data());
  return r;
}
int main() {
  const Pose xa{{2.3, -1.2}, .8}, z{{1.1, -.7}, -.6};
  const Pose ai{{.03, -.02}, .13}, aj{{-.08, .04}, -.21};
  const Pose zap = compose(compose(inverse(ai), z), aj);
  Eigen::Matrix3d j = Eigen::Matrix3d::Identity();
  j.topLeftCorner<2,2>() = RotationMatrix2D(ai.a).transpose();
  Eigen::Vector2d perpendicular(-aj.t.y(), aj.t.x());
  j.topRightCorner<2,1>() = RotationMatrix2D(ai.a).transpose() *
    RotationMatrix2D(z.a) * perpendicular;
  Eigen::Matrix3d sigma;
  sigma << .03, .004, -.002, .004, .02, .001, -.002, .001, .008;
  const Eigen::Matrix3d sp = j * sigma * j.transpose();
  const Eigen::Vector3d direction(.7, -.5, .4);
  double previous_remainder = 1e30, max_jacobian_error = 0.;
  for (int axis = 0; axis < 3; ++axis) {
    const double h = 1e-6;
    Eigen::Vector3d d = Eigen::Vector3d::Zero(); d(axis) = h;
    Pose plus{z.t + d.head<2>(), z.a + d.z()};
    Pose minus{z.t - d.head<2>(), z.a - d.z()};
    auto transformed = [&](const Pose &q) {
      return residual(compose(xa, ai), compose(compose(xa, q), aj), zap);
    };
    max_jacobian_error = std::max(max_jacobian_error,
      ((transformed(plus) - transformed(minus)) / (2*h) - j.col(axis)).norm());
  }
  if (max_jacobian_error > 2e-9) throw std::runtime_error("residual Jacobian mismatch");
  std::cout << "{\"native_residual_jacobian_max_error\":" << max_jacobian_error << ",\"trials\":[";
  bool first = true;
  for (double epsilon : {.01, .001, .0001}) {
    const Eigen::Vector3d d = epsilon * direction;
    Pose q{z.t + d.head<2>(), z.a + d.z()};
    const auto r = residual(xa, compose(xa, q), z);
    const auto rp = residual(compose(xa, ai), compose(compose(xa, q), aj), zap);
    const double remainder = (rp - j*r).norm();
    const double cost = r.dot(sigma.llt().solve(r));
    const double costp = rp.dot(sp.llt().solve(rp));
    if (remainder > .01 * epsilon * epsilon || remainder >= previous_remainder*.011) {
      throw std::runtime_error("nonlinear remainder does not scale quadratically");
    }
    if (std::abs(costp-cost)/cost > epsilon*.1) throw std::runtime_error("Mahalanobis mismatch");
    if (!first) std::cout << ','; first = false;
    std::cout << "{\"epsilon\":" << epsilon << ",\"second_order_remainder\":" << remainder
      << ",\"cost\":" << cost << ",\"reparameterized_cost\":" << costp << '}';
    previous_remainder = remainder;
  }
  std::cout << "],\"passed\":true}\n";
}
