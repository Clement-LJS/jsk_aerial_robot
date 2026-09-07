#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace aerial_robot_motion
{
struct QPProblem
{
  Eigen::MatrixXd H, A;
  Eigen::VectorXd g, lb, ub, lbA, ubA;

  void reset(int n, double regularization = 1e-8)
  {
    if (n <= 0 || !std::isfinite(regularization) || regularization < 0)
      throw std::invalid_argument("invalid QP dimension or regularization");
    H = regularization * Eigen::MatrixXd::Identity(n, n);
    g = Eigen::VectorXd::Zero(n);
    lb = Eigen::VectorXd::Constant(n, -std::numeric_limits<double>::infinity());
    ub = -lb;
    A.resize(0, n); lbA.resize(0); ubA.resize(0);
  }

  void addCost(const Eigen::MatrixXd& h, const Eigen::VectorXd& gradient)
  {
    if (h.rows() != H.rows() || h.cols() != H.cols() || gradient.size() != g.size() ||
        !h.allFinite() || !gradient.allFinite())
      throw std::invalid_argument("cost has invalid dimensions or nonfinite values");
    H += h; g += gradient;
  }

  void intersectBounds(const Eigen::VectorXd& lower, const Eigen::VectorXd& upper)
  {
    if (lower.size() != g.size() || upper.size() != g.size() || lower.hasNaN() || upper.hasNaN())
      throw std::invalid_argument("variable bounds have invalid dimensions or NaN");
    lb = lb.cwiseMax(lower); ub = ub.cwiseMin(upper);
  }

  void appendConstraints(const Eigen::MatrixXd& matrix, const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper)
  {
    if (matrix.cols() != g.size() || matrix.rows() != lower.size() || matrix.rows() != upper.size() ||
        !matrix.allFinite() || lower.hasNaN() || upper.hasNaN())
      throw std::invalid_argument("linear constraints have invalid dimensions or nonfinite values");
    const int offset = A.rows();
    A.conservativeResize(offset + matrix.rows(), Eigen::NoChange);
    lbA.conservativeResize(offset + matrix.rows()); ubA.conservativeResize(offset + matrix.rows());
    A.bottomRows(matrix.rows()) = matrix;
    lbA.tail(lower.size()) = lower; ubA.tail(upper.size()) = upper;
  }

  bool validate(std::string& reason) const
  {
    const int n = g.size();
    if (n <= 0 || H.rows() != n || H.cols() != n || lb.size() != n || ub.size() != n ||
        A.cols() != n || A.rows() != lbA.size() || A.rows() != ubA.size())
      { reason = "QP dimension mismatch"; return false; }
    if (!H.allFinite() || !g.allFinite() || !A.allFinite() || lb.hasNaN() || ub.hasNaN() ||
        lbA.hasNaN() || ubA.hasNaN())
      { reason = "QP contains nonfinite coefficients or NaN bounds"; return false; }
    if ((lb.array() > ub.array()).any() || (lbA.array() > ubA.array()).any() ||
        (lb.array() == std::numeric_limits<double>::infinity()).any() ||
        (ub.array() == -std::numeric_limits<double>::infinity()).any() ||
        (lbA.array() == std::numeric_limits<double>::infinity()).any() ||
        (ubA.array() == -std::numeric_limits<double>::infinity()).any())
      { reason = "QP bounds are infeasible"; return false; }
    const double tolerance = 1e-10 * std::max(1.0, H.norm());
    if ((H - H.transpose()).norm() > tolerance)
      { reason = "QP Hessian is not symmetric"; return false; }
    Eigen::LDLT<Eigen::MatrixXd> decomposition(H);
    if (decomposition.info() != Eigen::Success || decomposition.vectorD().minCoeff() < -tolerance)
      { reason = "QP Hessian is not positive semidefinite"; return false; }
    reason.clear(); return true;
  }
};
}  // namespace aerial_robot_motion
