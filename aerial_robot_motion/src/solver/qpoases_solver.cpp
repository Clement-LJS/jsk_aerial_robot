#include <aerial_robot_motion/solver/qpoases_solver.h>
#include <qpOASES.hpp>
#include <array>

namespace aerial_robot_motion
{
struct QpOasesSolver::Impl
{
  using Matrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using Vector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;
  struct Buffer { Matrix H, A; Vector g, lb, ub, lbA, ubA; };
  // qpOASES retains H/A pointers. Preserve the previous solve while preparing the next matrices.
  std::array<Buffer, 2> buffers;
  std::unique_ptr<qpOASES::SQProblem> qp;
  int active = 0, working_sets = 100;
  double tolerance = 1e-6;
};

QpOasesSolver::QpOasesSolver() : impl_(new Impl) {}
QpOasesSolver::~QpOasesSolver() = default;
void QpOasesSolver::reset() { impl_->qp.reset(); }
void QpOasesSolver::configure(const ros::NodeHandle& nh)
{
  nh.param("max_working_set_recalculations", impl_->working_sets, 100);
  nh.param("feasibility_tolerance", impl_->tolerance, 1e-6);
  if (impl_->working_sets <= 0 || !std::isfinite(impl_->tolerance) || impl_->tolerance <= 0)
    throw std::runtime_error("invalid qpOASES iteration/tolerance configuration");
  reset();
}

bool QpOasesSolver::solve(const QPProblem& problem, Eigen::VectorXd& solution, std::string& status)
{
  solution.resize(0);
  if (!problem.validate(status)) { reset(); return false; }
  const int n = problem.g.size(), m = problem.A.rows();
  if (impl_->qp && (impl_->qp->getNV() != n || impl_->qp->getNC() != m)) reset();
  const bool hotstart = static_cast<bool>(impl_->qp);
  const int next = hotstart ? 1 - impl_->active : 0;
  auto& buffer = impl_->buffers[next];
  buffer.H = problem.H.cast<qpOASES::real_t>(); buffer.A = problem.A.cast<qpOASES::real_t>();
  buffer.g = problem.g.cast<qpOASES::real_t>();
  const auto convert = [](const Eigen::VectorXd& input) -> Impl::Vector {
    return input.unaryExpr([](double x) { return static_cast<qpOASES::real_t>(
        std::max(-qpOASES::INFTY, std::min(qpOASES::INFTY, x))); });
  };
  buffer.lb = convert(problem.lb); buffer.ub = convert(problem.ub);
  buffer.lbA = convert(problem.lbA); buffer.ubA = convert(problem.ubA);
  qpOASES::int_t iterations = impl_->working_sets;
  qpOASES::returnValue result;
  if (!hotstart)
  {
    impl_->qp.reset(new qpOASES::SQProblem(n, m));
    qpOASES::Options options; options.setToReliable(); options.printLevel = qpOASES::PL_NONE;
    options.enableEqualities = qpOASES::BT_TRUE;
    impl_->qp->setOptions(options);
    result = impl_->qp->init(buffer.H.data(), buffer.g.data(), m ? buffer.A.data() : nullptr,
        buffer.lb.data(), buffer.ub.data(), m ? buffer.lbA.data() : nullptr,
        m ? buffer.ubA.data() : nullptr, iterations);
  }
  else
    result = impl_->qp->hotstart(buffer.H.data(), buffer.g.data(), m ? buffer.A.data() : nullptr,
        buffer.lb.data(), buffer.ub.data(), m ? buffer.lbA.data() : nullptr,
        m ? buffer.ubA.data() : nullptr, iterations);
  if (result != qpOASES::SUCCESSFUL_RETURN)
  {
    status = "qpOASES failed: " + std::to_string(static_cast<int>(result));
    reset(); return false;
  }
  Impl::Vector primal(n);
  if (impl_->qp->getPrimalSolution(primal.data()) != qpOASES::SUCCESSFUL_RETURN || !primal.allFinite())
    { status = "qpOASES returned invalid primal solution"; reset(); return false; }
  solution = primal.cast<double>();
  const Eigen::VectorXd rows = problem.A * solution;
  const double tolerance = impl_->tolerance;
  if ((solution.array() < problem.lb.array() - tolerance).any() ||
      (solution.array() > problem.ub.array() + tolerance).any() ||
      (rows.array() < problem.lbA.array() - tolerance).any() ||
      (rows.array() > problem.ubA.array() + tolerance).any())
  {
    status = "qpOASES primal solution violates hard bounds";
    solution.resize(0); reset(); return false;
  }
  impl_->active = next;
  status = hotstart ? "success (hotstart)" : "success (initialized)";
  return true;
}
}
