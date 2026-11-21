// -*- mode: c++ -*-
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace perching_dynamics
{
// Observer-only conditioning. The flight attitude controller keeps its original
// gyro feedback. All inputs here refer to one interpolated measurement time.
class ObservationFilter
{
public:
  Eigen::VectorXd q, rate, effort;
  Eigen::Vector3d omega = Eigen::Vector3d::Zero(), gyro = omega;
  Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
  // Condition measured coordinates and their derivatives at one bandwidth:
  // a 0.001534-rad roll encoder tick otherwise gives 0.077 rad/s at 50 Hz.
  // Body-rate disagreement is diagnostic only; flight feedback is unchanged.
  double time_constant = .4;
  bool observe(const Eigen::VectorXd& position, const Eigen::Matrix3d& rotation,
               const Eigen::Vector3d& measured_gyro, const Eigen::VectorXd& torque,
               const Eigen::Matrix<double, 6, 1>& input, double stamp, double max_interval);
  void reset();
private:
  double stamp_ = 0, started_ = 0;
  Eigen::VectorXd previous_;
  Eigen::Matrix3d previous_rotation_ = Eigen::Matrix3d::Identity();
};

// n-dimensional generalized load, never a Cartesian wrench. Revolute entries
// are N.m; friction, compliance and modelling error also appear in the residual.
class MomentumObserver
{
public:
  void reset(const std::string& reason = "awaiting first attached sample");
  bool observe(const Eigen::VectorXd& momentum, const Eigen::VectorXd& predicted_rate,
               double stamp, double gain, double max_dt);
  bool ready() const { return ready_; }
  double stamp() const { return stamp_; }
  const std::string& reason() const { return reason_; }
  const Eigen::VectorXd& load() const { return residual_; }
private:
  Eigen::VectorXd initial_, integral_, residual_;
  double stamp_ = 0;
  bool ready_ = false;
  std::string reason_ = "awaiting first attached sample";
};

// One pending/running job and one unread result; control ticks never wait.
template<class Result> class Worker
{
public:
  Worker() : thread_([this]() { run(); }) {}
  ~Worker()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      quit_ = true;
    }
    wake_.notify_one();
    thread_.join();
  }
  bool submit(std::function<Result()> job)
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if(!lock.owns_lock() || busy_ || result_ || quit_) return false;
    job_ = std::move(job);
    job_generation_ = generation_.load();
    busy_ = true;
    wake_.notify_one();
    return true;
  }
  std::unique_ptr<Result> take()
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if(!lock.owns_lock() || busy_) return {};
    if(job_generation_ != generation_.load()) result_.reset();
    return std::move(result_);
  }
  void cancel() { ++generation_; }
private:
  void run()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    while(true)
    {
      wake_.wait(lock, [this]() { return quit_ || static_cast<bool>(job_); });
      if(quit_) return;
      std::function<Result()> job;
      job.swap(job_);
      const auto generation = job_generation_;
      lock.unlock();
      auto result = std::make_unique<Result>();
      try { *result = job(); }
      catch(const std::exception& error) { result->ok = false; result->reason = error.what(); }
      catch(...) { result->ok = false; result->reason = "unknown observation worker exception"; }
      lock.lock();
      if(!quit_ && generation == generation_.load()) result_ = std::move(result);
      busy_ = false;
    }
  }
  std::mutex mutex_;
  std::condition_variable wake_;
  std::atomic<std::size_t> generation_{0};
  std::size_t job_generation_ = 0;
  bool busy_ = false, quit_ = false;
  std::function<Result()> job_;
  std::unique_ptr<Result> result_;
  std::thread thread_;
};
} // namespace perching_dynamics
