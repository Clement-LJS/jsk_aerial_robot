#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <exception>

namespace roll_yaw
{
inline double steadySeconds()
{ return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// One persistent thread, at most one job/result. Callback methods never wait.
// Cancellation is cooperative; destruction joins ONLY during plugin shutdown.
template<class Result> class Worker
{
public:
  using Job=std::function<Result(const std::function<bool()>&)>;
  Worker():thread_([this]{run();}) {}
  ~Worker()
  {
    // Protect the wait predicate to prevent a lost shutdown notification.
    { std::lock_guard<std::mutex> l(mutex_); quit_=true; cancel(); }
    wake_.notify_one(); thread_.join();
  }
  Worker(const Worker&)=delete;
  Worker& operator=(const Worker&)=delete;
  void cancel() { ++generation_; }
  bool submit(Job job)
  {
    std::unique_lock<std::mutex> l(mutex_,std::try_to_lock);
    if(!l||busy_||job_||result_) return false;
    job_=std::move(job); job_generation_=generation_.load(); wake_.notify_one(); return true;
  }
  std::unique_ptr<Result> take()
  {
    std::unique_lock<std::mutex> l(mutex_,std::try_to_lock);
    return l?std::move(result_):nullptr;
  }
private:
  void run()
  {
    while(!quit_) {
      Job job; unsigned long generation;
      {
        std::unique_lock<std::mutex> l(mutex_);
        wake_.wait(l,[this]{return quit_||bool(job_);});
        if(quit_) return;
        job=std::move(job_); generation=job_generation_; busy_=true;
      }
      auto cancelled=[this,generation]{return quit_||generation_.load()!=generation;};
      std::unique_ptr<Result> result;
      try { result.reset(new Result(job(cancelled))); }
      catch(const std::exception& e) { result.reset(new Result()); result->reason=std::string("worker exception: ")+e.what(); }
      catch(...) { result.reset(new Result()); result->reason="unknown worker exception"; }
      { std::lock_guard<std::mutex> l(mutex_); busy_=false; if(!cancelled()) result_=std::move(result); }
    }
  }
  std::atomic<bool> quit_{false};
  std::atomic<unsigned long> generation_{0};
  unsigned long job_generation_=0;
  std::mutex mutex_; std::condition_variable wake_; Job job_;
  std::unique_ptr<Result> result_; bool busy_=false;
  std::thread thread_;
};
}
