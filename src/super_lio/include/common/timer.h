#ifndef TIMER_H_
#define TIMER_H_

#include <glog/logging.h>
#include <chrono>
#include <cstddef>
#include <map>
#include <string>
#include <utility>

class Timer {
 public:
  struct TimerRecord {
    double total_ms = 0.0;
    std::size_t count = 0;
  };

  template <class F>
  void Evaluate(F&& func, const std::string& func_name) {
    const auto start = std::chrono::steady_clock::now();
    std::forward<F>(func)();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    auto& record = records_[func_name];
    record.total_ms += std::chrono::duration<double, std::milli>(elapsed).count();
    ++record.count;
  }

  void PrintAll() {
    LOG(INFO) << ">>> ===== Printing run time =====";
    for (const auto& [name, record] : records_) {
      LOG(INFO) << "> [ " << name << " ] average time usage: "
                << record.total_ms / record.count << " ms , called times: " << record.count;
    }
  }

  /// clean the records
  void Clear() { records_.clear(); }

 private:
  std::map<std::string, TimerRecord> records_;
};

#endif