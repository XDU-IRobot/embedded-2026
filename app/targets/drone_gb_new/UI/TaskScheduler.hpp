//
// Created by Jason on 26-5-18.
//

#ifndef TASKSCHEDULER_HPP
#define TASKSCHEDULER_HPP
#include "etl/vector.h"

namespace rm::device {
class UITask {
  void (*func_)();
  float acc_;
  float freq_;

 public:
  UITask(void (*func)(), const float freq) : func_(func), acc_(0.0f), freq_(freq) {};
  friend class UITaskScheduler;
};

// warning : 作为UI而言，freq必须小于等于30hz。
class UITaskScheduler {
  float freq_{0};  // 严格与这个管理器调用频率一致
  etl::vector<UITask *, 10> tasks_;

 public:
  explicit UITaskScheduler(const float freq) : freq_(freq) {};

  void addTask(UITask *task) { tasks_.push_back(task); }

  void schedule() {
    if (tasks_.empty()) return;

    UITask *selected = nullptr;
    float max_acc = 0;

    for (auto *task : tasks_) {
      if (task->acc_ > max_acc) {
        max_acc = task->acc_;
        selected = task;
      }
    }

    if (selected) selected->func_();

    for (auto *task : tasks_) {
      task->acc_ += task->freq_;
    }
    selected->acc_ -= freq_;
  }
};
}  // namespace rm::device

#endif  // TASKSCHEDULER_HPP