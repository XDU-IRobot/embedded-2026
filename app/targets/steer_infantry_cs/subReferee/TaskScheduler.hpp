//
// Created by Jason on 26-5-18.
//

#ifndef TASKSCHEDULER_HPP
#define TASKSCHEDULER_HPP
#include "etl/vector.h"

namespace rm::device {
class UITask {
  void (*func_)();
  float freq_;
  float acc_{0.0f};
  bool is_scheduled_{false};

 public:
  UITask() = delete;

  explicit UITask(void (*func)(), const float freq = 0.0f) : func_(func), freq_(freq) {};
  friend class UITaskScheduler;
};

// warning : 作为UI而言，freq必须小于等于30hz，注意只有30个任务。
class UITaskScheduler {
  float freq_{0};  // 严格与这个管理器调用频率一致
  etl::vector<UITask *, 30> tasks_;
  etl::vector<UITask *, 30> tasks_static_;

 public:
  UITaskScheduler() = delete;

  explicit UITaskScheduler(const float freq) : freq_(freq) {};

  bool addTask(UITask *task) {
    if (task->is_scheduled_ == true || task->freq_ <= 0) {
      return false;
    }
    if (tasks_.available() <= 0) {
      return false;
    }
    tasks_.push_back(task);
    task->is_scheduled_ = true;
    return true;
  }

  bool addTaskStatic(UITask *task) {
    if (task->is_scheduled_ == true || task->freq_ > 0) {
      return false;
    }
    if (tasks_static_.available() <= 0) {
      return false;
    }
    tasks_static_.push_back(task);
    task->is_scheduled_ = true;
    return true;
  }

  bool delTask(const UITask *task) {
    const auto it = std::find(tasks_.begin(), tasks_.end(), task);

    if (it == tasks_.end()) {
      return false;
    }

    (*it)->is_scheduled_ = false;
    tasks_.erase(it);
    return true;
  }

  bool delTaskStatic(const UITask *task) {
    const auto it = std::find(tasks_static_.begin(), tasks_static_.end(), task);

    if (it == tasks_static_.end()) {
      return false;
    }

    (*it)->is_scheduled_ = false;
    tasks_static_.erase(it);
    return true;
  }

  void schedule() {
    if (tasks_.empty() && tasks_static_.empty()) return;
    if (!tasks_static_.empty()) {
      tasks_static_.back()->func_();
      tasks_static_.back()->is_scheduled_ = false;
      tasks_static_.pop_back();
      return;
    }
    UITask *selected = nullptr;
    float max_acc = 0;

    for (auto *task : tasks_) {
      task->acc_ += task->freq_;
    }

    for (auto *task : tasks_) {
      if (task->acc_ > max_acc) {
        max_acc = task->acc_;
        selected = task;
      }
    }

    if (selected != nullptr) {
      selected->func_();
      selected->acc_ -= freq_;
    }
  }
};
}  // namespace rm::device

#endif  // TASKSCHEDULER_HPP