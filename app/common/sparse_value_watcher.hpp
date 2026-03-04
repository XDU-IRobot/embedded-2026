#pragma once

#include <etl/delegate.h>

/**
 * @brief 监视一个离散值的变化，当值发生变化时触发回调函数
 * @tparam T 要监视的值的类型
 */
template <typename T>
class SparseValueWatcher {
 public:
  /**
   * @brief 回调函数类型
   * @param old_value 变化前的值
   * @param new_value 变化后的值
   */
  using Callback = etl::delegate<void(const T &old_value, const T &new_value)>;

  SparseValueWatcher() = default;

  explicit SparseValueWatcher(T initial_value) : value_(std::move(initial_value)) {}

  /**
   * @brief 构造函数
   * @param initial_value 初始值
   * @param callback 回调函数
   */
  SparseValueWatcher(T initial_value, Callback callback) : value_(std::move(initial_value)), callback_(callback) {}

  /**
   * @brief 设置值变化时的回调函数
   * @param callback
   */
  void OnValueChange(Callback callback) { callback_ = callback; }

  /**
   * @brief 更新值并检查是否发生变化
   * @param new_value 新的值
   */
  void Update(const T &new_value) {
    if (new_value != value_) {
      T old_value = value_;
      value_ = new_value;
      if (callback_) {
        callback_(old_value, value_);
      }
    }
  }

  /**
   * @brief 获取当前值
   * @return 当前值
   */
  const T &value() const { return value_; }

 private:
  T value_;
  Callback callback_;
};
