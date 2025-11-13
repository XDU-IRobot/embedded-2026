#pragma once

class Counter {
public:
    Counter(float target_value_min, float target_value_max) : target_value_cycle_(target_value_max - target_value_min) {
    }

    void Init(float target_value) {
        count_ = target_value;
        last_value_ = target_value;
    }

    void IncreaseUpdate(float target_value) {
        if (target_value < last_value_ - target_value_cycle_ * 0.1f) {
            count_ += target_value - last_value_ + target_value_cycle_;
        } else {
            count_ += target_value - last_value_;
        }
        last_value_ = target_value;
    }

    void DecreaseUpdate(float target_value) {
        if (target_value > last_value_ + target_value_cycle_ * 0.1f) {
            count_ += target_value - last_value_ - target_value_cycle_;
        } else {
            count_ += target_value - last_value_;
        }
        last_value_ = target_value;

    }

    auto &output() { return count_; }

private:
    float count_ = 0.0f;
    float last_value_ = 0.0f;
    float target_value_cycle_ = 0.0f;
};
