#ifndef BOARDC_SHOOT_HPP
#define BOARDC_SHOOT_HPP
#include <librm.hpp>

enum ShootState { kStop, kInitialize, kReady, kShooting, kCooling };
constexpr float kBoosrterZeroPoint = 0.345f;

class Shoot_Controller {
 public:
  Shoot_Controller(rm::hal::Can &booster_can, rm::hal::Can &fric_can);

  rm::device::M3508 *friction_left{nullptr};
  rm::device::M3508 *friction_right{nullptr};
  rm::device::M3508 *friction_up{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *booster_motor{nullptr};

  rm::modules::PID *fric_left_pid{nullptr};
  rm::modules::PID *fric_right_pid{nullptr};
  rm::modules::PID *fric_up_pid{nullptr};
  rm::modules::PID *booster_position_pid{nullptr};
  rm::modules::PID *booster_speed_pid{nullptr};

  void Init();

  void Enable(bool enable);

  void Task();

 private:
  ShootState rotor_state_ = ShootState::kStop;
  bool booster_enable_ = false;
  bool booster_disable_ = false;

  float now_angle_ = 0.0f;
  float next_angle_ = 0.0f;
  float init_time_ = 0.0f;
  float shoot_time_ = 0.0f;
  float booster_pos_ = 0.0f;

  void Update();
};

#endif  // BOARDC_SHOOT_HPP
