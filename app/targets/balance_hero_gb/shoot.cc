#include "shoot.hpp"
#include "main.hpp"

Shoot_Controller::Shoot_Controller(rm::hal::Can &booster_can, rm::hal::Can &fric_can) {
  booster_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      booster_can, {0x10, 0x09, 3.141593f, 30.0f, 10.0f, {0.f, 500.f}, {0.f, 5.f}}, true};
  friction_left = new rm::device::M3508{
      fric_can,
      2,
  };
  friction_right = new rm::device::M3508{fric_can, 3, true};
  friction_up = new rm::device::M3508{fric_can, 1};
}

void Shoot_Controller::Init() {
  fric_left_pid = new rm::modules::PID{0, 0, 0, 0, 0};
  fric_right_pid = new rm::modules::PID{0, 0, 0, 0, 0};
  fric_up_pid = new rm::modules::PID{0, 0, 0, 0, 0};
  booster_position_pid = new rm::modules::PID{60.f, 0, 560.f, 24.f, 0};
  booster_speed_pid = new rm::modules::PID{0.3f, 0, 0.02f, 6.4f, 0};
  booster_position_pid->SetCircular(true);
  booster_position_pid->SetCircularCycle(M_PI * 2.f);
}

void Shoot_Controller::Enable(bool enable) {
  if (enable) {
    booster_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  }
}

void Shoot_Controller::Task() { Update(); }

void Shoot_Controller::Update() {
  booster_pos_ = booster_motor->pos();
  // 拨盘状态逻辑
  switch (rotor_state_) {
    case ShootState::kStop:
      if (globals->rc->switch_r() == DR16::SwitchPosition::kUp) {
        rotor_state_ = ShootState::kInitialize;
        now_angle_ = kBoosrterZeroPoint;
        next_angle_ = kBoosrterZeroPoint;
        init_time_ = 600;
      }
      break;
    case ShootState::kInitialize:
      if (init_time_ > 0) {
        init_time_--;
        if (init_time_ == 50) {
          booster_disable_ = true;
        }
      } else {
        while (now_angle_ < booster_pos_) {
          now_angle_ += M_PI / 3.f;
        }
        while (now_angle_ > booster_pos_) {
          now_angle_ -= M_PI / 3.f;
        }
        if (booster_pos_ - now_angle_ > M_PI / 6.f) {
          now_angle_ += M_PI / 3.f;
          if (now_angle_ > M_PI) {
            now_angle_ -= 2 * M_PI;
          }
          next_angle_ = now_angle_ + M_PI / 3.f;
          if (next_angle_ > M_PI) {
            next_angle_ -= 2 * M_PI;
          }
        } else {
          next_angle_ = now_angle_ + M_PI / 3.f;
          if (next_angle_ > M_PI) {
            next_angle_ -= 2 * M_PI;
          }
          now_angle_ = booster_pos_;
        }
        booster_enable_ = true;
        rotor_state_ = ShootState::kReady;
      }
      break;
    case ShootState::kReady:
      if (globals->rc->switch_r() != DR16::SwitchPosition::kUp) {
        booster_disable_ = true;
        rotor_state_ = ShootState::kStop;
      } else {
        if (globals->rc->dial() > 100) {
          rotor_state_ = ShootState::kShooting;
          shoot_time_ = 360;
          now_angle_ = next_angle_;
        }
      }
      break;
    case ShootState::kShooting:
      if (shoot_time_ > 0) {
        shoot_time_--;
      } else {
        rotor_state_ = ShootState::kCooling;
        // 堵转检测
        if (rm::modules::Wrap(now_angle_ - booster_pos_, -M_PI, M_PI) > M_PI / 18.f) {
          now_angle_ = booster_pos_ - M_PI / 90.f;
        } else {
          next_angle_ = now_angle_ + M_PI / 3.f;
          if (next_angle_ > M_PI) {
            next_angle_ -= 2 * M_PI;
          }
        }
      }
      break;
    case ShootState::kCooling:
      if (globals->rc->switch_r() != DR16::SwitchPosition::kUp) {
        booster_disable_ = true;
        rotor_state_ = ShootState::kStop;
      } else {
        rotor_state_ = ShootState::kReady;
      }
      break;
    default:
      rotor_state_ = ShootState::kStop;
      break;
  }

  booster_position_pid->Update(now_angle_, booster_motor->pos());
  booster_speed_pid->Update(booster_position_pid->out(), booster_motor->vel());

  if (booster_enable_ == true) {
    booster_motor->SendInstruction(DmMotorInstructions::kEnable);
    booster_enable_ = false;
  } else if (booster_disable_ == true) {
    booster_motor->SendInstruction(DmMotorInstructions::kDisable);
    booster_disable_ = false;
  } else if (rotor_state_ == ShootState::kReady || rotor_state_ == ShootState::kCooling ||
             rotor_state_ == ShootState::kShooting) {
    booster_motor->SetPosition(0, 0, -booster_speed_pid->out(), 0, 0);
    // booster_motor->SetPosition(0, 0, 0, 0, 0);
  }
}