
#pragma once

#include <etl/fsm.h>

#include "globals.hpp"
#include "trajectory_generator.hpp"

namespace fsm {

struct EventId {
  enum {
    kControlLoop,      ///< 控制循环时钟事件
    kForceModeSwitch,  ///< 切换模式命令
  };
};

struct StateId {
  enum : etl::fsm_state_id_t {
    kNoForce,           ///< 强制无力
    kManual,            ///< 遥控器手动模式
    kFollowTrajectory,  ///< 跟随激励轨迹
  };
};

namespace event {
/*******************************************/
struct ControlLoop : etl::message<EventId::kControlLoop> {};

/*******************************************/
struct ForceModeSwitch : etl::message<EventId::kForceModeSwitch> {
  explicit ForceModeSwitch(etl::fsm_state_id_t target) : target_mode(target) {}
  etl::fsm_state_id_t target_mode;
};
}  // namespace event

class Gimbal : public etl::fsm {
  constexpr static etl::message_router_id_t kMessageRouterId = 0;

 public:
  Gimbal() : fsm(kMessageRouterId) {}
};
extern Gimbal gimbal;

namespace state {

///*********************************
///*******  helper macros  *********
///*********************************
#define ACCEPT_MODE_SWITCH() \
  etl::fsm_state_id_t on_event(const event::ForceModeSwitch &e) { return e.target_mode; }
#define IGNORE_UNINTEREST_EVENT() \
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &) { return No_State_Change; }
#define ENTER etl::fsm_state_id_t on_enter_state()
#define REACT(EventType) etl::fsm_state_id_t on_event(const EventType &e)
///*********************************
///*********************************
///*********************************

/*******************************************/
struct NoForce : etl::fsm_state<Gimbal, NoForce, StateId::kNoForce,  //
                                event::ForceModeSwitch,              //
                                event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->led_controller.SetPattern<rm::modules::led_pattern::RedFlash>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<1>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
    globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
    return No_State_Change;
  }
};

/*******************************************/
struct Manual : etl::fsm_state<Gimbal, Manual, StateId::kManual,  //
                               event::ForceModeSwitch,            //
                               event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    // globals->yaw_motor.SetMitCommand(0, 0, 0, 12, 0.5);
    // globals->pitch_motor.SetMitCommand(0, 0, 0, 10, 0.5);
    const auto ff_torque =
        dynamics_.ComputeFf(globals->yaw_motor.pos(), globals->pitch_motor.pos(), 0, 0, 0, 0, {0, 0, -9.81f});
    ff_torque_[0] = ff_torque(0);
    ff_torque_[1] = ff_torque(1);
    globals->yaw_motor.SetMitCommand(0, 0, 0, 0, 0);
    globals->pitch_motor.SetMitCommand(0, 0, ff_torque_[1], 0, 0);
    return No_State_Change;
  }

 private:
  Gimbal2DofDynamics dynamics_;
  float ff_torque_[2]{};
};

/*******************************************/
struct FollowTrajectory : etl::fsm_state<Gimbal, FollowTrajectory, StateId::kFollowTrajectory,  //
                                         event::ForceModeSwitch,                                //
                                         event::ControlLoop> {
  ACCEPT_MODE_SWITCH();
  IGNORE_UNINTEREST_EVENT();
  ENTER {
    globals->yaw_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->pitch_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
    globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    globals->buzzer_controller.Play<rm::modules::buzzer_melody::Beeps<2>>();
    enter_timestamp_ms_ = HAL_GetTick();
    loop_divisor_ = 0;
    return No_State_Change;
  }
  REACT(event::ControlLoop) {
    const uint32_t elapsed_ms = HAL_GetTick() - enter_timestamp_ms_;
    const auto pitch_traj = traj_pitch_.evaluate(elapsed_ms / 1000.f);
    const auto yaw_traj = traj_yaw_.evaluate(elapsed_ms / 1000.f);
    yaw_target_ = yaw_traj.q * 2.5f;
    pitch_target_ = pitch_traj.q;
    // const auto ff = dynamics_.ComputeFf(0.f, pitch_traj.q, 0.f, pitch_traj.dq, 0.f, pitch_traj.ddq, {0, 0, -9.81});
    const auto ff = dynamics_.ComputeFf(0.f, pitch_traj.q, 0.f, 0, 0.f, 0, {0, 0, -9.81});
    yaw_ff_ = ff(0);
    pitch_ff_ = ff(1);
    globals->yaw_motor.SetMitCommand(yaw_target_, 0, 0, 10, 0.45);
    globals->pitch_motor.SetMitCommand(pitch_target_, 0, pitch_ff_, 10, 0.4);

    pitch_error_ = pitch_target_ - globals->pitch_motor.pos();

    loop_divisor_ = (loop_divisor_ + 1) % 50;  // 10hz
    if (loop_divisor_ == 0) {
      ReportData(elapsed_ms, globals->yaw_motor.tau(), globals->yaw_motor.pos(), globals->yaw_motor.vel(),
                 globals->pitch_motor.tau(), globals->pitch_motor.pos(), globals->pitch_motor.vel());
    }
    return No_State_Change;
  }

 private:
  static void ReportData(     //
      uint32_t timestamp_ms,  //
      float tau_yaw,          //
      float q_yaw,            //
      float dq_yaw,           //
      float tau_pitch,        //
      float q_pitch,          //
      float dq_pitch          //
  ) {
    static char tx_buf[500];
    sprintf(tx_buf, "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp_ms, tau_yaw, q_yaw, dq_yaw, tau_pitch, q_pitch,
            dq_pitch);
    while (HAL_UART_GetState(&huart6) == HAL_UART_STATE_BUSY_TX) {
      // 等待上一次传输完成
    }
    HAL_UART_Transmit_DMA(&huart6, reinterpret_cast<uint8_t *>(tx_buf), strlen(tx_buf));
  }

  uint32_t enter_timestamp_ms_{0u};  ///< 进入状态的时间戳，单位毫秒
  int loop_divisor_{0};

  float yaw_target_{0.f}, pitch_target_{0.f};
  float pitch_error_{0.f};

  Gimbal2DofDynamics dynamics_;
  float pitch_ff_{0.f}, yaw_ff_{0.f};

  // 跑5项傅里叶级数
  constexpr static size_t N_HARMONICS = 5;
  constexpr static float BASE_FREQ_HZ = 0.17f;  // 10秒一个大周期

  // 为 Yaw 轴和 Pitch 轴设置不相同的 a, b 系数，防止多轴线性相关
  constexpr static float YAW_A[N_HARMONICS] = {0.5f, -0.2f, 0.1f, -0.05f, 0.02f};
  constexpr static float YAW_B[N_HARMONICS] = {-0.3f, 0.4f, -0.15f, 0.08f, -0.01f};

  constexpr static float PITCH_A[N_HARMONICS] = {0.3f, 0.1f, -0.2f, 0.05f, -0.03f};
  constexpr static float PITCH_B[N_HARMONICS] = {0.2f, -0.3f, 0.1f, -0.02f, 0.04f};

  FourierTrajectoryGenerator<N_HARMONICS> traj_yaw_{BASE_FREQ_HZ, 0.0f, YAW_A, YAW_B};
  FourierTrajectoryGenerator<N_HARMONICS> traj_pitch_{BASE_FREQ_HZ, 0.0f, PITCH_A, PITCH_B};
};

#undef IGNORE_UNINTEREST_EVENT
#undef ENTER
#undef REACT

}  // namespace state

void Init();

}  // namespace fsm