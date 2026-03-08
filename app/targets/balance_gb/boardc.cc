#include <librm.hpp>

#include "spi.h"
#include "usart.h"
#include "global.hpp"
#include "communiate.hpp"

#include "boardc.hpp"
#include "firstorderfilter.hpp"

static FirstOrderFilter g_zfilter(1.f / 500.f, 0.02f);
extern rm::hal::Can *can1;
f32 g_z;

void BoardC::BoardcInit() {
  buzzer = new Buzzer;
  led = new LED;

  dbus = new hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  tc_serial = new hal::Serial{huart6, 128, rm::hal::stm32::UartMode::kDma, rm::hal::stm32::UartMode::kDma};
  imu = new BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rc = new DR16{*dbus};
  tc_receiver = new TcReceiver{*tc_serial};
  device_rc << rc;
  rc->Begin();
  tc_receiver->Begin();
  buzzer->Init();
  led->Init();
  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  hipnuc_serial =  new rm::hal::Serial{huart1, 518, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  hipnuc_imu = new rm::device::HipnucImu{*hipnuc_serial};
  hipnuc_imu->Begin();

  for (auto ch : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
    HAL_TIM_PWM_Start(&htim1, ch);
  }
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

void BoardC::EulerUpdate() {
  // imu->Update();
  // g_zfilter.Update(imu->gyro_z() - 0.001f);
  // g_z = g_zfilter.value();
  // ahrs.Update(
  //     rm::modules::ImuData6Dof{imu->gyro_y(), -imu->gyro_x(), g_z, imu->accel_y(), -imu->accel_x(), imu->accel_z()});
  // roll = -ahrs.euler_angle().roll;
  // roll = roll * 57.3f;
  // yaw = -ahrs.euler_angle().yaw;
  // yaw = yaw * 57.3f;
  // pitch = -ahrs.euler_angle().pitch;
  // pitch = pitch * 57.3f;
  roll = hipnuc_imu->roll() * 57.3f;
  yaw = -hipnuc_imu->yaw() * 57.3f;
  pitch = -hipnuc_imu->pitch() * 57.3f;
}