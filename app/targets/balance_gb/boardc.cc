#include <librm.hpp>

#include "spi.h"
#include "usart.h"

#include "boardc.hpp"

void BoardC::BoardcInit() {
  buzzer = new Buzzer;
  led = new LED;

  dbus = new rm::hal::Serial{huart3, 18, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
  imu = new rm::device::BMI088{hspi1, CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, CS1_GYRO_GPIO_Port, CS1_GYRO_Pin};
  rc = new rm::device::DR16{*dbus};

  device_rc << rc;

  rc->Begin();
  buzzer->Init();
  led->Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::TheLick>();
}

void BoardC::EulerUpdate() {
  // imu处理
  imu->Update();
  ahrs.Update(rm::modules::ImuData6Dof{-imu->gyro_x(), -imu->gyro_y(), imu->gyro_z(), -imu->accel_x(), -imu->accel_y(),
                                       imu->accel_z()});
  roll = -ahrs.euler_angle().roll + M_PI;
  roll = roll * 57.3f;
  yaw = -ahrs.euler_angle().yaw + M_PI;
  yaw = yaw * 57.3f;
  pitch = -ahrs.euler_angle().pitch + M_PI;
  pitch = pitch * 57.3f;
}