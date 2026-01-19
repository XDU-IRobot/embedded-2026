#include <librm.hpp>

#include "spi.h"
#include "usart.h"

#include "boardc.hpp"

f32 gyro_x;
f32 gyro_y;
f32 gyro_z;

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
  buzzer_controller.Play<modules::buzzer_melody::Startup>();
}

void BoardC::EulerUpdate() {
  gyro_x = imu->gyro_x();
  gyro_y = imu->gyro_y();
  gyro_z = imu->gyro_z();
  imu->Update();
  ahrs.Update(rm::modules::ImuData6Dof{imu->gyro_y(), -imu->gyro_x(), imu->gyro_z(),
                                      imu->accel_y(),-imu->accel_x(), imu->accel_z()});
  roll = -ahrs.euler_angle().roll ;
  roll = roll*57.3f;
  yaw = -ahrs.euler_angle().yaw ;
  yaw = yaw*57.3f;
  pitch = -ahrs.euler_angle().pitch ;
  pitch = pitch*57.3f;
}