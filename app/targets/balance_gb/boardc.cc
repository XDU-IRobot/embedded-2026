#include <librm.hpp>

#include "spi.h"
#include "usart.h"

#include "boardc.hpp"
#include "firstorderfilter.hpp"

static FirstOrderFilter g_zfilter(1.f / 500.f, 0.02f);

f32 g_z;

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
  imu->Update();
  g_zfilter.Update(imu->gyro_z()-0.001f);
  g_z = g_zfilter.value();
  ahrs.Update(rm::modules::ImuData6Dof{imu->gyro_y(), -imu->gyro_x(), g_z,
                                      imu->accel_y(),-imu->accel_x(), imu->accel_z()});
  roll = -ahrs.euler_angle().roll ;
  roll = roll*57.3f;
  yaw = -ahrs.euler_angle().yaw ;
  yaw = yaw*57.3f;
  pitch = -ahrs.euler_angle().pitch ;
  pitch = pitch*57.3f;
}