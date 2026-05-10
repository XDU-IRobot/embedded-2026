#include "gimbal.hpp"
// 修改pid参数
void Gimbal::GimbalPIDInit() {
  gimbal_controller.pid().yaw_position.SetKp(160.0f).SetKi(0.0f).SetKd(0.01f).SetMaxOut(10000.0f).SetMaxIout(
      1000.0f);  // TODO yaw初版函数 160 0.0 0.01
  gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(1000.0f);
  // yaw原始参数 200 0 0.2  350 0 0

  gimbal_controller.pid().pitch_position.SetKp(35.0f).SetKi(0.00f).SetKd(0.01f).SetMaxOut(500.0f).SetMaxIout(
      10.0f);  // TODO pitch初版参数 35 0 0.01  20 0.001 0.001
  gimbal_controller.pid().pitch_speed.SetKp(0.8f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);
}  // 0.8 0.0 0.001  1.2 0.0 0.003

void Gimbal::AmmoPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
  shoot_controller.pid().fric_2_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
  shoot_controller.pid().loader_speed.SetKp(15.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(2000.0f);
}
