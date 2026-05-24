#include "gimbal.hpp"
// 修改pid参数

void Gimbal::GimbalPIDInitAIM() {
  // yaw
  gimbal_controller.pid()
      .yaw_position.SetKp(250.0f)
      .SetKi(0.0f)
      .SetKd(12000.0f)
      .SetMaxOut(10000.0f)
      .SetMaxIout(1000.0f)
      .SetDiffLpfAlpha(0.01);
  gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(1000.0f);
  // pitch
  gimbal_controller.pid()
      .pitch_position.SetKp(30.0f)
      .SetKi(0.0f)
      .SetKd(500.0f)
      .SetMaxOut(500.0f)
      .SetMaxIout(10.0f)
      .SetDiffLpfAlpha(0.01);
  gimbal_controller.pid().pitch_speed.SetKp(1.0f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);
}
void Gimbal::GimbalPIDInitMAU() {
  // yaw
  gimbal_controller.pid()
      .yaw_position.SetKp(300.0f)
      .SetKi(0.0f)
      .SetKd(12000.0f)
      .SetMaxOut(10000.0f)
      .SetMaxIout(1000.0f)
      .SetDiffLpfAlpha(0.01);
  gimbal_controller.pid().yaw_speed.SetKp(350.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(1000.0f);
  // pitch
  gimbal_controller.pid()
      .pitch_position.SetKp(30.0f)
      .SetKi(0.0f)
      .SetKd(50.0f)
      .SetMaxOut(500.0f)
      .SetMaxIout(10.0f)
      .SetDiffLpfAlpha(0.01);
  gimbal_controller.pid().pitch_speed.SetKp(1.0f).SetKi(0.0f).SetKd(0.001f).SetMaxOut(10.0f).SetMaxIout(5.0f);
}

void Gimbal::AmmoPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
  shoot_controller.pid().fric_2_speed.SetKp(18.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(1000.0f);
  shoot_controller.pid().loader_speed.SetKp(15.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(20000.0f).SetMaxIout(2000.0f);
}
