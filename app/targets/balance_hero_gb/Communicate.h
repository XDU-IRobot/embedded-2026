#ifndef REFEREEDATA_H
#define REFEREEDATA_H

#include "librm/device/can_device.hpp"

using namespace rm;
using namespace rm::device;

enum class ChassisState {
  UNABLE = 0x00,
  FOLLOW = 0x01,
  ROTATE = 0x02,
  JUMP = 0x03,
};

enum class LegLength {
  LOW = 0x00,
  NORMAL = 0x01,
  HIGH = 0x02,
};

struct ChassisCommand {
  ChassisState state : 4;
  LegLength leg_length : 4;
  int8_t move_x;
  int8_t move_y;
};

struct UICommand {
  uint8_t ui1;
  uint8_t ui2;
  uint8_t ui3;
  uint8_t ui4;
};

struct Command {
  ChassisCommand chassis;
  UICommand ui;
};

class Chassis_Command {
 public:
  Chassis_Command() = default;
  ~Chassis_Command() = default;
  Command _command{{ChassisState::UNABLE, LegLength::NORMAL, 0, 0}, {0, 0, 0, 0}};
  uint8_t speedset{0};
  void act();
};
extern Chassis_Command mychassis;

enum class POWER { CHASSIS = 0, GIMBAL = 1, AMMOBOOSTER = 2 };
struct ShooterCooling {
  i16 maxheat;
  i16 cooling;
};

struct ChassisReferee {
  u8 powerstate;  // 0:chassis, 1:gimbal, 2:ammo, 3-7:reserved
  u8 robot_id;    // 机器人ID
  u16 maxheat;    // 最大热量
  u16 cooling;    // 每秒冷却
  u16 realheat;   // 实时热量
  f32 speed;      // 实时弹速
};

class Communicate final : public CanDevice {
 public:
  explicit Communicate(hal::CanInterface &can);
  Communicate() = delete;
  ~Communicate() override = default;

  void RxCallback(const hal::CanFrame *msg) override;
  void SendChassisCommand();

 private:
  u8 tx_buf_[8]{0};

 public:
  ChassisReferee referee_;
};

class RefereeData {
 private:
  static constexpr ShooterCooling _heatpra[12]{{0, 0},    {200, 40}, {230, 48}, {260, 56},  {290, 64}, {320, 72},
                                               {350, 80}, {380, 88}, {410, 96}, {440, 104}, {500, 120}};

  bool _powerstate[3]{false, false, false};
  bool _prepowerstate[3]{false, false, false};

  u8 _robotid{0};
  u16 _maxheat{100};
  u16 _cooling{40};
  u16 _realheat{0};
  f32 _speed{0};

 public:
  RefereeData();
  ~RefereeData();

  void act(Communicate &communicate);

  [[nodiscard]] bool getPowerState(enum POWER id) const;

  [[nodiscard]] u8 getrobotid() const;
  [[nodiscard]] u16 getmaxheat() const;
  [[nodiscard]] u16 getcooling() const;
  [[nodiscard]] u16 getRealHeat() const;
  [[nodiscard]] f32 getspeed() const;
};

extern RefereeData refereedata;

#endif /* REFEREEDATA_H */