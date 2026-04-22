//
// Created by Asuna on 2026/4/15.
//

#ifndef BOARDC_PARAMETER_CONFIG_HPP
#define BOARDC_PARAMETER_CONFIG_HPP
namespace pid_config {
constexpr float kPitchPosP = 25.f;
constexpr float kPitchPosI = 0.8f;
constexpr float kPitchPosD = 130.f;
constexpr float kPitchSpdP = 0.6f;
constexpr float kPitchSpdI = 0.f;
constexpr float kPitchSpdD = 0.f;

constexpr float kYawPosP = 22.f;
constexpr float kYawPosI = 1.f;
constexpr float kYawPosD = 120.f;
constexpr float kYawSpdP = 0.6f;
constexpr float kYawSpdI = 0.f;
constexpr float kYawSpdD = 0.f;
}  // namespace pid_config

#endif  // BOARDC_PARAMETER_CONFIG_HPP
