#ifndef UI_HPP
#define UI_HPP

#include "drone_gb_new/gimbal.hpp"
#include "protocol_user.hpp"
#include "referee_user.hpp"
#include "TaskScheduler.hpp"

void Layer0_func();
void Layer1_func();
static rm::device::UITask Layer0 = rm::device::UITask(Layer0_func, 10);
static rm::device::UITask Layer1 = rm::device::UITask(Layer1_func, 1);
static rm::device::UITaskScheduler schedule = rm::device::UITaskScheduler(30);

#endif  // UI_HPP


