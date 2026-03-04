/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include <gtest/gtest.h>

#include "librm.hpp"

using namespace rm::modules;

// 定义测试用的按键掩码
namespace TestKeys {
constexpr uint32_t kKeyA = 1U << 0;
constexpr uint32_t kKeyB = 1U << 1;
constexpr uint32_t kKeyC = 1U << 2;
constexpr uint32_t kKeyD = 1U << 3;

// Modifier 键
constexpr uint32_t kCtrl = 1U << 16;
constexpr uint32_t kShift = 1U << 17;
constexpr uint32_t kAlt = 1U << 18;

constexpr uint32_t kAllModifiers = kCtrl | kShift | kAlt;
}  // namespace TestKeys

class KeyboardHandlerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    key_down_count_ = 0;
    key_up_count_ = 0;
    last_event_ = {};
  }

  int key_down_count_ = 0;
  int key_up_count_ = 0;
  KeyEvent last_event_{};
};

// 测试构造函数和 modifier_mask 获取
TEST_F(KeyboardHandlerTest, ConstructorAndModifierMask) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);
  EXPECT_EQ(handler.modifier_mask(), TestKeys::kAllModifiers);
}

// 测试 SetModifierMask 方法
TEST_F(KeyboardHandlerTest, SetModifierMask) {
  KeyboardHandler<> handler(0);
  handler.SetModifierMask(TestKeys::kAllModifiers);
  EXPECT_EQ(handler.modifier_mask(), TestKeys::kAllModifiers);
}

// 测试链式调用
TEST_F(KeyboardHandlerTest, ChainedCalls) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  // 验证链式调用返回自身引用
  auto& ref = handler.SetModifierMask(TestKeys::kCtrl);
  EXPECT_EQ(&ref, &handler);

  auto& ref2 = handler.OnKeyDown(TestKeys::kKeyA, [](const KeyEvent&) {});
  EXPECT_EQ(&ref2, &handler);

  auto& ref3 = handler.OnKeyUp(TestKeys::kKeyA, [](const KeyEvent&) {});
  EXPECT_EQ(&ref3, &handler);
}

// 测试单键按下事件
TEST_F(KeyboardHandlerTest, SingleKeyDown) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent& event) {
    key_down_count_++;
    last_event_ = event;
  });

  // 初始状态
  handler.Update(0);

  // 按下 A 键
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);
  EXPECT_EQ(last_event_.key, TestKeys::kKeyA);
  EXPECT_EQ(last_event_.modifiers, 0U);
  EXPECT_EQ(last_event_.type, KeyEvent::Type::kKeyDown);
}

// 测试单键释放事件
TEST_F(KeyboardHandlerTest, SingleKeyUp) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyUp(TestKeys::kKeyA, [this](const KeyEvent& event) {
    key_up_count_++;
    last_event_ = event;
  });

  // 按下 A 键
  handler.Update(TestKeys::kKeyA);

  // 释放 A 键
  handler.Update(0);

  EXPECT_EQ(key_up_count_, 1);
  EXPECT_EQ(last_event_.key, TestKeys::kKeyA);
  EXPECT_EQ(last_event_.modifiers, 0U);
  EXPECT_EQ(last_event_.type, KeyEvent::Type::kKeyUp);
}

// 测试按键持续按下不会重复触发
TEST_F(KeyboardHandlerTest, KeyHeldNoRepeat) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });

  handler.Update(0);
  handler.Update(TestKeys::kKeyA);  // 按下
  handler.Update(TestKeys::kKeyA);  // 持续按住
  handler.Update(TestKeys::kKeyA);  // 持续按住

  EXPECT_EQ(key_down_count_, 1);  // 只触发一次
}

// 测试带 Ctrl 的组合键
TEST_F(KeyboardHandlerTest, CtrlModifierKeyDown) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, TestKeys::kCtrl, [this](const KeyEvent& event) {
    key_down_count_++;
    last_event_ = event;
  });

  handler.Update(0);

  // 先按下 Ctrl
  handler.Update(TestKeys::kCtrl);

  // 再按下 A（Ctrl+A）
  handler.Update(TestKeys::kCtrl | TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);
  EXPECT_EQ(last_event_.key, TestKeys::kKeyA);
  EXPECT_EQ(last_event_.modifiers, TestKeys::kCtrl);
}

// 测试 Ctrl+Shift 组合键
TEST_F(KeyboardHandlerTest, CtrlShiftModifierKeyDown) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, TestKeys::kCtrl | TestKeys::kShift, [this](const KeyEvent& event) {
    key_down_count_++;
    last_event_ = event;
  });

  handler.Update(0);

  // 按下 Ctrl+Shift+A
  handler.Update(TestKeys::kCtrl | TestKeys::kShift | TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);
  EXPECT_EQ(last_event_.modifiers, TestKeys::kCtrl | TestKeys::kShift);
}

// 测试 modifier 精确匹配：Ctrl+A 绑定不应被 Ctrl+Shift+A 触发
TEST_F(KeyboardHandlerTest, ModifierExactMatch) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, TestKeys::kCtrl, [this](const KeyEvent&) { key_down_count_++; });

  handler.Update(0);

  // 按下 Ctrl+Shift+A（比绑定的 Ctrl+A 多了一个 modifier）
  handler.Update(TestKeys::kCtrl | TestKeys::kShift | TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 0);  // 不应触发
}

// 测试无 modifier 的单键在有 modifier 按下时不触发
TEST_F(KeyboardHandlerTest, SingleKeyNotTriggeredWithModifier) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });

  handler.Update(0);

  // 按下 Ctrl+A
  handler.Update(TestKeys::kCtrl | TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 0);  // 单键绑定不应被触发
}

// 测试多个绑定优先级（更多 modifier 的优先）
TEST_F(KeyboardHandlerTest, BindingPriority) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  int single_key_count = 0;
  int ctrl_key_count = 0;
  int ctrl_shift_key_count = 0;

  handler.OnKeyDown(TestKeys::kKeyA, [&single_key_count](const KeyEvent&) { single_key_count++; });

  handler.OnKeyDown(TestKeys::kKeyA, TestKeys::kCtrl, [&ctrl_key_count](const KeyEvent&) { ctrl_key_count++; });

  handler.OnKeyDown(TestKeys::kKeyA, TestKeys::kCtrl | TestKeys::kShift,
                    [&ctrl_shift_key_count](const KeyEvent&) { ctrl_shift_key_count++; });

  handler.Update(0);

  // 测试 Ctrl+Shift+A：只触发最匹配的
  handler.Update(TestKeys::kCtrl | TestKeys::kShift | TestKeys::kKeyA);

  EXPECT_EQ(ctrl_shift_key_count, 1);
  EXPECT_EQ(ctrl_key_count, 0);
  EXPECT_EQ(single_key_count, 0);
}

// 测试多个不同按键的绑定
TEST_F(KeyboardHandlerTest, MultipleKeyBindings) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  int key_a_count = 0;
  int key_b_count = 0;

  handler.OnKeyDown(TestKeys::kKeyA, [&key_a_count](const KeyEvent&) { key_a_count++; });

  handler.OnKeyDown(TestKeys::kKeyB, [&key_b_count](const KeyEvent&) { key_b_count++; });

  handler.Update(0);
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_a_count, 1);
  EXPECT_EQ(key_b_count, 0);

  handler.Update(TestKeys::kKeyA | TestKeys::kKeyB);  // A 保持，B 按下

  EXPECT_EQ(key_a_count, 1);  // A 没有重复触发
  EXPECT_EQ(key_b_count, 1);  // B 触发
}

// 测试 KeyUp 时使用按下时的 modifier 状态
TEST_F(KeyboardHandlerTest, KeyUpUsesModifierWhenPressed) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyUp(TestKeys::kKeyA, TestKeys::kCtrl, [this](const KeyEvent& event) {
    key_up_count_++;
    last_event_ = event;
  });

  handler.Update(0);

  // Ctrl+A 按下
  handler.Update(TestKeys::kCtrl | TestKeys::kKeyA);

  // 先释放 Ctrl
  handler.Update(TestKeys::kKeyA);

  // 再释放 A
  handler.Update(0);

  // 应该触发，因为 A 是在 Ctrl 按下时按下的
  EXPECT_EQ(key_up_count_, 1);
  EXPECT_EQ(last_event_.modifiers, TestKeys::kCtrl);
}

// 测试 Reset 方法
TEST_F(KeyboardHandlerTest, Reset) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });

  handler.Update(0);
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);

  // 重置状态
  handler.Reset();

  // 再次更新相同状态，应该重新触发（因为状态被重置了）
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 2);
}

// 测试 ClearBindings 方法
TEST_F(KeyboardHandlerTest, ClearBindings) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });

  handler.Update(0);
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);

  // 清除绑定
  handler.ClearBindings();
  handler.Reset();

  // 再次触发
  handler.Update(0);
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);  // 没有增加，因为绑定已清除
}

// 测试同时按下多个键
TEST_F(KeyboardHandlerTest, SimultaneousKeys) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  int key_a_down = 0;
  int key_b_down = 0;
  int key_a_up = 0;
  int key_b_up = 0;

  handler.OnKeyDown(TestKeys::kKeyA, [&key_a_down](const KeyEvent&) { key_a_down++; });
  handler.OnKeyDown(TestKeys::kKeyB, [&key_b_down](const KeyEvent&) { key_b_down++; });
  handler.OnKeyUp(TestKeys::kKeyA, [&key_a_up](const KeyEvent&) { key_a_up++; });
  handler.OnKeyUp(TestKeys::kKeyB, [&key_b_up](const KeyEvent&) { key_b_up++; });

  handler.Update(0);

  // 同时按下 A 和 B
  handler.Update(TestKeys::kKeyA | TestKeys::kKeyB);

  EXPECT_EQ(key_a_down, 1);
  EXPECT_EQ(key_b_down, 1);

  // 同时释放 A 和 B
  handler.Update(0);

  EXPECT_EQ(key_a_up, 1);
  EXPECT_EQ(key_b_up, 1);
}

// 测试自定义模板参数
TEST_F(KeyboardHandlerTest, CustomTemplateParameters) {
  // 使用较小的绑定数量
  KeyboardHandler<4, 2, true> handler(TestKeys::kAllModifiers);

  int count = 0;
  handler.OnKeyDown(TestKeys::kKeyA, [&count](const KeyEvent&) { count++; });
  handler.OnKeyDown(TestKeys::kKeyB, [&count](const KeyEvent&) { count++; });
  handler.OnKeyDown(TestKeys::kKeyC, [&count](const KeyEvent&) { count++; });
  handler.OnKeyDown(TestKeys::kKeyD, [&count](const KeyEvent&) { count++; });

  handler.Update(0);
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(count, 1);

  handler.Update(TestKeys::kKeyA | TestKeys::kKeyB);

  EXPECT_EQ(count, 2);
}

// 测试 UseStdFunction = false（使用 etl::delegate）
// 辅助函数用于 etl::delegate 测试
namespace {
int g_etl_delegate_count = 0;
void EtlDelegateTestCallback(const KeyEvent&) { g_etl_delegate_count++; }
}  // namespace

TEST_F(KeyboardHandlerTest, EtlDelegateCallback) {
  KeyboardHandler<32, 8, false> handler(TestKeys::kAllModifiers);

  g_etl_delegate_count = 0;

  // 使用 etl::delegate 绑定自由函数
  handler.OnKeyDown(TestKeys::kKeyA,
                    etl::delegate<void(const KeyEvent&)>::create<EtlDelegateTestCallback>());

  handler.Update(0);
  handler.Update(TestKeys::kKeyA);

  EXPECT_EQ(g_etl_delegate_count, 1);
}

// 测试按键快速按下释放
TEST_F(KeyboardHandlerTest, RapidPressRelease) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });
  handler.OnKeyUp(TestKeys::kKeyA, [this](const KeyEvent&) { key_up_count_++; });

  handler.Update(0);

  // 快速按下释放多次
  for (int i = 0; i < 5; i++) {
    handler.Update(TestKeys::kKeyA);
    handler.Update(0);
  }

  EXPECT_EQ(key_down_count_, 5);
  EXPECT_EQ(key_up_count_, 5);
}

// 测试 modifier 键本身不触发普通键回调
TEST_F(KeyboardHandlerTest, ModifierKeyAlone) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  handler.OnKeyDown(TestKeys::kCtrl, [this](const KeyEvent&) { key_down_count_++; });

  handler.Update(0);
  handler.Update(TestKeys::kCtrl);

  // modifier 键被跳过，不触发普通键的处理逻辑
  EXPECT_EQ(key_down_count_, 0);
}

// 测试 KeyEvent 结构体的内容
TEST_F(KeyboardHandlerTest, KeyEventContent) {
  KeyboardHandler<> handler(TestKeys::kAllModifiers);

  KeyEvent captured_event{};

  handler.OnKeyDown(TestKeys::kKeyA, TestKeys::kCtrl | TestKeys::kShift,
                    [&captured_event](const KeyEvent& event) { captured_event = event; });

  handler.Update(0);
  handler.Update(TestKeys::kCtrl | TestKeys::kShift | TestKeys::kKeyA);

  EXPECT_EQ(captured_event.key, TestKeys::kKeyA);
  EXPECT_EQ(captured_event.modifiers, TestKeys::kCtrl | TestKeys::kShift);
  EXPECT_EQ(captured_event.type, KeyEvent::Type::kKeyDown);
}

// 测试拷贝构造函数
TEST_F(KeyboardHandlerTest, CopyConstructor) {
  KeyboardHandler<> handler1(TestKeys::kAllModifiers);

  handler1.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });

  KeyboardHandler<> handler2 = handler1;

  EXPECT_EQ(handler2.modifier_mask(), TestKeys::kAllModifiers);

  handler2.Update(0);
  handler2.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);
}

// 测试拷贝赋值运算符
TEST_F(KeyboardHandlerTest, CopyAssignment) {
  KeyboardHandler<> handler1(TestKeys::kAllModifiers);
  KeyboardHandler<> handler2(0);

  handler1.OnKeyDown(TestKeys::kKeyA, [this](const KeyEvent&) { key_down_count_++; });

  handler2 = handler1;

  EXPECT_EQ(handler2.modifier_mask(), TestKeys::kAllModifiers);

  handler2.Update(0);
  handler2.Update(TestKeys::kKeyA);

  EXPECT_EQ(key_down_count_, 1);
}
