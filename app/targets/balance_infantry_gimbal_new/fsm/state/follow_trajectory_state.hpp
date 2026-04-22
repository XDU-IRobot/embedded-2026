#pragma once

#include "../fsm_common.hpp"

namespace fsm {
    namespace state {

        struct FollowTrajectory : etl::fsm_state<Gimbal, FollowTrajectory, StateId::kFollowTrajectory,  //
                event::ForceModeSwitch,              //
                event::ControlLoop>
        {
            etl::fsm_state_id_t on_enter_state();
            etl::fsm_state_id_t on_event(const event::ControlLoop& e);
            etl::fsm_state_id_t on_event(const event::ForceModeSwitch& e);
            etl::fsm_state_id_t on_event_unknown(const etl::imessage&);
        private:
            static void ReportData(     //
                    uint32_t timestamp_ms,  //
                    float tau_yaw,          //
                    float q_yaw,            //
                    float dq_yaw,           //
                    float tau_pitch,        //
                    float q_pitch,          //
                    float dq_pitch          //
            ) {
                static char tx_buf[500];
                sprintf(tx_buf, "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", timestamp_ms, tau_yaw, q_yaw, dq_yaw, tau_pitch, q_pitch,
                        dq_pitch);
                while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX) {
                    // 等待上一次传输完成
                }
                HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t *>(tx_buf), strlen(tx_buf));
            }

            uint32_t enter_timestamp_ms_{0u};  ///< 进入状态的时间戳，单位毫秒
            int loop_divisor_{0};

            float yaw_target_{0.f}, pitch_target_{0.f};

            float pitch_error_{0.f};

            float pitch_ff_{0.f}, yaw_ff_{0.f};

            // 跑5项傅里叶级数
            constexpr static size_t N_HARMONICS = 5;
            constexpr static float BASE_FREQ_HZ = 0.17f;  // 10秒一个大周期

            // 为 Yaw 轴和 Pitch 轴设置不相同的 a, b 系数，防止多轴线性相关
            constexpr static float YAW_A[N_HARMONICS] = {0.5f, -0.2f, 0.1f, -0.05f, 0.02f};
            constexpr static float YAW_B[N_HARMONICS] = {-0.3f, 0.4f, -0.15f, 0.08f, -0.01f};

            constexpr static float PITCH_A[N_HARMONICS] = {0.4f, 0.1f, -0.2f, 0.05f, -0.03f};
            constexpr static float PITCH_B[N_HARMONICS] = {0.2f, -0.3f, 0.1f, -0.02f, 0.04f};

            FourierTrajectoryGenerator<N_HARMONICS> traj_yaw_{BASE_FREQ_HZ, 0.0f, YAW_A, YAW_B};
            FourierTrajectoryGenerator<N_HARMONICS> traj_pitch_{BASE_FREQ_HZ, 0.0f, PITCH_A, PITCH_B};
        };


    }  // namespace state
}  // namespace fsm
