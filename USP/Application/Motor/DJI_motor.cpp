#include "DJI_motor.h"

namespace
{
constexpr unsigned int kDjiRecIdMin = 0x201U;
constexpr unsigned int kDjiRecIdMax = 0x208U;
volatile TickType_t g_dji_last_rx_tick[kDjiRecIdMax - kDjiRecIdMin + 1] = {0};

int recIdToIndex(unsigned int can_rec_id)
{
    if (can_rec_id < kDjiRecIdMin || can_rec_id > kDjiRecIdMax)
    {
        return -1;
    }
    return static_cast<int>(can_rec_id - kDjiRecIdMin);
}

TickType_t getLastRxTick(unsigned int can_rec_id)
{
    const int idx = recIdToIndex(can_rec_id);
    if (idx < 0)
    {
        return 0;
    }
    return g_dji_last_rx_tick[idx];
}
}

void DjiMotor_OnCanFeedback(unsigned int can_rec_id)
{
    const int idx = recIdToIndex(can_rec_id);
    if (idx < 0)
    {
        return;
    }
    // 在接收路径打点，供 update() 判断在线状态。
    g_dji_last_rx_tick[idx] = xTaskGetTickCount();
}

void DJI_Motor_2006::bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort)
{
    this->motor_msg = _motor_msg;
    this->CAN_TxPort = _CAN_TxPort;
    // 对于DJI电机，不需要绑定发送队列，会在执行任务中放到统一一个包中对多个电机进行发送
}

void DJI_Motor_2006::setPolar(int polar)
{
    polar_sign_ = (polar >= 0) ? 1 : -1;
}

void DJI_Motor_2006::setConversionScale(float speed_to_rad_s,
                                        float current_to_torque,
                                        float torque_to_out)
{
    speed_to_rad_s_scale_ = speed_to_rad_s;
    current_to_torque_scale_ = current_to_torque;
    torque_to_out_scale_ = torque_to_out;
}

void DJI_Motor_2006::setOfflineTimeoutMs(unsigned int timeout_ms)
{
    offline_timeout_tick_ = pdMS_TO_TICKS(timeout_ms);
}

HardwareFeedback DJI_Motor_2006::update()
{
    HardwareFeedback feedback;

    if (dji_motor == nullptr)
    {
        feedback.status = HardwareStatus::error;
        return feedback;
    }

    const TickType_t now_tick = xTaskGetTickCount();
    const TickType_t last_rx_tick = getLastRxTick(dji_motor->getRecID());

    // DJI 无独立硬件状态位：用“最近反馈超时”判在线。
    if (last_rx_tick == 0 || (now_tick - last_rx_tick) > offline_timeout_tick_)
    {
        status_ = HardwareStatus::error;
    }
    else
    {
        status_ = HardwareStatus::ready;
        feedback.current_angle = static_cast<float>(polar_sign_) * dji_motor->getAngle();
        feedback.current_speed = static_cast<float>(polar_sign_) *
                                 (static_cast<float>(dji_motor->getSpeed()) * speed_to_rad_s_scale_);
        feedback.current_torque = static_cast<float>(polar_sign_) *
                                  (static_cast<float>(dji_motor->getTorqueCurrent()) * current_to_torque_scale_);
    }

    feedback.status = status_;
    if (status_ == HardwareStatus::ready)
    {
        last_feedback_ = feedback;
    }
    else
    {
        // 离线时保持上一帧反馈，避免上层状态突跳。
        feedback.current_angle = last_feedback_.current_angle;
        feedback.current_speed = last_feedback_.current_speed;
        feedback.current_torque = last_feedback_.current_torque;
    }
    return feedback;
}

void DJI_Motor_2006::setTorque(float target_torque)
{
    if (dji_motor == nullptr)
    {
        return;
    }

    const float out = static_cast<float>(polar_sign_) * target_torque * torque_to_out_scale_;
    dji_motor->Out = std_lib::constrain(out, -dji_motor->MAX_CURRENT(), dji_motor->MAX_CURRENT());

    // 包装can信息和发送队列放在外面
    // motor_dji::MotorMsgPack(this->motor_msg, *dji_motor);
    // xQueueSend(this->CAN_TxPort, &this->motor_msg.Id2ff, 0);
}

void DJI_Motor_2006::setMIT(const ImpedanceCommand &motor_cmd)
{
    // DJI 仅支持力矩输出：在适配器内模拟 MIT 模式。
    const float tau = motor_cmd.kp * (motor_cmd.q_des - last_feedback_.current_angle) +
                      motor_cmd.kd * (motor_cmd.dq_des - last_feedback_.current_speed) +
                      motor_cmd.tau_ff;
    setTorque(tau);
}

void DJI_Motor_2006::disable()
{
    if (dji_motor == nullptr)
    {
        return;
    }
    dji_motor->Out = 0.0f;
}

void DJI_Motor_3508::bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort)
{
    this->motor_msg = _motor_msg;
    this->CAN_TxPort = _CAN_TxPort;
    // 对于DJI电机，不需要绑定发送队列，会在执行任务中放到统一一个包中对多个电机进行发送
}

void DJI_Motor_3508::setPolar(int polar)
{
    polar_sign_ = (polar >= 0) ? 1 : -1;
}

void DJI_Motor_3508::setConversionScale(float speed_to_rad_s,
                                        float current_to_torque,
                                        float torque_to_out)
{
    speed_to_rad_s_scale_ = speed_to_rad_s;
    current_to_torque_scale_ = current_to_torque;
    torque_to_out_scale_ = torque_to_out;
}

void DJI_Motor_3508::setOfflineTimeoutMs(unsigned int timeout_ms)
{
    offline_timeout_tick_ = pdMS_TO_TICKS(timeout_ms);
}

HardwareFeedback DJI_Motor_3508::update()
{
    HardwareFeedback feedback;

    if (dji_motor == nullptr)
    {
        feedback.status = HardwareStatus::error;
        return feedback;
    }

    const TickType_t now_tick = xTaskGetTickCount();
    const TickType_t last_rx_tick = getLastRxTick(dji_motor->getRecID());

    // DJI 无独立硬件状态位：用“最近反馈超时”判在线。
    if (last_rx_tick == 0 || (now_tick - last_rx_tick) > offline_timeout_tick_)
    {
        status_ = HardwareStatus::error;
    }
    else
    {
        status_ = HardwareStatus::ready;
        feedback.current_angle = static_cast<float>(polar_sign_) * dji_motor->getAngle();
        feedback.current_speed = static_cast<float>(polar_sign_) *
                                 (static_cast<float>(dji_motor->getSpeed()) * speed_to_rad_s_scale_);
        feedback.current_torque = static_cast<float>(polar_sign_) *
                                  (static_cast<float>(dji_motor->getTorqueCurrent()) * current_to_torque_scale_);
    }

    feedback.status = status_;
    if (status_ == HardwareStatus::ready)
    {
        last_feedback_ = feedback;
    }
    else
    {
        // 离线时保持上一帧反馈，避免上层状态突跳。
        feedback.current_angle = last_feedback_.current_angle;
        feedback.current_speed = last_feedback_.current_speed;
        feedback.current_torque = last_feedback_.current_torque;
    }
    return feedback;
}

void DJI_Motor_3508::setTorque(float target_torque)
{
    if (dji_motor == nullptr)
    {
        return;
    }

    const float out = static_cast<float>(polar_sign_) * target_torque * torque_to_out_scale_;
    dji_motor->Out = std_lib::constrain(out, -dji_motor->MAX_CURRENT(), dji_motor->MAX_CURRENT());
}

void DJI_Motor_3508::setMIT(const ImpedanceCommand &motor_cmd)
{
    const float tau = motor_cmd.kp * (motor_cmd.q_des - last_feedback_.current_angle) +
                      motor_cmd.kd * (motor_cmd.dq_des - last_feedback_.current_speed) +
                      motor_cmd.tau_ff;
    setTorque(tau);
}

void DJI_Motor_3508::disable()
{
    if (dji_motor == nullptr)
    {
        return;
    }
    dji_motor->Out = 0.0f;
}
