#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "SRML.h"
#include "Motor.h"
#include <stdint.h>

/**
 * @brief 记录 DJI 电机反馈帧到达事件。
 * @note 在 CAN 接收任务中，当某个 DJI 电机 update(...) 返回 true 时调用。
 * @param can_rec_id 反馈帧标准 ID。
 */
void DjiMotor_OnCanFeedback(unsigned int can_rec_id);

/**
 * @brief DJI C610 电机硬件适配器（电机侧）。
 */
class DJI_Motor_2006 : public HardwareInterface
{
private:
    Motor_C610 *dji_motor = nullptr;
    HardwareStatus status_ = HardwareStatus::error;
    std::int8_t polar_sign_ = 1;

    // DJI 原始反馈速度通常为 rpm，默认换算到 rad/s。
    float speed_to_rad_s_scale_ = (2.0f * PI) / 60.0f;
    // 力矩电流到 N·m 的标定比例，默认 1，建议按实机标定。
    float current_to_torque_scale_ = 1.0f;
    // N·m 到 ESC Out 的比例，默认 1，建议按实机标定。
    float torque_to_out_scale_ = 1.0f;
    TickType_t offline_timeout_tick_ = pdMS_TO_TICKS(20);

    HardwareFeedback last_feedback_;

public:
    explicit DJI_Motor_2006(Motor_C610 *dji_motor)
    {
        if (dji_motor)
        {
            this->dji_motor = dji_motor;
        }
        status_ = HardwareStatus::error;
    }

    /**
     * @brief 绑定 CAN 打包结构与发送队列句柄。
     */
    void bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort);

    /**
     * @brief 设置方向极性。
     */
    void setPolar(int polar);

    /**
     * @brief 设置单位换算比例。
     */
    void setConversionScale(float speed_to_rad_s,
                            float current_to_torque,
                            float torque_to_out);

    /**
     * @brief 设置离线超时（无反馈判错）。
     */
    void setOfflineTimeoutMs(unsigned int timeout_ms);

    HardwareFeedback update() override;
    void setTorque(float target_torque) override;
    void setMIT(const ImpedanceCommand &motor_cmd) override;
    void disable() override;
    bool supportsMIT() const override { return false; }

    Motor_CAN_COB motor_msg;
    QueueHandle_t CAN_TxPort;
};

/**
 * @brief DJI C620 电机硬件适配器（电机侧）。
 */
class DJI_Motor_3508 : public HardwareInterface
{
private:
    Motor_C620 *dji_motor = nullptr;
    HardwareStatus status_ = HardwareStatus::error;
    std::int8_t polar_sign_ = 1;
    float speed_to_rad_s_scale_ = (2.0f * PI) / 60.0f;
    float current_to_torque_scale_ = 1.0f;
    float torque_to_out_scale_ = 1.0f;
    TickType_t offline_timeout_tick_ = pdMS_TO_TICKS(20);
    HardwareFeedback last_feedback_;

public:
    explicit DJI_Motor_3508(Motor_C620 *dji_motor)
    {
        if (dji_motor)
        {
            this->dji_motor = dji_motor;
        }
        status_ = HardwareStatus::error;
    }

    /**
     * @brief 绑定 CAN 打包结构与发送队列句柄。
     */
    void bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort);

    /**
     * @brief 设置方向极性。
     */
    void setPolar(int polar);

    /**
     * @brief 设置单位换算比例。
     */
    void setConversionScale(float speed_to_rad_s,
                            float current_to_torque,
                            float torque_to_out);

    /**
     * @brief 设置离线超时（无反馈判错）。
     */
    void setOfflineTimeoutMs(unsigned int timeout_ms);

    HardwareFeedback update() override;
    void setTorque(float target_torque) override;
    void setMIT(const ImpedanceCommand &motor_cmd) override;
    void disable() override;
    bool supportsMIT() const override { return false; }

    Motor_CAN_COB motor_msg;
    QueueHandle_t CAN_TxPort;
};

#endif
