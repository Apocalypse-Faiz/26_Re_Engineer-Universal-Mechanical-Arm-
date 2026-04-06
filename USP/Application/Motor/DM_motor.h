#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "SRML.h"
#include "Motor.h"

/**
 * @brief 达妙电机硬件适配器（电机侧）。
 */
class DM_Motor : public HardwareInterface
{
private:
    Motor_DM_classdef *dm_motor = nullptr;
    HardwareStatus status_ = HardwareStatus::error;
    std::int8_t polar_sign_ = 1;

public:
    explicit DM_Motor(Motor_DM_classdef *dm_motor)
    {
        if (dm_motor)
        {
            this->dm_motor = dm_motor;
        }
        status_ = HardwareStatus::error;
    }
    ~DM_Motor() override
    {
    }

    /**
     * @brief 绑定 DM 电机发送队列。
     */
    void bindTxQueueHandle(QueueHandle_t _txQueue);

    /**
     * @brief 设置方向极性。
     */
    void setPolar(int polar);

    /**
     * @brief 使能电机（清错 + 启动）。
     */
    void enableMotor();

    HardwareFeedback update() override;
    void setTorque(float target_torque) override;
    void setMIT(const ImpedanceCommand &motor_cmd) override;
    void disable() override;
    bool supportsMIT() const override { return true; }
};

#endif
