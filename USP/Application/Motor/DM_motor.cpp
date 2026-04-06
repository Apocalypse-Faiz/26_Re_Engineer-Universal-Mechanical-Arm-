#include "DM_motor.h"

namespace
{
constexpr float kDegToRad = PI / 180.0f;
}

void DM_Motor::bindTxQueueHandle(QueueHandle_t _txQueue)
{
    if (this->dm_motor)
    {
        this->dm_motor->bindCanQueueHandle(_txQueue);
    }
}

void DM_Motor::setPolar(int polar)
{
    polar_sign_ = (polar >= 0) ? 1 : -1;
}

void DM_Motor::enableMotor()
{
    if (dm_motor == nullptr)
    {
        return;
    }

    vTaskDelay(2);
    dm_motor->ClearError();
    vTaskDelay(2);
    dm_motor->startMotor();
    vTaskDelay(2);
}

HardwareFeedback DM_Motor::update()
{
    HardwareFeedback feedback;

    if (dm_motor == nullptr)
    {
        feedback.status = HardwareStatus::error;
        return feedback;
    }

    switch (status_)
    {
    case HardwareStatus::error:
        if (dm_motor->GetMotorState() == 1)
        {
            status_ = HardwareStatus::ready;
        }
        break;
    case HardwareStatus::ready:
        if (dm_motor->GetMotorState() != 1)
        {
            status_ = HardwareStatus::error;
        }
        break;
    default:
        status_ = HardwareStatus::error;
        break;
    }

    feedback.status = status_;
    if (status_ == HardwareStatus::ready)
    {
        const float sign = static_cast<float>(polar_sign_);
        const auto &rec = dm_motor->getRecData();
        feedback.current_angle = sign * rec.angle;
        feedback.current_speed = sign * rec.velocity;
        feedback.current_torque = sign * rec.torque;
    }

    return feedback;
}

void DM_Motor::setTorque(float target_torque)
{
    if (dm_motor == nullptr)
    {
        return;
    }
    dm_motor->setTorque(static_cast<float>(polar_sign_) * target_torque);
}

void DM_Motor::setMIT(const ImpedanceCommand &motor_cmd)
{
    if (dm_motor == nullptr)
    {
        return;
    }

    const float sign = static_cast<float>(polar_sign_);
    dm_motor->control(sign * motor_cmd.q_des * kDegToRad,
                      sign * motor_cmd.dq_des,
                      motor_cmd.kp,
                      motor_cmd.kd,
                      sign * motor_cmd.tau_ff);
}

void DM_Motor::disable()
{
    if (dm_motor == nullptr)
    {
        return;
    }
    dm_motor->stopMotor();
}
