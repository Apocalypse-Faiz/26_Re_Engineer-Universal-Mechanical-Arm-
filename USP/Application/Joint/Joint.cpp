#include "Joint.h"

#include "../Transmission/Transmission.h"

Joint::Joint(float min_q, float max_q)
    : min_q_(min_q), max_q_(max_q)
{
    if (min_q_ > max_q_)
    {
        const float tmp = min_q_;
        min_q_ = max_q_;
        max_q_ = tmp;
    }
}

void Joint::bindTransmission(Transmission *transmission)
{
    transmission_ = transmission;
}

void Joint::setSoftLimit(float min_q, float max_q)
{
    min_q_ = min_q;
    max_q_ = max_q;
    if (min_q_ > max_q_)
    {
        const float tmp = min_q_;
        min_q_ = max_q_;
        max_q_ = tmp;
    }

    q_des_ = clampPosition(q_des_);
}

float Joint::getMinLimit() const
{
    return min_q_;
}

float Joint::getMaxLimit() const
{
    return max_q_;
}

void Joint::setTargetPosition(float q_d)
{
    q_des_ = clampPosition(q_d);
}

void Joint::setTargetAngle(float q_d)
{
    setTargetPosition(q_d);
}

void Joint::setTargetVelocity(float dq_d)
{
    dq_des_ = dq_d;
}

void Joint::setTargetTorqueFF(float tau_ff)
{
    tau_ff_cmd_ = tau_ff;
}

void Joint::setImpedanceTarget(float q_d, float dq_d, float kp, float kd, float tau_ff)
{
    q_des_ = clampPosition(q_d);
    dq_des_ = dq_d;
    impedance_kp_ = kp;
    impedance_kd_ = kd;
    tau_ff_cmd_ = tau_ff;
}

void Joint::setGravityComp(float gravity_comp)
{
    gravity_comp_ = gravity_comp;
}

void Joint::setFrictionComp(float friction_comp)
{
    friction_comp_ = friction_comp;
}

void Joint::setInertiaComp(float inertia_comp)
{
    inertia_comp_ = inertia_comp;
}

float Joint::getGravityComp() const
{
    return gravity_comp_;
}

float Joint::getFrictionComp() const
{
    return friction_comp_;
}

float Joint::getInertiaComp() const
{
    return inertia_comp_;
}

float Joint::getTotalTauFF() const
{
    // 总前馈 = 上层外部前馈 + 重力补偿 + 摩擦补偿 + 惯性补偿。
    return tau_ff_cmd_ + gravity_comp_ + friction_comp_ + inertia_comp_;
}

void Joint::setMode(JointMode mode)
{
    // Joint 作为模式源头时，向下同步到对应电机。
    mode_ = mode;
    if (transmission_ != nullptr)
    {
        transmission_->syncModeToMotors();
    }
}

void Joint::setModeFromTransmission(JointMode mode)
{
    // 仅用于 Transmission 反向同步，避免触发递归同步。
    mode_ = mode;
}

JointMode Joint::getMode() const
{
    return mode_;
}

void Joint::calibrate()
{
    // 标定由具体传动模型决定（简单传动/差速传动）。
    if (transmission_ != nullptr)
    {
        transmission_->calibrate(this);
    }
}

void Joint::updateFeedback(float q, float dq, float tau)
{
    q_ = q;
    dq_ = dq;
    tau_ = tau;
}

float Joint::getPosition() const
{
    return q_;
}

float Joint::getVelocity() const
{
    return dq_;
}

float Joint::getTorque() const
{
    return tau_;
}

float Joint::getTargetPosition() const
{
    return q_des_;
}

float Joint::getTargetVelocity() const
{
    return dq_des_;
}

float Joint::getTargetTorqueFF() const
{
    return tau_ff_cmd_;
}

float Joint::getImpedanceKp() const
{
    return impedance_kp_;
}

float Joint::getImpedanceKd() const
{
    return impedance_kd_;
}

float Joint::clampPosition(float q) const
{
    if (q > max_q_)
    {
        return max_q_;
    }
    if (q < min_q_)
    {
        return min_q_;
    }
    return q;
}
