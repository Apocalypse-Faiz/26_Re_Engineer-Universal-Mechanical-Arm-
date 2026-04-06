#include "Transmission.h"

#include <cmath>
#include "../Motor/Motor.h"
#include "../Joint/Joint.h"

namespace
{
constexpr float kRatioEpsilon = 1e-6f;

inline float safeRatio(float ratio)
{
    return (std::fabs(ratio) < kRatioEpsilon) ? 1.0f : ratio;
}

inline float polarityToSign(int8_t polarity)
{
    return (polarity >= 0) ? 1.0f : -1.0f;
}

inline MotorMode jointModeToMotorMode(JointMode mode)
{
    // Joint 模式与 Motor 模式一一对应。
    switch (mode)
    {
    case JointMode::MODE_POSITION:
        return MotorMode::MODE_POSITION;
    case JointMode::MODE_TORQUE:
        return MotorMode::MODE_TORQUE;
    case JointMode::MODE_IMPEDANCE:
        return MotorMode::MODE_IMPEDANCE;
    case JointMode::MODE_DISABLED:
    default:
        return MotorMode::MODE_DISABLED;
    }
}

inline JointMode motorModeToJointMode(MotorMode mode)
{
    // Motor 模式与 Joint 模式一一对应。
    switch (mode)
    {
    case MotorMode::MODE_POSITION:
        return JointMode::MODE_POSITION;
    case MotorMode::MODE_TORQUE:
        return JointMode::MODE_TORQUE;
    case MotorMode::MODE_IMPEDANCE:
        return JointMode::MODE_IMPEDANCE;
    case MotorMode::MODE_DISABLED:
    default:
        return JointMode::MODE_DISABLED;
    }
}

inline float clampRatioSafe(float ratio)
{
    return safeRatio(ratio);
}
}

SimpleTransmission::SimpleTransmission(Motor *motor,
                                       Joint *joint,
                                       float reduction_ratio,
                                       float offset,
                                       int8_t polarity)
    : motor_(motor), joint_(joint)
{
    if (joint_)
    {
        joint_->bindTransmission(this);
    }
    setParameters(reduction_ratio, offset, polarity);
}

void SimpleTransmission::setParameters(float reduction_ratio,
                                       float offset,
                                       int8_t polarity)
{
    reduction_ratio_ = safeRatio(reduction_ratio);
    inv_reduction_ratio_ = 1.0f / reduction_ratio_;
    base_offset_ = offset;
    polarity_sign_ = polarityToSign(polarity);
}

bool SimpleTransmission::isBound() const
{
    return (motor_ != nullptr && joint_ != nullptr);
}

float SimpleTransmission::totalOffset() const
{
    return base_offset_ + calibration_offset_;
}

void SimpleTransmission::updateFeedback()
{
    if (!isBound())
    {
        return;
    }

    syncModeToJoints();

    const MotorState &m = motor_->getState();
    // 反馈映射：q = sgn * (m/ratio) - offset, dq = sgn * (dm/ratio), tau = sgn * (tm*ratio)。
    const float q = polarity_sign_ * (m.current_angle * inv_reduction_ratio_) - totalOffset();
    const float dq = polarity_sign_ * (m.current_speed * inv_reduction_ratio_);
    const float tau = polarity_sign_ * (m.current_torque * reduction_ratio_);
    joint_->updateFeedback(q, dq, tau);
}

void SimpleTransmission::applyJointCommand()
{
    if (!isBound())
    {
        return;
    }

    syncModeToMotors();

    const JointMode mode = joint_->getMode();
    const float q_cmd = joint_->getTargetPosition();
    const float dq_cmd = joint_->getTargetVelocity();
    const float tau_ff_joint = joint_->getTotalTauFF();

    switch (mode)
    {
    case JointMode::MODE_POSITION:
    {
        // 目标位置映射到电机空间，关节前馈力矩映射到电机侧。
        const float motor_q_des = polarity_sign_ * (q_cmd + totalOffset()) * reduction_ratio_;
        const float motor_tau_ff = polarity_sign_ * (tau_ff_joint * inv_reduction_ratio_);
        motor_->setTargetPosition(motor_q_des);
        motor_->setTorqueFeedforward(motor_tau_ff);
        break;
    }

    case JointMode::MODE_TORQUE:
    {
        const float motor_tau_ff = polarity_sign_ * (tau_ff_joint * inv_reduction_ratio_);
        motor_->setTargetTorque(motor_tau_ff);
        break;
    }

    case JointMode::MODE_IMPEDANCE:
    {
        // Kp/Kd 从关节空间映射到电机空间，缩放系数约为 1/ratio^2。
        ImpedanceCommand cmd;
        cmd.q_des = polarity_sign_ * (q_cmd + totalOffset()) * reduction_ratio_;
        cmd.dq_des = polarity_sign_ * (dq_cmd * reduction_ratio_);
        cmd.kp = joint_->getImpedanceKp() * inv_reduction_ratio_ * inv_reduction_ratio_;
        cmd.kd = joint_->getImpedanceKd() * inv_reduction_ratio_ * inv_reduction_ratio_;
        cmd.tau_ff = polarity_sign_ * (tau_ff_joint * inv_reduction_ratio_);
        motor_->setImpedanceCommand(cmd);
        break;
    }

    case JointMode::MODE_DISABLED:
    default:
        break;
    }
}

void SimpleTransmission::syncModeToMotors()
{
    if (!isBound())
    {
        return;
    }
    motor_->setMode(jointModeToMotorMode(joint_->getMode()));
}

void SimpleTransmission::syncModeToJoints()
{
    if (!isBound())
    {
        return;
    }

    joint_->setModeFromTransmission(motorModeToJointMode(motor_->getMode()));
}

void SimpleTransmission::calibrate(Joint *joint)
{
    if (!isBound() || joint != joint_)
    {
        return;
    }

    calibration_offset_ += joint_->getPosition();
    updateFeedback();
}

DifferentialTransmission::DifferentialTransmission(Motor *motor0,
                                                   Motor *motor1,
                                                   Joint *joint0,
                                                   Joint *joint1,
                                                   float differential_reduction_ratio)
{
    motors_[0] = motor0;
    motors_[1] = motor1;
    joints_[0] = joint0;
    joints_[1] = joint1;

    if (joints_[0])
    {
        joints_[0]->bindTransmission(this);
    }
    if (joints_[1])
    {
        joints_[1]->bindTransmission(this);
    }

    setDifferentialReductionRatio(differential_reduction_ratio);
}

void DifferentialTransmission::setDifferentialReductionRatio(float differential_reduction_ratio)
{
    differential_reduction_ratio_ = clampRatioSafe(differential_reduction_ratio);
    inv_differential_reduction_ratio_ = 1.0f / differential_reduction_ratio_;
}

bool DifferentialTransmission::isBound() const
{
    return (motors_[0] != nullptr && motors_[1] != nullptr &&
            joints_[0] != nullptr && joints_[1] != nullptr);
}

JointMode DifferentialTransmission::mergedJointMode() const
{
    if (!isBound())
    {
        return JointMode::MODE_DISABLED;
    }

    const JointMode m0 = joints_[0]->getMode();
    const JointMode m1 = joints_[1]->getMode();

    if (m0 == JointMode::MODE_DISABLED || m1 == JointMode::MODE_DISABLED)
    {
        return JointMode::MODE_DISABLED;
    }

    if (m0 == JointMode::MODE_TORQUE || m1 == JointMode::MODE_TORQUE)
    {
        return JointMode::MODE_TORQUE;
    }

    if (m0 == JointMode::MODE_IMPEDANCE || m1 == JointMode::MODE_IMPEDANCE)
    {
        return JointMode::MODE_IMPEDANCE;
    }

    return JointMode::MODE_POSITION;
}

MotorMode DifferentialTransmission::mergedMotorMode() const
{
    if (!isBound())
    {
        return MotorMode::MODE_DISABLED;
    }

    const MotorMode m0 = motors_[0]->getMode();
    const MotorMode m1 = motors_[1]->getMode();

    if (m0 == MotorMode::MODE_DISABLED || m1 == MotorMode::MODE_DISABLED)
    {
        return MotorMode::MODE_DISABLED;
    }

    if (m0 == MotorMode::MODE_TORQUE || m1 == MotorMode::MODE_TORQUE)
    {
        return MotorMode::MODE_TORQUE;
    }

    if (m0 == MotorMode::MODE_IMPEDANCE || m1 == MotorMode::MODE_IMPEDANCE)
    {
        return MotorMode::MODE_IMPEDANCE;
    }

    return MotorMode::MODE_POSITION;
}

void DifferentialTransmission::updateFeedback()
{
    if (!isBound())
    {
        return;
    }

    syncModeToJoints();

    const MotorState &m0 = motors_[0]->getState();
    const MotorState &m1 = motors_[1]->getState();

    // 差速正解：由两个电机角度/速度解算两关节状态。
    const float m1_pos_neg = -m1.current_angle;
    const float q0 = 0.5f * (m0.current_angle + m1_pos_neg) - calibration_offset_[0];
    const float q1 = 0.5f * differential_reduction_ratio_ * (m0.current_angle - m1_pos_neg) - calibration_offset_[1];

    const float m1_vel_neg = -m1.current_speed;
    const float dq0 = 0.5f * (m0.current_speed + m1_vel_neg);
    const float dq1 = 0.5f * differential_reduction_ratio_ * (m0.current_speed - m1_vel_neg);

    // 力矩映射采用差速耦合关系。
    const float tq0 = m0.current_torque - m1.current_torque;
    const float tq1 = (m0.current_torque + m1.current_torque) * inv_differential_reduction_ratio_;

    joints_[0]->updateFeedback(q0, dq0, tq0);
    joints_[1]->updateFeedback(q1, dq1, tq1);
}

void DifferentialTransmission::applyJointCommand()
{
    if (!isBound())
    {
        return;
    }

    syncModeToMotors();

    const JointMode merged_mode = mergedJointMode();

    float q_cmd[2] = {
        joints_[0]->getTargetPosition() + calibration_offset_[0],
        joints_[1]->getTargetPosition() + calibration_offset_[1]};
    float dq_cmd[2] = {
        joints_[0]->getTargetVelocity(),
        joints_[1]->getTargetVelocity()};
    float tau_ff[2] = {
        joints_[0]->getTotalTauFF(),
        joints_[1]->getTotalTauFF()};

    if (joints_[0]->getMode() == JointMode::MODE_IMPEDANCE)
    {
        // 若关节处于阻抗模式，在传动层先合成等效关节力矩。
        tau_ff[0] += joints_[0]->getImpedanceKp() * (joints_[0]->getTargetPosition() - joints_[0]->getPosition()) +
                     joints_[0]->getImpedanceKd() * (joints_[0]->getTargetVelocity() - joints_[0]->getVelocity());
    }
    if (joints_[1]->getMode() == JointMode::MODE_IMPEDANCE)
    {
        tau_ff[1] += joints_[1]->getImpedanceKp() * (joints_[1]->getTargetPosition() - joints_[1]->getPosition()) +
                     joints_[1]->getImpedanceKd() * (joints_[1]->getTargetVelocity() - joints_[1]->getVelocity());
    }

    // 差速逆解：将关节角/速目标映射到双电机目标。
    const float q1_scaled = q_cmd[1] * inv_differential_reduction_ratio_;
    const float dq1_scaled = dq_cmd[1] * inv_differential_reduction_ratio_;

    const float m_q_des_0 = q_cmd[0] + q1_scaled;
    const float m_q_des_1 = -q_cmd[0] + q1_scaled;
    const float m_dq_des_0 = dq_cmd[0] + dq1_scaled;
    const float m_dq_des_1 = -dq_cmd[0] + dq1_scaled;

    // 力矩雅可比转置映射：tau_m = J^T * tau_q。
    const float m_tau_ff_0 = 0.5f * (tau_ff[0] + differential_reduction_ratio_ * tau_ff[1]);
    const float m_tau_ff_1 = 0.5f * (-tau_ff[0] + differential_reduction_ratio_ * tau_ff[1]);

    if (merged_mode == JointMode::MODE_POSITION)
    {
        motors_[0]->setTargetPosition(m_q_des_0);
        motors_[1]->setTargetPosition(m_q_des_1);
        motors_[0]->setTorqueFeedforward(m_tau_ff_0);
        motors_[1]->setTorqueFeedforward(m_tau_ff_1);
    }
    else if (merged_mode == JointMode::MODE_TORQUE)
    {
        motors_[0]->setTargetTorque(m_tau_ff_0);
        motors_[1]->setTargetTorque(m_tau_ff_1);
    }
    else if (merged_mode == JointMode::MODE_IMPEDANCE)
    {
        ImpedanceCommand cmd0;
        cmd0.q_des = m_q_des_0;
        cmd0.dq_des = m_dq_des_0;
        cmd0.kp = 0.0f;
        cmd0.kd = 0.0f;
        cmd0.tau_ff = m_tau_ff_0;

        ImpedanceCommand cmd1;
        cmd1.q_des = m_q_des_1;
        cmd1.dq_des = m_dq_des_1;
        cmd1.kp = 0.0f;
        cmd1.kd = 0.0f;
        cmd1.tau_ff = m_tau_ff_1;

        motors_[0]->setImpedanceCommand(cmd0);
        motors_[1]->setImpedanceCommand(cmd1);
    }
}

void DifferentialTransmission::syncModeToMotors()
{
    if (!isBound())
    {
        return;
    }

    const JointMode merged_mode = mergedJointMode();
    const MotorMode mm = jointModeToMotorMode(merged_mode);
    motors_[0]->setMode(mm);
    motors_[1]->setMode(mm);
}

void DifferentialTransmission::syncModeToJoints()
{
    if (!isBound())
    {
        return;
    }

    const JointMode jm = motorModeToJointMode(mergedMotorMode());
    joints_[0]->setModeFromTransmission(jm);
    joints_[1]->setModeFromTransmission(jm);
}

void DifferentialTransmission::calibrate(Joint *joint)
{
    if (!isBound() || joint == nullptr)
    {
        return;
    }

    if (joint == joints_[0])
    {
        // 标定后当前位置作为该关节逻辑零点。
        calibration_offset_[0] += joints_[0]->getPosition();
    }
    else if (joint == joints_[1])
    {
        calibration_offset_[1] += joints_[1]->getPosition();
    }

    updateFeedback();
}
