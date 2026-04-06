#include "Motor.h"

#include <cmath>
Motor::Motor(MotorHardware *specific_motor)
    : specific_motor_(specific_motor)
{
}

void Motor::setPidParameters(const MotorPidParameters &param)
{
    pid_params_ = param;

    // 位置环：输出目标速度。
    a_pid.SetPIDParam(param.pos_kp, param.pos_ki, param.pos_kd, param.pos_int_max, param.pos_max);
    a_pid.I_SeparThresh = param.pos_int_sep;

    // 速度环：输出目标力矩。
    v_pid.SetPIDParam(param.spd_kp, param.spd_ki, param.spd_kd, param.spd_int_max, param.spd_max);
    v_pid.I_SeparThresh = param.spd_int_sep;
}

void Motor::setFeedforward(const MotorFeedforward &ff)
{
    feedforward_ = ff;
}

void Motor::setGravity_feedforward(float data)
{
    feedforward_.gravity = data;
}

void Motor::setBackEMF_feedforward(float data)
{
    feedforward_.back_emf = data;
}

void Motor::setCoulomb_feedforward(float data)
{
    feedforward_.coulomb = data;
}

void Motor::setViscous_feedforward(float data)
{
    feedforward_.viscous = data;
}

void Motor::setInertia_feedforward(float data)
{
    feedforward_.inertia = data;
}

void Motor::update()
{
    if (specific_motor_ == nullptr)
    {
        status_ = MotorHardwareStatus::error;
        motor_state_.status = status_;
        return;
    }

    // 读取原始反馈后统一应用逻辑零位偏置。
    const MotorState raw_state = specific_motor_->update();
    status_ = raw_state.status;

    motor_state_ = raw_state;
    motor_state_.current_angle = raw_state.current_angle - angle_zero_offset_deg_;
}

void Motor::setMode(MotorMode mode)
{
    // 安全闸门：非 ready 阶段仅允许进入 DISABLED。
    if (status_ != MotorHardwareStatus::ready && mode != MotorMode::MODE_DISABLED)
    {
        return;
    }
    forceSetMode(mode);
}

void Motor::forceSetMode(MotorMode mode)
{
    // 切模式时同步清理目标缓存，防止旧目标泄漏到新模式。
    switch (mode)
    {
    case MotorMode::MODE_DISABLED:
        target_torque_nm_ = 0.0f;
        torque_feedforward_nm_ = 0.0f;
        impedance_cmd_ = ImpedanceCommand();
        break;

    case MotorMode::MODE_POSITION:
        target_angle_deg_ = motor_state_.current_angle;
        target_torque_nm_ = 0.0f;
        torque_feedforward_nm_ = 0.0f;
        resetInternalControllerState();
        break;

    case MotorMode::MODE_TORQUE:
        target_torque_nm_ = 0.0f;
        break;

    case MotorMode::MODE_IMPEDANCE:
        impedance_cmd_.q_des = motor_state_.current_angle;
        impedance_cmd_.dq_des = 0.0f;
        break;

    default:
        return;
    }

    mode_ = mode;
}

void Motor::setTarget(float target_value)
{
    // 兼容旧接口：根据当前模式解释 target_value 语义。
    switch (mode_)
    {
    case MotorMode::MODE_POSITION:
        target_angle_deg_ = target_value;
        break;
    case MotorMode::MODE_TORQUE:
        target_torque_nm_ = target_value;
        break;
    case MotorMode::MODE_IMPEDANCE:
        impedance_cmd_.q_des = target_value;
        break;
    case MotorMode::MODE_DISABLED:
    default:
        break;
    }
}

void Motor::setTargetPosition(float target_angle_deg)
{
    target_angle_deg_ = target_angle_deg;
}

void Motor::setTargetTorque(float target_torque_nm)
{
    target_torque_nm_ = target_torque_nm;
}

void Motor::setTorqueFeedforward(float tau_ff_nm)
{
    torque_feedforward_nm_ = tau_ff_nm;
}

void Motor::setImpedanceCommand(const ImpedanceCommand &cmd)
{
    impedance_cmd_ = cmd;
}

void Motor::calibrate()
{
    if (status_ != MotorHardwareStatus::ready)
    {
        return;
    }

    // 通过累加当前角度构造零位偏置，适配单圈编码器的逻辑零点标定。
    angle_zero_offset_deg_ += motor_state_.current_angle;
    motor_state_.current_angle = 0.0f;

    if (mode_ == MotorMode::MODE_POSITION)
    {
        target_angle_deg_ = 0.0f;
        resetInternalControllerState();
    }
    else if (mode_ == MotorMode::MODE_IMPEDANCE)
    {
        impedance_cmd_.q_des = 0.0f;
    }
}

void Motor::execute()
{
    if (specific_motor_ == nullptr)
    {
        return;
    }

    // 非 ready 阶段强制下发零力矩，防止保留旧目标导致危险动作。
    if (status_ != MotorHardwareStatus::ready)
    {
        specific_motor_->setTorque(0.0f);
        return;
    }

    switch (mode_)
    {
    case MotorMode::MODE_DISABLED:
        // 硬件侧失能（DM stop / DJI Out=0）。
        specific_motor_->disable();
        break;

    case MotorMode::MODE_TORQUE:
        // 力矩模式直接下发。
        specific_motor_->setTorque(target_torque_nm_);
        break;

    case MotorMode::MODE_POSITION:
    {
        // 1) 位置环：输出目标速度；微分通道使用当前速度抑制超调。
        a_pid.Target = target_angle_deg_;
        a_pid.Current = motor_state_.current_angle;
        a_pid.Adjust_importDiff(motor_state_.current_speed);

        // 2) 速度环：输出基础力矩。
        v_pid.Target = a_pid.Out;
        v_pid.Current = motor_state_.current_speed;
        v_pid.Adjust();

        // 3) 力矩合成：基础输出 + 全量前馈。
        float total_torque = v_pid.Out + feedforward_.sum() + torque_feedforward_nm_;

        // 4) 按速度环输出上限限幅并下发纯力矩。
        total_torque = std_lib::constrain(total_torque, -v_pid.Out_Max, v_pid.Out_Max);
        specific_motor_->setTorque(total_torque);
        break;
    }

    case MotorMode::MODE_IMPEDANCE:
        // MIT 命令由底层适配器实现（原生透传或软件合成）。
        specific_motor_->setMIT(impedance_cmd_);
        break;

    default:
        specific_motor_->setTorque(0.0f);
        break;
    }
}

const MotorState &Motor::getState() const
{
    return motor_state_;
}

const MotorFeedforward &Motor::getFeedforward() const
{
    return feedforward_;
}

MotorHardwareStatus Motor::getStatus() const
{
    return status_;
}

MotorMode Motor::getMode() const
{
    return mode_;
}

const ImpedanceCommand &Motor::getImpedanceCommand() const
{
    return impedance_cmd_;
}

float Motor::getAngle() const
{
    return motor_state_.current_angle;
}

float Motor::getSpeed() const
{
    return motor_state_.current_speed;
}

float Motor::getTorque() const
{
    return motor_state_.current_torque;
}

void Motor::resetInternalControllerState()
{
    a_pid.clean_intergral();
    v_pid.clean_intergral();
}
