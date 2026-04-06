#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "SRML.h"
#include <cstdint>

/**
 * @brief 电机控制模式（纯电机空间）。
 */
enum class MotorMode : std::uint8_t
{
    MODE_DISABLED = 0, /**< 失能模式：下发 0 力矩或断电/stop 指令。 */
    MODE_POSITION,     /**< 位置模式：内部执行位置环 + 速度环串级 PID。 */
    MODE_TORQUE,       /**< 力矩模式：直接下发目标力矩。 */
    MODE_IMPEDANCE     /**< 阻抗模式：MIT(q,dq,kp,kd,tau_ff) 透传。 */
};

/**
 * @brief 底层硬件状态。
 */
enum class MotorHardwareStatus : std::uint8_t
{
    ready = 0,
    error
};

/**
 * @brief MIT 阻抗命令（电机空间）。
 * @note 单位统一为：q_des[deg], dq_des[rad/s], tau_ff[N·m]。
 */
struct ImpedanceCommand
{
    float q_des = 0.0f;   /**< 期望电机角度（deg） */
    float dq_des = 0.0f;  /**< 期望电机角速度（rad/s） */
    float kp = 0.0f;      /**< MIT 位置刚度 */
    float kd = 0.0f;      /**< MIT 速度阻尼 */
    float tau_ff = 0.0f;  /**< 力矩前馈（N·m） */
};

/**
 * @brief 电机反馈状态（电机空间）。
 */
struct MotorState
{
    MotorHardwareStatus status = MotorHardwareStatus::error; /**< 底层状态 */
    float current_angle = 0.0f;   /**< 当前电机角度（deg） */
    float current_speed = 0.0f;   /**< 当前电机角速度（rad/s） */
    float current_torque = 0.0f;  /**< 当前电机力矩（N·m） */
};

/**
 * @brief 串级 PID 参数。
 */
struct MotorPidParameters
{
    float pos_kp = 0.0f;          /**< 位置环 Kp */
    float pos_ki = 0.0f;          /**< 位置环 Ki */
    float pos_kd = 0.0f;          /**< 位置环 Kd */
    float pos_min = 0.0f;         /**< 位置环输出下限（预留） */
    float pos_max = 0.0f;         /**< 位置环输出上限（目标速度限幅） */
    float pos_int_sep = 0.0f;     /**< 位置环积分分离阈值 */
    float pos_int_max = 0.0f;     /**< 位置环积分上限 */
    float spd_kp = 0.0f;          /**< 速度环 Kp */
    float spd_ki = 0.0f;          /**< 速度环 Ki */
    float spd_kd = 0.0f;          /**< 速度环 Kd */
    float spd_min = 0.0f;         /**< 速度环输出下限（预留） */
    float spd_max = 0.0f;         /**< 速度环输出上限（目标力矩限幅） */
    float spd_int_sep = 0.0f;     /**< 速度环积分分离阈值 */
    float spd_int_max = 0.0f;     /**< 速度环积分上限 */
};

/**
 * @brief 前馈补偿集合（电机侧）。
 */
struct MotorFeedforward
{
    float gravity = 0.0f; /**< 重力前馈 */
    float back_emf = 0.0f; /**< 反电动势前馈 */
    float coulomb = 0.0f; /**< 库仑摩擦前馈 */
    float viscous = 0.0f; /**< 黏性摩擦前馈 */
    float inertia = 0.0f; /**< 转动惯量前馈 */

    /**
     * @brief 获取各项前馈求和。
     */
    float sum() const
    {
        return gravity + back_emf + coulomb + viscous + inertia;
    }
};

/**
 * @brief 底层硬件抽象接口（电机侧）。
 * @note update() 返回状态结构体，避免底层驱动反向修改上层对象成员。
 */
class MotorHardware
{
public:
    virtual ~MotorHardware() {}

    /**
     * @brief 更新底层反馈并返回电机侧状态。
     */
    virtual MotorState update() = 0;

    /**
     * @brief 下发电机侧纯力矩。
     * @param motor_tau_nm 电机侧力矩（N·m）
     */
    virtual void setTorque(float motor_tau_nm) = 0;

    /**
     * @brief 下发电机侧 MIT 命令。
     * @param motor_cmd 电机侧 MIT 命令（q[deg], dq[rad/s], kp, kd, tau_ff[N·m]）
     */
    virtual void setMIT(const ImpedanceCommand &motor_cmd) = 0;

    /**
     * @brief 失能硬件输出（例如 DM stopMotor 或输出清零）。
     */
    virtual void disable() = 0;

    /**
     * @brief 硬件是否原生支持 MIT。
     */
    virtual bool supportsMIT() const = 0;
};

/**
 * @brief 统一电机控制器（只处理电机空间）。
 *
 * 控制循环典型调用顺序：
 * 1) update();
 * 2) setTarget...()/setMode(...);
 * 3) execute();
 */
class Motor
{
public:
    explicit Motor(MotorHardware *specific_motor);

    /**
     * @brief 设置串级 PID 参数。
     */
    void setPidParameters(const MotorPidParameters &param);

    /**
     * @brief 设置电机侧前馈。
     */
    void setFeedforward(const MotorFeedforward &ff);

    /**
     * @brief 更新电机反馈（angle, speed, torque）。
     */
    void update();

    /**
     * @brief 切换控制模式。
     */
    void setMode(MotorMode mode);

    /**
     * @brief 强制切换控制模式（用于掉线保护等场景）。
     */
    void forceSetMode(MotorMode mode);

    /**
     * @brief 兼容旧接口：
     * - MODE_POSITION: target_value -> 目标角度（deg）
     * - MODE_TORQUE: target_value -> 目标力矩（N·m）
     * - MODE_IMPEDANCE: target_value -> q_des（deg）
     */
    void setTarget(float target_value);

    /**
     * @brief 设置电机空间位置目标（deg）。
     */
    void setTargetPosition(float target_angle_deg);

    /**
     * @brief 设置电机空间力矩目标（N·m）。
     */
    void setTargetTorque(float target_torque_nm);

    /**
     * @brief 设置位置模式额外力矩前馈（N·m）。
     */
    void setTorqueFeedforward(float tau_ff_nm);

    /**
     * @brief 设置 MIT 阻抗命令（电机空间）。
     */
    void setImpedanceCommand(const ImpedanceCommand &cmd);

    /**
     * @brief 标定当前电机角度为零点。
     * @note 仅在底层状态为 ready 时生效。
     */
    void calibrate();

    /**
     * @brief 执行控制逻辑（通常由 1kHz RTOS 任务调用）。
     */
    void execute();

    /**
     * @brief 获取当前电机反馈状态。
     */
    const MotorState &getState() const;

    /**
     * @brief 获取当前前馈参数。
     */
    const MotorFeedforward &getFeedforward() const;

    /**
     * @brief 获取底层硬件状态。
     */
    MotorHardwareStatus getStatus() const;

    /**
     * @brief 获取当前控制模式。
     */
    MotorMode getMode() const;

    /**
     * @brief 获取阻抗命令缓存。
     */
    const ImpedanceCommand &getImpedanceCommand() const;

    /* 兼容旧接口的封装函数 */
    float getAngle() const;
    float getSpeed() const;
    float getTorque() const;

    /* 保留原前馈接口命名 */
    void setGravity_feedforward(float data);
    void setBackEMF_feedforward(float data);
    void setCoulomb_feedforward(float data);
    void setViscous_feedforward(float data);
    void setInertia_feedforward(float data);

    myPID a_pid; /**< 角度环 PID（输出目标速度） */
    myPID v_pid; /**< 速度环 PID（输出目标力矩） */

private:
    MotorHardware *specific_motor_ = nullptr;
    MotorHardwareStatus status_ = MotorHardwareStatus::error;
    MotorMode mode_ = MotorMode::MODE_DISABLED;

    MotorState motor_state_;
    ImpedanceCommand impedance_cmd_;
    MotorFeedforward feedforward_;
    MotorPidParameters pid_params_;
    // 标定零位偏置：逻辑角度 = 原始角度 - 偏置。
    float angle_zero_offset_deg_ = 0.0f;

    float target_angle_deg_ = 0.0f;
    float target_torque_nm_ = 0.0f;
    float torque_feedforward_nm_ = 0.0f;

    /**
     * @brief 清空内部 PID 积分状态，避免切模式时突变。
     */
    void resetInternalControllerState();
};

/* 兼容历史命名，便于逐步迁移 */
using MotorStatus = MotorHardwareStatus;
using Motor_ = MotorHardware;
using HardwareInterface = MotorHardware;
using HardwareStatus = MotorHardwareStatus;
using HardwareFeedback = MotorState;
using JointCommand = ImpedanceCommand;

#endif
