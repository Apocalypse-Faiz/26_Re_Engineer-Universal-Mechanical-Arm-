#ifndef __JOINT_H__
#define __JOINT_H__

#include <stdint.h>

class Transmission;

enum class JointMode : uint8_t
{
    MODE_DISABLED = 0, /**< 失能模式 */
    MODE_POSITION,     /**< 位置模式 */
    MODE_TORQUE,       /**< 力矩模式 */
    MODE_IMPEDANCE     /**< 阻抗模式 */
};

/**
 * @brief 关节空间对象。
 * @note 单位：q[deg], dq[rad/s], tau[N·m]。
 */
class Joint
{
public:
    /**
     * @brief 构造关节对象并设置软限位。
     * @param min_q 最小关节角（deg）
     * @param max_q 最大关节角（deg）
     */
    explicit Joint(float min_q = -180.0f, float max_q = 180.0f);

    /**
     * @brief 绑定传动对象（非拥有关系）。
     */
    void bindTransmission(Transmission *transmission);

    /**
     * @brief 设置关节软限位（deg）。
     */
    void setSoftLimit(float min_q, float max_q);

    /**
     * @brief 获取软限位最小值（deg）。
     */
    float getMinLimit() const;

    /**
     * @brief 获取软限位最大值（deg）。
     */
    float getMaxLimit() const;

    /**
     * @brief 设置位置目标（deg）。
     */
    void setTargetPosition(float q_d);

    /**
     * @brief setTargetPosition 的语义别名。
     */
    void setTargetAngle(float q_d);

    /**
     * @brief 设置速度目标（rad/s）。
     */
    void setTargetVelocity(float dq_d);

    /**
     * @brief 设置外部力矩前馈（N·m）。
     */
    void setTargetTorqueFF(float tau_ff);

    /**
     * @brief 一次性设置阻抗目标。
     */
    void setImpedanceTarget(float q_d, float dq_d, float kp, float kd, float tau_ff);

    /**
     * @brief 设置重力补偿项（N·m）。
     */
    void setGravityComp(float gravity_comp);

    /**
     * @brief 设置摩擦补偿项（N·m）。
     */
    void setFrictionComp(float friction_comp);

    /**
     * @brief 设置惯性补偿项（N·m）。
     */
    void setInertiaComp(float inertia_comp);

    float getGravityComp() const;
    float getFrictionComp() const;
    float getInertiaComp() const;

    /**
     * @brief 获取总力矩前馈（目标前馈+三项补偿）。
     */
    float getTotalTauFF() const;

    /**
     * @brief 设置关节模式，并同步到关联电机。
     */
    void setMode(JointMode mode);

    /**
     * @brief 仅由 Transmission 反向更新模式，不触发再次同步。
     */
    void setModeFromTransmission(JointMode mode);

    /**
     * @brief 获取当前关节模式。
     */
    JointMode getMode() const;

    /**
     * @brief 执行关节零位标定（委托 Transmission 处理）。
     */
    void calibrate();

    /**
     * @brief 由 Transmission 写入反馈状态。
     */
    void updateFeedback(float q, float dq, float tau);

    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;

    float getTargetPosition() const;
    float getTargetVelocity() const;
    float getTargetTorqueFF() const;

    /**
     * @brief 获取阻抗参数 Kp。
     */
    float getImpedanceKp() const;

    /**
     * @brief 获取阻抗参数 Kd。
     */
    float getImpedanceKd() const;

private:
    Transmission *transmission_ = nullptr;

    JointMode mode_ = JointMode::MODE_DISABLED;

    float q_ = 0.0f;
    float dq_ = 0.0f;
    float tau_ = 0.0f;

    float min_q_ = -180.0f;
    float max_q_ = 180.0f;

    float q_des_ = 0.0f;
    float dq_des_ = 0.0f;
    float tau_ff_cmd_ = 0.0f;

    float impedance_kp_ = 0.0f;
    float impedance_kd_ = 0.0f;

    float gravity_comp_ = 0.0f;
    float friction_comp_ = 0.0f;
    float inertia_comp_ = 0.0f;

    /**
     * @brief 位置软限位裁剪。
     */
    float clampPosition(float q) const;
};

#endif
