#ifndef __TRANSMISSION_H__
#define __TRANSMISSION_H__

#include <stdint.h>

class Motor;
class Joint;

enum class MotorMode : uint8_t;
enum class JointMode : uint8_t;

/**
 * @brief 绑定模式传动接口。
 * @note Transmission 持有关联 Motor/Joint 指针，负责双向同步。
 */
class Transmission
{
public:
    virtual ~Transmission() {}

    /**
     * @brief 从电机侧更新关节反馈。
     */
    virtual void updateFeedback() = 0;

    /**
     * @brief 将关节目标映射并应用到电机。
     */
    virtual void applyJointCommand() = 0;

    /**
     * @brief 同步关联电机控制模式。
     */
    virtual void syncModeToMotors() = 0;

    /**
     * @brief 同步关联关节控制模式。
     */
    virtual void syncModeToJoints() = 0;

    /**
     * @brief 针对指定关节执行标定。
     */
    virtual void calibrate(Joint *joint) = 0;
};

/**
 * @brief 1对1 简单传动（减速器 + 偏置 + 极性），绑定 1 个 Motor 和 1 个 Joint。
 */
class SimpleTransmission : public Transmission
{
public:
    explicit SimpleTransmission(Motor *motor,
                                Joint *joint,
                                float reduction_ratio = 1.0f,
                                float offset = 0.0f,
                                int8_t polarity = 1);

    void setParameters(float reduction_ratio, float offset, int8_t polarity);

    void updateFeedback() override;
    void applyJointCommand() override;
    void syncModeToMotors() override;
    void syncModeToJoints() override;
    void calibrate(Joint *joint) override;

private:
    Motor *motor_ = nullptr;
    Joint *joint_ = nullptr;

    float reduction_ratio_ = 1.0f;
    float inv_reduction_ratio_ = 1.0f;
    float base_offset_ = 0.0f;
    float calibration_offset_ = 0.0f;
    float polarity_sign_ = 1.0f;

    bool isBound() const;
    float totalOffset() const;
};

/**
 * @brief 2对2 差速传动，绑定 2 个 Motor 和 2 个 Joint。
 * @note 参考 Special_Actuator2:
 * q0 = 0.5 * (m0 + (-m1))
 * q1 = 0.5 * r * (m0 - (-m1))
 */
class DifferentialTransmission : public Transmission
{
public:
    explicit DifferentialTransmission(Motor *motor0,
                                     Motor *motor1,
                                     Joint *joint0,
                                     Joint *joint1,
                                     float differential_reduction_ratio = 1.0f);

    void setDifferentialReductionRatio(float differential_reduction_ratio);

    void updateFeedback() override;
    void applyJointCommand() override;
    void syncModeToMotors() override;
    void syncModeToJoints() override;
    void calibrate(Joint *joint) override;

private:
    Motor *motors_[2] = {nullptr, nullptr};
    Joint *joints_[2] = {nullptr, nullptr};

    float differential_reduction_ratio_ = 1.0f;
    float inv_differential_reduction_ratio_ = 1.0f;
    float calibration_offset_[2] = {0.0f, 0.0f};

    bool isBound() const;
    JointMode mergedJointMode() const;
    MotorMode mergedMotorMode() const;
};

#endif
