#ifndef __JOINT_H__
#define __JOINT_H__

#include <stdint.h>

class Transmission;

enum class JointMode : uint8_t
{
    MODE_DISABLED = 0,
    MODE_POSITION,
    MODE_TORQUE,
    MODE_IMPEDANCE
};

/**
 * @brief 关节空间对象。
 * @note 单位：q[deg], dq[rad/s], tau[N·m]。
 */
class Joint
{
public:
    explicit Joint(float min_q = -180.0f, float max_q = 180.0f);

    void bindTransmission(Transmission *transmission);

    void setSoftLimit(float min_q, float max_q);
    float getMinLimit() const;
    float getMaxLimit() const;

    void setTargetPosition(float q_d);
    void setTargetVelocity(float dq_d);
    void setTargetTorqueFF(float tau_ff);
    void setImpedanceTarget(float q_d, float dq_d, float kp, float kd, float tau_ff);

    void setGravityComp(float gravity_comp);
    void setFrictionComp(float friction_comp);
    void setInertiaComp(float inertia_comp);

    float getGravityComp() const;
    float getFrictionComp() const;
    float getInertiaComp() const;
    float getTotalTauFF() const;

    void setMode(JointMode mode);
    void setModeFromTransmission(JointMode mode);
    JointMode getMode() const;

    void calibrate();

    void updateFeedback(float q, float dq, float tau);

    float getPosition() const;
    float getVelocity() const;
    float getTorque() const;

    float getTargetPosition() const;
    float getTargetVelocity() const;
    float getTargetTorqueFF() const;
    float getImpedanceKp() const;
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

    float clampPosition(float q) const;
};

#endif
