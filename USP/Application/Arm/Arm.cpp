#include "Arm.h"

#include "../Joint/Joint.h"

namespace
{
inline float clamp01(float x)
{
    if (x < 0.0f)
    {
        return 0.0f;
    }
    if (x > 1.0f)
    {
        return 1.0f;
    }
    return x;
}
}

Arm::Arm()
    : waypoint_count_(0),
      running_(false),
      segment_index_(0),
      local_t_ms_(0),
      segment_duration_ms_(0)
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        joints_[i] = nullptr;
        segment_q_start_[i] = 0.0f;
        segment_q_end_[i] = 0.0f;
    }

    M_ = Math3D::eye4();
    Slist_ = Math3D::zeros6();
}

void Arm::bindJoint(uint8_t index, Joint *joint)
{
    if (index >= kNumJoints)
    {
        return;
    }
    joints_[index] = joint;
}

bool Arm::isReady() const
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        if (joints_[i] == nullptr)
        {
            return false;
        }
    }
    return true;
}

void Arm::setModel(const Math3D::Mat4 &M, const Math3D::Mat6 &Slist)
{
    M_ = M;
    Slist_ = Slist;
}

Math3D::Mat4 Arm::fkine(const float q[kNumJoints]) const
{
    return Math3D::fkine(M_, Slist_, q);
}

Math3D::Mat4 Arm::fkineCurrent() const
{
    float q[kNumJoints] = {};
    captureCurrentJointPosition(q);
    return Math3D::fkine(M_, Slist_, q);
}

Math3D::Mat6 Arm::jacobian(const float q[kNumJoints]) const
{
    return Math3D::cal_Jacobian(Slist_, q);
}

bool Arm::ikine(const Math3D::Mat4 &T_target,
                const float q0[kNumJoints],
                float eomg,
                float ev,
                float q_out[kNumJoints],
                float lambda,
                float max_step,
                int max_iterations) const
{
    return Math3D::ikine(Slist_, M_, T_target, q0, eomg, ev, q_out, lambda, max_step, max_iterations);
}

bool Arm::setCartesianTarget(const Math3D::Mat4 &T_target,
                             float eomg,
                             float ev,
                             float lambda,
                             float max_step,
                             int max_iterations)
{
    if (!isReady())
    {
        return false;
    }

    // 以当前姿态为 IK 初值，提高收敛稳定性。
    float q0[kNumJoints] = {};
    captureCurrentJointPosition(q0);

    float q_sol[kNumJoints] = {};
    const bool ok = Math3D::ikine(Slist_, M_, T_target, q0, eomg, ev, q_sol, lambda, max_step, max_iterations);
    if (!ok)
    {
        return false;
    }

    setJointTarget(q_sol);
    return true;
}

void Arm::setJointTarget(const float q_target[kNumJoints])
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        if (joints_[i] != nullptr)
        {
            joints_[i]->setTargetAngle(q_target[i]);
        }
    }
}

void Arm::clearWaypoints()
{
    waypoint_count_ = 0;
    running_ = false;
    segment_index_ = 0;
    local_t_ms_ = 0;
    segment_duration_ms_ = 0;
}

bool Arm::addWaypoint(const float q_target[kNumJoints], uint32_t duration_ms)
{
    if (waypoint_count_ >= kMaxWaypoints)
    {
        return false;
    }

    ArmWaypoint &wp = waypoints_[waypoint_count_];
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        wp.q[i] = q_target[i];
    }
    wp.duration_ms = duration_ms;

    ++waypoint_count_;
    return true;
}

bool Arm::startWaypointsFromCurrent()
{
    if (!isReady() || waypoint_count_ == 0)
    {
        return false;
    }

    float q_start[kNumJoints] = {};
    captureCurrentJointPosition(q_start);
    prepareSegment(0, q_start);

    running_ = true;
    segment_index_ = 0;
    local_t_ms_ = 0;
    return true;
}

void Arm::stopWaypoints()
{
    running_ = false;
}

bool Arm::waypointsRunning() const
{
    return running_;
}

void Arm::update(uint32_t dt_ms)
{
    if (!running_)
    {
        return;
    }

    // 时长为 0 时直接跳段，兼容“立即到位”的路径点。
    if (segment_duration_ms_ == 0)
    {
        setJointTarget(segment_q_end_);

        const uint8_t next_index = static_cast<uint8_t>(segment_index_ + 1);
        if (next_index >= waypoint_count_)
        {
            running_ = false;
            return;
        }

        prepareSegment(next_index, segment_q_end_);
        segment_index_ = next_index;
        local_t_ms_ = 0;
        return;
    }

    if (local_t_ms_ < segment_duration_ms_)
    {
        local_t_ms_ += dt_ms;
        if (local_t_ms_ > segment_duration_ms_)
        {
            local_t_ms_ = segment_duration_ms_;
        }
    }

    const float u = clamp01(static_cast<float>(local_t_ms_) / static_cast<float>(segment_duration_ms_));
    // 使用 quintic 曲线保证段首尾速度/加速度连续。
    const float s = quinticBlend(u);

    float q_des[kNumJoints] = {};
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        q_des[i] = segment_q_start_[i] + (segment_q_end_[i] - segment_q_start_[i]) * s;
    }
    setJointTarget(q_des);

    if (local_t_ms_ >= segment_duration_ms_)
    {
        const uint8_t next_index = static_cast<uint8_t>(segment_index_ + 1);
        if (next_index >= waypoint_count_)
        {
            running_ = false;
            return;
        }

        prepareSegment(next_index, segment_q_end_);
        segment_index_ = next_index;
        local_t_ms_ = 0;
    }
}

void Arm::captureCurrentJointPosition(float q_out[kNumJoints]) const
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        q_out[i] = (joints_[i] != nullptr) ? joints_[i]->getPosition() : 0.0f;
    }
}

void Arm::prepareSegment(uint8_t index, const float start_q[kNumJoints])
{
    if (index >= waypoint_count_)
    {
        return;
    }

    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        segment_q_start_[i] = start_q[i];
        segment_q_end_[i] = waypoints_[index].q[i];
    }

    segment_duration_ms_ = waypoints_[index].duration_ms;
}

float Arm::quinticBlend(float u)
{
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float u4 = u3 * u;
    const float u5 = u4 * u;
    return 10.0f * u3 - 15.0f * u4 + 6.0f * u5;
}
