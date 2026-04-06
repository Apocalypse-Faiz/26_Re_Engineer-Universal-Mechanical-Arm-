#include "Arm.h"

#include "main.h"
#include "../Joint/Joint.h"

Arm::Arm()
    : waypoint_count_(0),
      running_(false),
    trajectory_start_tick_ms_(0),
      trajectory_total_ms_(0)
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        joints_[i] = nullptr;
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
    trajectory_start_tick_ms_ = 0;
    trajectory_total_ms_ = 0;

    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        traj_[i].planner.reset();
        traj_[i].points.clear();
    }
}

bool Arm::addWaypoint(const float q_target[kNumJoints], std::uint32_t duration_ms)
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

    trajectory_start_tick_ms_ = HAL_GetTick();
    trajectory_total_ms_ = 0;
    for (uint8_t i = 0; i < waypoint_count_; ++i)
    {
        trajectory_total_ms_ += waypoints_[i].duration_ms;
    }

    if (!buildJointTrajectories(q_start))
    {
        return false;
    }

    if (trajectory_total_ms_ == 0)
    {
        setJointTarget(waypoints_[waypoint_count_ - 1].q);
        running_ = false;
        return true;
    }

    running_ = true;
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

void Arm::update()
{
    if (!running_)
    {
        return;
    }

    const std::uint32_t now_tick_ms = HAL_GetTick();
    const std::uint32_t elapsed_ms = now_tick_ms - trajectory_start_tick_ms_;

    const std::uint32_t sample_tick_ms = (elapsed_ms > trajectory_total_ms_)
                                             ? (trajectory_start_tick_ms_ + trajectory_total_ms_)
                                             : now_tick_ms;

    // 每个关节独立执行 CQC 插值，并下放目标到 Joint。
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        if (joints_[i] == nullptr)
        {
            continue;
        }

        float q_des = joints_[i]->getPosition();
        if (traj_[i].planner.get(sample_tick_ms, &q_des))
        {
            joints_[i]->setTargetAngle(q_des);
        }
    }

    if (elapsed_ms >= trajectory_total_ms_)
    {
        running_ = false;
    }
}

void Arm::captureCurrentJointPosition(float q_out[kNumJoints]) const
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        q_out[i] = (joints_[i] != nullptr) ? joints_[i]->getPosition() : 0.0f;
    }
}

bool Arm::buildJointTrajectories(const float q_start[kNumJoints])
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        JointTrajectoryChannel &channel = traj_[i];
        channel.planner.reset();
        channel.points.clear();

        for (uint8_t wp_idx = 0; wp_idx < waypoint_count_; ++wp_idx)
        {
            channel.points.add(waypoints_[wp_idx].duration_ms, waypoints_[wp_idx].q[i]);
        }

        channel.planner.receive(channel.points, q_start[i]);
        channel.planner.compute(trajectory_start_tick_ms_);
    }

    return true;
}
