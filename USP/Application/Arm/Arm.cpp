#include "Arm.h"

#include <cmath>

#include "main.h"
#include "../Joint/Joint.h"

Arm::Arm()
        : model_configured_(false),
            waypoint_count_(0),
            running_(false),
            last_exec_status_(ExecStatus::IDLE),
            planner_sample_fail_count_(0),
            consecutive_sample_fail_count_(0),
            trajectory_start_tick_ms_(0),
            trajectory_total_ms_(0)
{
    // 关节槽位默认全空，必须由上层按索引绑定后才可进入 ready 状态。
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        joints_[i] = nullptr;
    }

    // 默认模型为单位位姿 + 零旋量，避免未初始化内存参与运动学计算。
    M0_ = Math3D::eye4();
    Slist0_ = Math3D::zeros6();
    // 当前模型初值与 home 模型一致，后续可扩展为在线模型切换。
    M_ = M0_;
    Slist_ = Slist0_;
}

Arm::Arm(const float alpha[kNumJoints],
         const float a[kNumJoints],
         const float d[kNumJoints],
         const float theta_offset[kNumJoints],
         Math3D::AngleUnit angle_unit,
         Math3D::LengthUnit length_unit)
    : Arm()
{
    (void)setModelFromMDH(alpha, a, d, theta_offset, angle_unit, length_unit);
}

Arm::Arm(const float alpha[kNumJoints],
         const float a[kNumJoints],
         const float d[kNumJoints],
         const float theta_offset[kNumJoints],
         const bool sigma[kNumJoints],
         Math3D::AngleUnit angle_unit,
         Math3D::LengthUnit length_unit)
    : Arm()
{
    (void)setModelFromMDH(alpha, a, d, theta_offset, sigma, angle_unit, length_unit);
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
    // 以“6个关节全部绑定”为执行前置条件。
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
    // 写入 home 模型并同步当前模型。
    M0_ = M;
    Slist0_ = Slist;
    M_ = M;
    Slist_ = Slist;
    // 仅在模型被显式配置后，笛卡尔目标接口才允许执行。
    model_configured_ = true;
}

bool Arm::setModelFromMDH(const float alpha[kNumJoints],
                          const float a[kNumJoints],
                          const float d[kNumJoints],
                          const float theta_offset[kNumJoints])
{
    return setModelFromMDH(alpha,
                           a,
                           d,
                           theta_offset,
                           Math3D::AngleUnit::Radians,
                           Math3D::LengthUnit::Meters);
}

bool Arm::setModelFromMDH(const float alpha[kNumJoints],
                          const float a[kNumJoints],
                          const float d[kNumJoints],
                          const float theta_offset[kNumJoints],
                          Math3D::AngleUnit angle_unit,
                          Math3D::LengthUnit length_unit)
{
    Math3D::Mat4 M_tmp = Math3D::eye4();
    Math3D::Mat6 Slist_tmp = Math3D::zeros6();
    const bool ok = Math3D::buildModelFromMDH(alpha,
                                              a,
                                              d,
                                              theta_offset,
                                              angle_unit,
                                              length_unit,
                                              &M_tmp,
                                              &Slist_tmp);
    if (!ok)
    {
        return false;
    }

    setModel(M_tmp, Slist_tmp);
    return true;
}

bool Arm::setModelFromMDH(const float alpha[kNumJoints],
                          const float a[kNumJoints],
                          const float d[kNumJoints],
                          const float theta_offset[kNumJoints],
                          const bool sigma[kNumJoints])
{
    return setModelFromMDH(alpha,
                           a,
                           d,
                           theta_offset,
                           sigma,
                           Math3D::AngleUnit::Radians,
                           Math3D::LengthUnit::Meters);
}

bool Arm::setModelFromMDH(const float alpha[kNumJoints],
                          const float a[kNumJoints],
                          const float d[kNumJoints],
                          const float theta_offset[kNumJoints],
                          const bool sigma[kNumJoints],
                          Math3D::AngleUnit angle_unit,
                          Math3D::LengthUnit length_unit)
{
    Math3D::Mat4 M_tmp = Math3D::eye4();
    Math3D::Mat6 Slist_tmp = Math3D::zeros6();
    const bool ok = Math3D::buildModelFromMDH(alpha,
                                              a,
                                              d,
                                              theta_offset,
                                              sigma,
                                              angle_unit,
                                              length_unit,
                                              &M_tmp,
                                              &Slist_tmp);
    if (!ok)
    {
        return false;
    }

    setModel(M_tmp, Slist_tmp);
    return true;
}

const Math3D::Mat4 &Arm::getHomeM() const
{
    return M0_;
}

const Math3D::Mat6 &Arm::getHomeSlist() const
{
    return Slist0_;
}

Math3D::Mat4 Arm::fkine(const float q[kNumJoints]) const
{
    float q_rad[kNumJoints] = {};
    degArrayToRad(q, q_rad);
    return Math3D::fkine(M_, Slist_, q_rad);
}

Math3D::Mat4 Arm::fkineCurrent() const
{
    float q_deg[kNumJoints] = {};
    float q_rad[kNumJoints] = {};
    captureCurrentJointPosition(q_deg);
    degArrayToRad(q_deg, q_rad);
    return Math3D::fkine(M_, Slist_, q_rad);
}

Math3D::Mat6 Arm::jacobian(const float q[kNumJoints]) const
{
    float q_rad[kNumJoints] = {};
    degArrayToRad(q, q_rad);
    return Math3D::cal_Jacobian(Slist_, q_rad);
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
    // Arm 对外约定角度单位为 deg；Math3D::ikine 内部使用 rad。
    float q0_rad[kNumJoints] = {};
    float q_out_rad[kNumJoints] = {};
    degArrayToRad(q0, q0_rad);

    const bool ok = Math3D::ikine(Slist_, M_, T_target, q0_rad, eomg, ev, q_out_rad, lambda, max_step, max_iterations);
    radArrayToDeg(q_out_rad, q_out);
    return ok;
}

bool Arm::setCartesianTarget(const Math3D::Mat4 &T_target,
                             float eomg,
                             float ev,
                             float lambda,
                             float max_step,
                             int max_iterations)
{
    // 保护策略：任何前置条件失败都保持当前关节角为目标，避免写入不确定命令。
    if (!isReady())
    {
        last_exec_status_ = ExecStatus::NOT_READY;
        holdCurrentAsTarget();
        return false;
    }

    if (!model_configured_)
    {
        last_exec_status_ = ExecStatus::MODEL_NOT_CONFIGURED;
        holdCurrentAsTarget();
        return false;
    }

    // 以当前姿态为 IK 初值，提高收敛稳定性。
    float q0[kNumJoints] = {};
    captureCurrentJointPosition(q0);

    float q_sol[kNumJoints] = {};
    // 统一走 Arm::ikine，保证 setCartesianTarget 与外部直接调用 ikine 的单位链路一致。
    const bool ok = ikine(T_target, q0, eomg, ev, q_sol, lambda, max_step, max_iterations);
    if (!ok)
    {
        last_exec_status_ = ExecStatus::IK_FAILED;
        holdCurrentAsTarget();
        return false;
    }

    setJointTarget(q_sol);
    last_exec_status_ = ExecStatus::IDLE;
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
    // 清空轨迹时同步复位运行态与故障统计，确保下一次启动从干净状态开始。
    waypoint_count_ = 0;
    running_ = false;
    last_exec_status_ = ExecStatus::IDLE;
    planner_sample_fail_count_ = 0;
    consecutive_sample_fail_count_ = 0;
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
    // 容量上限保护：固定数组缓存，超过上限直接拒绝。
    if (waypoint_count_ >= kMaxWaypoints)
    {
        return false;
    }

    // 对接 SRML TrajectoryPointArray 规则：除首点外，dt 不能为 0。
    // 否则 add() 会静默忽略该点，导致后续构建数量不一致。
    if (waypoint_count_ > 0 && duration_ms == 0)
    {
        return false;
    }

    // 过滤 NaN/Inf，避免插值器收到非法数值后行为不可预期。
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        if (!std::isfinite(q_target[i]))
        {
            return false;
        }
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
    // 启动前统一检查：绑定状态 + 路径数据一致性。
    if (!isReady())
    {
        last_exec_status_ = ExecStatus::NOT_READY;
        holdCurrentAsTarget();
        return false;
    }

    std::uint32_t total_ms = 0;
    if (!validateWaypointsAndComputeTotal(&total_ms))
    {
        last_exec_status_ = ExecStatus::INVALID_INPUT;
        holdCurrentAsTarget();
        return false;
    }

    float q_start[kNumJoints] = {};
    captureCurrentJointPosition(q_start);

    // 使用绝对时基，避免周期抖动带来的累积误差。
    trajectory_start_tick_ms_ = HAL_GetTick();
    trajectory_total_ms_ = total_ms;
    planner_sample_fail_count_ = 0;
    consecutive_sample_fail_count_ = 0;

    if (!buildJointTrajectories(q_start))
    {
        running_ = false;
        last_exec_status_ = ExecStatus::BUILD_FAILED;
        holdCurrentAsTarget();
        return false;
    }

    if (trajectory_total_ms_ == 0)
    {
        // 全 0 时长等价“立即到终点”。
        setJointTarget(waypoints_[waypoint_count_ - 1].q);
        running_ = false;
        last_exec_status_ = ExecStatus::FINISHED;
        return true;
    }

    running_ = true;
    last_exec_status_ = ExecStatus::RUNNING;
    return true;
}

void Arm::stopWaypoints()
{
    // 手动停止只退出运行态，不视为错误。
    running_ = false;
    if (last_exec_status_ == ExecStatus::RUNNING)
    {
        last_exec_status_ = ExecStatus::IDLE;
    }
}

bool Arm::waypointsRunning() const
{
    return running_;
}

Arm::ExecStatus Arm::getLastExecStatus() const
{
    return last_exec_status_;
}

uint16_t Arm::getPlannerSampleFailCount() const
{
    return planner_sample_fail_count_;
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

    // 每个关节独立执行 CQC 插值，并下发目标到 Joint。
    // 任一关节采样失败都视为本周期失败，触发保护计数。
    bool sample_ok = true;
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
        else
        {
            sample_ok = false;
        }
    }

    if (!sample_ok)
    {
        // 累计失败次数用于诊断，连续失败次数用于停机判定。
        ++planner_sample_fail_count_;
        if (consecutive_sample_fail_count_ < 0xFF)
        {
            ++consecutive_sample_fail_count_;
        }

        // 失败周期内保持当前目标，抑制目标跳变。
        holdCurrentAsTarget();

        // 连续失败达到阈值后判定轨迹不可继续执行，停止并上报码。
        if (consecutive_sample_fail_count_ >= kMaxConsecutiveSampleFail)
        {
            running_ = false;
            last_exec_status_ = ExecStatus::SAMPLE_FAILED;
            return;
        }
    }
    else
    {
        consecutive_sample_fail_count_ = 0;
    }

    if (elapsed_ms >= trajectory_total_ms_)
    {
        running_ = false;
        if (last_exec_status_ == ExecStatus::RUNNING)
        {
            last_exec_status_ = ExecStatus::FINISHED;
        }
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
    // 防御式检查：轨迹点数量必须处于有效范围。
    if (waypoint_count_ == 0 || waypoint_count_ > kTrajectoryPointCapacity)
    {
        return false;
    }

    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        JointTrajectoryChannel &channel = traj_[i];
        channel.planner.reset();
        channel.points.clear();

        for (uint8_t wp_idx = 0; wp_idx < waypoint_count_; ++wp_idx)
        {
            channel.points.add(waypoints_[wp_idx].duration_ms, waypoints_[wp_idx].q[i]);
        }

        // 若 add() 因输入非法静默丢点，这里通过数量一致性立即发现问题。
        if (channel.points.size() != waypoint_count_)
        {
            return false;
        }

        channel.planner.receive(channel.points, q_start[i]);
        channel.planner.compute(trajectory_start_tick_ms_);

        // 计算后做一次起点时刻可采样性探测，提前暴露构建失败。
        float q_check = q_start[i];
        if (!channel.planner.get(trajectory_start_tick_ms_, &q_check))
        {
            return false;
        }
    }

    return true;
}

bool Arm::validateWaypointsAndComputeTotal(std::uint32_t *total_ms) const
{
    // 调用方应传入有效输出指针，且路径点不能为空。
    if (total_ms == nullptr || waypoint_count_ == 0)
    {
        return false;
    }

    std::uint32_t sum_ms = 0;
    for (uint8_t wp_idx = 0; wp_idx < waypoint_count_; ++wp_idx)
    {
        const ArmWaypoint &wp = waypoints_[wp_idx];

        // 与 addWaypoint 规则保持一致：非首点时长不能为 0。
        if (wp_idx > 0 && wp.duration_ms == 0)
        {
            return false;
        }

        // 检查总时长累加是否会溢出 uint32_t。
        if ((0xFFFFFFFFu - sum_ms) < wp.duration_ms)
        {
            return false;
        }
        sum_ms += wp.duration_ms;

        // 每个关节目标都必须是有限值。
        for (uint8_t i = 0; i < kNumJoints; ++i)
        {
            if (!std::isfinite(wp.q[i]))
            {
                return false;
            }
        }
    }

    *total_ms = sum_ms;
    return true;
}

void Arm::holdCurrentAsTarget()
{
    // 保护动作：锁定当前反馈角作为目标，避免上层失败时电机继续追踪旧轨迹。
    float q_now[kNumJoints] = {};
    captureCurrentJointPosition(q_now);
    setJointTarget(q_now);
}

void Arm::degArrayToRad(const float in_deg[kNumJoints], float out_rad[kNumJoints])
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        out_rad[i] = Math3D::angleToRad(in_deg[i], Math3D::AngleUnit::Degrees);
    }
}

void Arm::radArrayToDeg(const float in_rad[kNumJoints], float out_deg[kNumJoints])
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        out_deg[i] = Math3D::radToAngle(in_rad[i], Math3D::AngleUnit::Degrees);
    }
}
