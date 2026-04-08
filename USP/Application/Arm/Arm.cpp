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
            trajectory_total_ms_(0),
    admittance_force_lpf_x_(25.0f, 1000.0f),
    admittance_force_lpf_y_(25.0f, 1000.0f),
    admittance_force_lpf_z_(25.0f, 1000.0f),
            admittance_state_initialized_(false)
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

    admittance_ref_pose_ = M0_;
    admittance_cmd_pose_ = M0_;

    for (uint8_t i = 0; i < 3; ++i)
    {
        admittance_cmd_pos_m_[i] = 0.0f;
        admittance_vel_mps_[i] = 0.0f;
    }

    external_wrench_raw_ = Math3D::makeVec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    external_wrench_filtered_ = external_wrench_raw_;
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

    // 笛卡尔目标最终会落到关节角目标，关节必须处于可接收位置命令的模式。
    if (!jointsAcceptPositionTarget())
    {
        last_exec_status_ = ExecStatus::MODE_MISMATCH;
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

    // 轨迹执行依赖 setTargetAngle 下发，模式必须允许位置类目标。
    if (!jointsAcceptPositionTarget())
    {
        last_exec_status_ = ExecStatus::MODE_MISMATCH;
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
    // 手动停止时将目标锁定到当前反馈位姿，避免继续追踪旧轨迹目标。
    if (running_)
    {
        holdCurrentAsTarget();
    }

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

void Arm::setAdmittanceConfig(const AdmittanceConfig &config)
{
    // 参数入口统一做健壮化，避免负阻尼/零质量等导致数值不稳定。
    admittance_config_ = config;

    for (uint8_t i = 0; i < 3; ++i)
    {
        if (admittance_config_.mass[i] < 1e-4f)
        {
            admittance_config_.mass[i] = 1e-4f;
        }
        if (admittance_config_.damping[i] < 0.0f)
        {
            admittance_config_.damping[i] = 0.0f;
        }
        if (admittance_config_.stiffness[i] < 0.0f)
        {
            admittance_config_.stiffness[i] = 0.0f;
        }
    }

    admittance_config_.wrench_lowpass_alpha = clampScalar(admittance_config_.wrench_lowpass_alpha, 0.0f, 1.0f);
    if (admittance_config_.force_lpf_cutoff_hz < 1e-3f)
    {
        admittance_config_.force_lpf_cutoff_hz = 1e-3f;
    }
    if (admittance_config_.control_frequency_hz < 1e-3f)
    {
        admittance_config_.control_frequency_hz = 1e-3f;
    }
    if (admittance_config_.force_lpf_damping < 0.0f)
    {
        admittance_config_.force_lpf_damping = 0.0f;
    }
    if (admittance_config_.force_deadband_n < 0.0f)
    {
        admittance_config_.force_deadband_n = 0.0f;
    }
    if (admittance_config_.max_force_n < 0.0f)
    {
        admittance_config_.max_force_n = 0.0f;
    }
    if (admittance_config_.max_velocity_mps < 0.0f)
    {
        admittance_config_.max_velocity_mps = 0.0f;
    }
    if (admittance_config_.max_position_offset_m < 0.0f)
    {
        admittance_config_.max_position_offset_m = 0.0f;
    }
    if (admittance_config_.wrench_dls_lambda < 1e-4f)
    {
        admittance_config_.wrench_dls_lambda = 1e-4f;
    }

    // 复用 SRML 二阶巴特沃斯低通滤波器处理外力分量。
    admittance_force_lpf_x_.calculateCoefficients(admittance_config_.force_lpf_cutoff_hz,
                                                  admittance_config_.control_frequency_hz,
                                                  admittance_config_.force_lpf_damping);
    admittance_force_lpf_y_.calculateCoefficients(admittance_config_.force_lpf_cutoff_hz,
                                                  admittance_config_.control_frequency_hz,
                                                  admittance_config_.force_lpf_damping);
    admittance_force_lpf_z_.calculateCoefficients(admittance_config_.force_lpf_cutoff_hz,
                                                  admittance_config_.control_frequency_hz,
                                                  admittance_config_.force_lpf_damping);

    if (!admittance_config_.enable)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            admittance_vel_mps_[i] = 0.0f;
        }
    }
}

const Arm::AdmittanceConfig &Arm::getAdmittanceConfig() const
{
    return admittance_config_;
}

void Arm::setAdmittanceEnabled(bool enable)
{
    admittance_config_.enable = enable;
    if (!admittance_config_.enable)
    {
        for (uint8_t i = 0; i < 3; ++i)
        {
            admittance_vel_mps_[i] = 0.0f;
        }
        return;
    }

    if (!admittance_state_initialized_)
    {
        (void)resetAdmittanceStateFromCurrentPose();
    }
}

bool Arm::isAdmittanceEnabled() const
{
    return admittance_config_.enable;
}

bool Arm::resetAdmittanceStateFromCurrentPose()
{
    if (!isReady() || !model_configured_)
    {
        return false;
    }

    // 以当前位姿作为导纳初值，避免首次启用时出现跳变。
    admittance_ref_pose_ = fkineCurrent();
    admittance_cmd_pose_ = admittance_ref_pose_;

    for (uint8_t i = 0; i < 3; ++i)
    {
        admittance_cmd_pos_m_[i] = admittance_ref_pose_.m[i][3];
        admittance_vel_mps_[i] = 0.0f;
    }

    external_wrench_raw_ = Math3D::makeVec6(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    external_wrench_filtered_ = external_wrench_raw_;
    admittance_force_lpf_x_.reset();
    admittance_force_lpf_y_.reset();
    admittance_force_lpf_z_.reset();
    admittance_state_initialized_ = true;
    return true;
}

void Arm::setAdmittanceReferencePose(const Math3D::Mat4 &T_ref)
{
    admittance_ref_pose_ = T_ref;

    if (!admittance_state_initialized_)
    {
        admittance_cmd_pose_ = T_ref;
        for (uint8_t i = 0; i < 3; ++i)
        {
            admittance_cmd_pos_m_[i] = T_ref.m[i][3];
            admittance_vel_mps_[i] = 0.0f;
        }
        admittance_force_lpf_x_.reset();
        admittance_force_lpf_y_.reset();
        admittance_force_lpf_z_.reset();
        admittance_state_initialized_ = true;
    }
}

bool Arm::estimateExternalWrenchFromJointTorque(Math3D::Vec6 *wrench_out) const
{
    if (wrench_out == nullptr || !isReady())
    {
        return false;
    }

    // 当前关节姿态下的空间雅可比 J(q)。
    float q_deg[kNumJoints] = {};
    captureCurrentJointPosition(q_deg);
    const Math3D::Mat6 J = jacobian(q_deg);
    const Math3D::Mat6 JT = Math3D::mat6Transpose(J);

    Math3D::Vec6 tau_ext = {};
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        if (joints_[i] == nullptr)
        {
            return false;
        }

        // 关节净外力矩估计：测量值减去已知模型前馈项。
        tau_ext.v[i] = joints_[i]->getTorque() - joints_[i]->getTotalTauFF();
    }

    // DLS 反解末端外力：F = (J*J^T + lambda^2 I)^-1 * J * tau_ext。
    const Math3D::Mat6 JJT = Math3D::mat6Mul(J, JT);
    const float lambda2 = admittance_config_.wrench_dls_lambda * admittance_config_.wrench_dls_lambda;
    const Math3D::Mat6 regularization = Math3D::mat6Scale(Math3D::eye6(), lambda2);
    const Math3D::Mat6 A = Math3D::mat6Add(JJT, regularization);
    const Math3D::Vec6 b = Math3D::mat6MulVec6(J, tau_ext);

    Math3D::Vec6 wrench = {};
    if (!Math3D::solveLinear6x6(A, b, &wrench))
    {
        return false;
    }

    *wrench_out = wrench;
    return true;
}

bool Arm::updateAdmittance(float dt_s,
                           float ik_eomg,
                           float ik_ev,
                           float ik_lambda,
                           float ik_max_step,
                           int ik_max_iterations)
{
    if (!admittance_config_.enable)
    {
        last_exec_status_ = ExecStatus::ADMITTANCE_DISABLED;
        return false;
    }

    if (!(dt_s > 0.0f) || !std::isfinite(dt_s))
    {
        last_exec_status_ = ExecStatus::INVALID_INPUT;
        return false;
    }

    if (!isReady())
    {
        last_exec_status_ = ExecStatus::NOT_READY;
        holdCurrentAsTarget();
        return false;
    }

    if (!jointsAcceptPositionTarget())
    {
        last_exec_status_ = ExecStatus::MODE_MISMATCH;
        holdCurrentAsTarget();
        return false;
    }

    if (!model_configured_)
    {
        last_exec_status_ = ExecStatus::MODEL_NOT_CONFIGURED;
        holdCurrentAsTarget();
        return false;
    }

    // 启用后首次运行自动做状态对齐，避免导纳内部状态未初始化。
    if (!admittance_state_initialized_ && !resetAdmittanceStateFromCurrentPose())
    {
        last_exec_status_ = ExecStatus::INVALID_INPUT;
        holdCurrentAsTarget();
        return false;
    }

    // 1) 外力估计（关节净外力矩 -> 末端 wrench）。
    Math3D::Vec6 wrench_raw = {};
    if (!estimateExternalWrenchFromJointTorque(&wrench_raw))
    {
        last_exec_status_ = ExecStatus::WRENCH_ESTIMATE_FAILED;
        holdCurrentAsTarget();
        return false;
    }
    external_wrench_raw_ = wrench_raw;

    // 2) 外力滤波：角力矩分量直接透传，线力分量由 SRML 二阶低通处理。
    external_wrench_filtered_.v[0] = external_wrench_raw_.v[0];
    external_wrench_filtered_.v[1] = external_wrench_raw_.v[1];
    external_wrench_filtered_.v[2] = external_wrench_raw_.v[2];
    external_wrench_filtered_.v[3] = admittance_force_lpf_x_.f(external_wrench_raw_.v[3]);
    external_wrench_filtered_.v[4] = admittance_force_lpf_y_.f(external_wrench_raw_.v[4]);
    external_wrench_filtered_.v[5] = admittance_force_lpf_z_.f(external_wrench_raw_.v[5]);

    // 3) 平移 3 轴导纳积分：死区 + 限幅 + M-D-K 离散更新。
    // 当前实现使用 wrench 线力分量 [3..5] -> [fx, fy, fz]。
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        float f = external_wrench_filtered_.v[axis + 3];

        if (std::fabs(f) < admittance_config_.force_deadband_n)
        {
            f = 0.0f;
        }
        f = clampScalar(f, -admittance_config_.max_force_n, admittance_config_.max_force_n);
        external_wrench_filtered_.v[axis + 3] = f;

        const float ref_pos = admittance_ref_pose_.m[axis][3];
        const float pos_err = admittance_cmd_pos_m_[axis] - ref_pos;

        const float acc = (f - admittance_config_.damping[axis] * admittance_vel_mps_[axis] -
                           admittance_config_.stiffness[axis] * pos_err) /
                          admittance_config_.mass[axis];

        admittance_vel_mps_[axis] += acc * dt_s;
        admittance_vel_mps_[axis] = clampScalar(admittance_vel_mps_[axis],
                                                -admittance_config_.max_velocity_mps,
                                                admittance_config_.max_velocity_mps);

        admittance_cmd_pos_m_[axis] += admittance_vel_mps_[axis] * dt_s;

        float offset = admittance_cmd_pos_m_[axis] - ref_pos;
        offset = clampScalar(offset,
                             -admittance_config_.max_position_offset_m,
                             admittance_config_.max_position_offset_m);
        admittance_cmd_pos_m_[axis] = ref_pos + offset;
    }

    admittance_cmd_pose_ = admittance_ref_pose_;
    for (uint8_t axis = 0; axis < 3; ++axis)
    {
        admittance_cmd_pose_.m[axis][3] = admittance_cmd_pos_m_[axis];
    }

    // 4) 由导纳修正位姿做 IK，得到关节目标并下发。
    float q0[kNumJoints] = {};
    captureCurrentJointPosition(q0);

    float q_cmd[kNumJoints] = {};
    if (!ikine(admittance_cmd_pose_, q0, ik_eomg, ik_ev, q_cmd, ik_lambda, ik_max_step, ik_max_iterations))
    {
        last_exec_status_ = ExecStatus::IK_FAILED;
        holdCurrentAsTarget();
        return false;
    }

    setJointTarget(q_cmd);
    last_exec_status_ = ExecStatus::IDLE;
    return true;
}

bool Arm::getAdmittanceCommandPose(Math3D::Mat4 *T_cmd_out) const
{
    if (T_cmd_out == nullptr || !admittance_state_initialized_)
    {
        return false;
    }

    *T_cmd_out = admittance_cmd_pose_;
    return true;
}

void Arm::update()
{
    if (!running_)
    {
        return;
    }

    // 运行过程中若被外部切到非位置模式，立即停止并上报模式不一致。
    if (!jointsAcceptPositionTarget())
    {
        running_ = false;
        last_exec_status_ = ExecStatus::MODE_MISMATCH;
        holdCurrentAsTarget();
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

bool Arm::jointsAcceptPositionTarget() const
{
    for (uint8_t i = 0; i < kNumJoints; ++i)
    {
        if (joints_[i] == nullptr)
        {
            return false;
        }

        const JointMode mode = joints_[i]->getMode();
        if (mode != JointMode::MODE_POSITION && mode != JointMode::MODE_IMPEDANCE)
        {
            return false;
        }
    }

    return true;
}

float Arm::clampScalar(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
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
