#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__

#include <cstdint>

#include "Math3D.h"
#include "Middlewares/Algorithm/Filters/SecondButterworthLPF.h"
#include "Middlewares/Algorithm/Trajectory/cqc.h"

class Joint;

/**
 * @brief 固定长度动作组点。
 */
struct ArmWaypoint
{
    float q[Math3D::kDoF];
    std::uint32_t duration_ms;
};

/**
 * @brief 6轴机械臂顶层控制器。
 * @note 轨迹插值基于 SRML CQC 算法；内部 TrajectoryPointArray 使用 vector 作为点缓存。
 */
class Arm
{
public:
    static const uint8_t kNumJoints = Math3D::kDoF;
    static const uint8_t kMaxWaypoints = 24;

    /**
     * @brief 轨迹/笛卡尔指令最近一次执行状态。
     */
    enum class ExecStatus : uint8_t
    {
        IDLE = 0,
        RUNNING,
        FINISHED,
        NOT_READY,
        INVALID_INPUT,
        MODE_MISMATCH,
        MODEL_NOT_CONFIGURED,
        BUILD_FAILED,
        SAMPLE_FAILED,
        IK_FAILED,
        ADMITTANCE_DISABLED,
        WRENCH_ESTIMATE_FAILED
    };

    /**
     * @brief 笛卡尔导纳控制参数（当前实现为平移 3 轴）。
        * @note 外力向量采用 twist 顺序：[wx, wy, wz, fx, fy, fz]。
     */
    struct AdmittanceConfig
    {
        bool enable;
        float mass[3];                 /**< 虚拟质量 M */
        float damping[3];              /**< 虚拟阻尼 D */
        float stiffness[3];            /**< 虚拟刚度 K */
        float wrench_lowpass_alpha;    /**< 兼容字段：外力一阶低通系数 [0,1]（当前实现不使用） */
        float force_lpf_cutoff_hz;     /**< 外力二阶低通截止频率（Hz） */
        float control_frequency_hz;    /**< 控制频率（Hz） */
        float force_lpf_damping;       /**< 二阶低通阻尼比 */
        float force_deadband_n;        /**< 力死区阈值（N） */
        float max_force_n;             /**< 力限幅（N） */
        float max_velocity_mps;        /**< 导纳速度限幅（m/s） */
        float max_position_offset_m;   /**< 相对参考位姿的位置偏移限幅（m） */
        float wrench_dls_lambda;       /**< 外力估计 DLS 阻尼系数 */

        AdmittanceConfig()
            : enable(false),
              wrench_lowpass_alpha(0.2f),
                            force_lpf_cutoff_hz(25.0f),
                            control_frequency_hz(1000.0f),
                            force_lpf_damping(0.70710678f),
              force_deadband_n(0.5f),
              max_force_n(80.0f),
              max_velocity_mps(0.25f),
              max_position_offset_m(0.08f),
              wrench_dls_lambda(0.10f)
        {
            mass[0] = 1.5f;
            mass[1] = 1.5f;
            mass[2] = 1.5f;

            damping[0] = 60.0f;
            damping[1] = 60.0f;
            damping[2] = 60.0f;

            stiffness[0] = 150.0f;
            stiffness[1] = 150.0f;
            stiffness[2] = 150.0f;
        }
    };

    /**
     * @brief 默认构造：使用单位模型（M=I, Slist=0）。
     */
    Arm();

    /**
     * @brief 构造时使用 MDH 参数创建零位模型（默认全转动关节）。
     */
    Arm(const float alpha[kNumJoints],
        const float a[kNumJoints],
        const float d[kNumJoints],
        const float theta_offset[kNumJoints],
        Math3D::AngleUnit angle_unit = Math3D::AngleUnit::Radians,
        Math3D::LengthUnit length_unit = Math3D::LengthUnit::Meters);

    /**
     * @brief 构造时使用 MDH 参数创建零位模型（可指定关节类型）。
     */
    Arm(const float alpha[kNumJoints],
        const float a[kNumJoints],
        const float d[kNumJoints],
        const float theta_offset[kNumJoints],
        const bool sigma[kNumJoints],
        Math3D::AngleUnit angle_unit = Math3D::AngleUnit::Radians,
        Math3D::LengthUnit length_unit = Math3D::LengthUnit::Meters);

    /**
     * @brief 绑定第 index 个关节。
     */
    void bindJoint(uint8_t index, Joint *joint);

    /**
     * @brief 判断 6 个关节是否全部绑定。
     */
    bool isReady() const;

    /**
     * @brief 设置 PoE 运动学模型参数。
     * @param M 零位位姿
     * @param Slist 空间旋量轴（列向量）
     */
    void setModel(const Math3D::Mat4 &M, const Math3D::Mat6 &Slist);

    /**
     * @brief 使用 MDH 参数自动构建并设置 PoE 模型（默认全转动关节）。
     */
    bool setModelFromMDH(const float alpha[kNumJoints],
                         const float a[kNumJoints],
                         const float d[kNumJoints],
                         const float theta_offset[kNumJoints]);

    /**
     * @brief 使用 MDH 参数自动构建并设置 PoE 模型（默认全转动关节，支持单位转换）。
     */
    bool setModelFromMDH(const float alpha[kNumJoints],
                         const float a[kNumJoints],
                         const float d[kNumJoints],
                         const float theta_offset[kNumJoints],
                         Math3D::AngleUnit angle_unit,
                         Math3D::LengthUnit length_unit);

    /**
     * @brief 使用 MDH 参数自动构建并设置 PoE 模型（可指定关节类型）。
     * @param sigma true=转动关节，false=移动关节
     */
    bool setModelFromMDH(const float alpha[kNumJoints],
                         const float a[kNumJoints],
                         const float d[kNumJoints],
                         const float theta_offset[kNumJoints],
                         const bool sigma[kNumJoints]);

    /**
     * @brief 使用 MDH 参数自动构建并设置 PoE 模型（可指定关节类型，支持单位转换）。
     */
    bool setModelFromMDH(const float alpha[kNumJoints],
                         const float a[kNumJoints],
                         const float d[kNumJoints],
                         const float theta_offset[kNumJoints],
                         const bool sigma[kNumJoints],
                         Math3D::AngleUnit angle_unit,
                         Math3D::LengthUnit length_unit);

    /**
     * @brief 获取零位模型（home）M。
     */
    const Math3D::Mat4 &getHomeM() const;

    /**
     * @brief 获取零位模型（home）Slist。
     */
    const Math3D::Mat6 &getHomeSlist() const;

    /**
     * @brief 前向运动学（输入关节角，单位 deg）。
     */
    Math3D::Mat4 fkine(const float q[kNumJoints]) const;

    /**
     * @brief 前向运动学（读取当前绑定关节角）。
     */
    Math3D::Mat4 fkineCurrent() const;

    /**
     * @brief 空间雅可比矩阵（输入关节角，单位 deg）。
     */
    Math3D::Mat6 jacobian(const float q[kNumJoints]) const;

    /**
     * @brief 数值逆运动学（DLS，q0/q_out 单位 deg）。
     */
    bool ikine(const Math3D::Mat4 &T_target,
               const float q0[kNumJoints],
               float eomg,
               float ev,
               float q_out[kNumJoints],
               float lambda = 0.05f,
               float max_step = 0.2f,
               int max_iterations = 200) const;

    /**
     * @brief 直接下发笛卡尔目标（内部调用 IK 得到关节目标）。
     */
    bool setCartesianTarget(const Math3D::Mat4 &T_target,
                            float eomg,
                            float ev,
                            float lambda = 0.05f,
                            float max_step = 0.2f,
                            int max_iterations = 200);

    /**
     * @brief 下发关节角目标（deg）。
     */
    void setJointTarget(const float q_target[kNumJoints]);

    /**
     * @brief 清空路径点并停止执行。
     */
    void clearWaypoints();

    /**
     * @brief 追加一个路径点。
     */
    bool addWaypoint(const float q_target[kNumJoints], std::uint32_t duration_ms);

    /**
     * @brief 以当前关节角作为起点启动路径执行。
     */
    bool startWaypointsFromCurrent();

    /**
        * @brief 停止路径执行并将目标锁定到当前反馈位姿。
     */
    void stopWaypoints();

    /**
     * @brief 路径是否正在运行。
     */
    bool waypointsRunning() const;

    /**
     * @brief 获取最近一次执行状态。
     */
    ExecStatus getLastExecStatus() const;

    /**
     * @brief 获取累计轨迹采样失败次数。
     */
    uint16_t getPlannerSampleFailCount() const;

    /**
     * @brief 设置导纳控制参数。
     */
    void setAdmittanceConfig(const AdmittanceConfig &config);

    /**
     * @brief 获取当前导纳控制参数。
     */
    const AdmittanceConfig &getAdmittanceConfig() const;

    /**
     * @brief 启用/关闭导纳控制。
     */
    void setAdmittanceEnabled(bool enable);

    /**
     * @brief 导纳控制是否启用。
     */
    bool isAdmittanceEnabled() const;

    /**
     * @brief 将导纳内部状态重置到当前末端位姿。
     */
    bool resetAdmittanceStateFromCurrentPose();

    /**
        * @brief 设置导纳参考位姿（未初始化时会同步重置导纳状态）。
        * @note 导纳位移偏移以该位姿平移分量作为零点。
     */
    void setAdmittanceReferencePose(const Math3D::Mat4 &T_ref);

    /**
     * @brief 由关节力矩反馈估计末端外力（wrench）。
        * @note 当前估计采用 tau_ext = tau_meas - tau_ff，再经 DLS 反解末端 wrench。
     */
    bool estimateExternalWrenchFromJointTorque(Math3D::Vec6 *wrench_out) const;

    /**
     * @brief 导纳控制周期更新（净外力矩 -> 外力估计 -> 导纳积分 -> IK -> 下发）。
        * @note 当前仅对平移 3 轴做导纳，姿态保持参考位姿不变。
     */
    bool updateAdmittance(float dt_s,
                          float ik_eomg = 1e-3f,
                          float ik_ev = 1e-3f,
                          float ik_lambda = 0.05f,
                          float ik_max_step = 0.2f,
                          int ik_max_iterations = 80);

    /**
     * @brief 获取当前导纳修正后的目标位姿。
     */
    bool getAdmittanceCommandPose(Math3D::Mat4 *T_cmd_out) const;

    /**
     * @brief 由外部周期调用，内部直接使用 HAL_GetTick() 作为时基。
     */
    void update();

private:
    static const uint16_t kTrajectoryPointCapacity = kMaxWaypoints;
    static const uint16_t kCqcCapacity = static_cast<uint16_t>(kMaxWaypoints + 1);
    static const uint8_t kMaxConsecutiveSampleFail = 3;

    struct JointTrajectoryChannel
    {
        TrajectoryPointArray points;
        CQC_<kCqcCapacity> planner;

        JointTrajectoryChannel() : points(kTrajectoryPointCapacity)
        {
        }
    };

    Joint *joints_[kNumJoints];

    // 零位模型（home）：由 setModel/setModelFromMDH 或构造函数建立。
    Math3D::Mat4 M0_;
    Math3D::Mat6 Slist0_;

    // 当前生效模型（默认与 home 相同，保留扩展空间）。
    Math3D::Mat4 M_;
    Math3D::Mat6 Slist_;
    bool model_configured_;

    ArmWaypoint waypoints_[kMaxWaypoints];
    uint8_t waypoint_count_;

    JointTrajectoryChannel traj_[kNumJoints];

    bool running_;
    ExecStatus last_exec_status_;
    uint16_t planner_sample_fail_count_;
    uint8_t consecutive_sample_fail_count_;
    std::uint32_t trajectory_start_tick_ms_;
    std::uint32_t trajectory_total_ms_;

    AdmittanceConfig admittance_config_;
    LPF_SecondOrder_Classdef admittance_force_lpf_x_;
    LPF_SecondOrder_Classdef admittance_force_lpf_y_;
    LPF_SecondOrder_Classdef admittance_force_lpf_z_;
    bool admittance_state_initialized_;
    // 导纳参考位姿（通常来自轨迹/规划器）。
    Math3D::Mat4 admittance_ref_pose_;
    // 导纳修正后的实时命令位姿。
    Math3D::Mat4 admittance_cmd_pose_;
    // 当前导纳平移状态（m, m/s）。
    float admittance_cmd_pos_m_[3];
    float admittance_vel_mps_[3];
    // 末端外力估计（原始/滤波后）。
    Math3D::Vec6 external_wrench_raw_;
    Math3D::Vec6 external_wrench_filtered_;

    /**
     * @brief 采集当前关节角。
     */
    void captureCurrentJointPosition(float q_out[kNumJoints]) const;

    /**
     * @brief 按动作组构建每个关节的 CQC 轨迹规划器。
     */
    bool buildJointTrajectories(const float q_start[kNumJoints]);

    /**
     * @brief 执行前一致性检查，并计算总时长。
     */
    bool validateWaypointsAndComputeTotal(std::uint32_t *total_ms) const;

    /**
     * @brief 检查全部关节是否可接收位置类目标（POSITION/IMPEDANCE）。
     */
    bool jointsAcceptPositionTarget() const;

    /**
     * @brief 标量限幅。
     */
    static float clampScalar(float x, float lo, float hi);

    /**
     * @brief 安全保护：保持当前关节角作为目标。
     */
    void holdCurrentAsTarget();

    // Arm 对外接口统一使用角度制（deg），内部 PoE 运算统一转弧度（rad）。
    static void degArrayToRad(const float in_deg[kNumJoints], float out_rad[kNumJoints]);
    static void radArrayToDeg(const float in_rad[kNumJoints], float out_deg[kNumJoints]);
};

#endif
