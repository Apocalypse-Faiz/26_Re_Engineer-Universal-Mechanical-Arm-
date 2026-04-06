#ifndef __ROBOT_ARM_H__
#define __ROBOT_ARM_H__

#include <stdint.h>

#include "Math3D.h"

class Joint;

/**
 * @brief 固定长度动作组点。
 */
struct ArmWaypoint
{
    float q[Math3D::kDoF];
    uint32_t duration_ms;
};

/**
 * @brief 6轴机械臂顶层控制器。
 * @note 不使用动态内存，所有缓存均为固定大小数组。
 */
class Arm
{
public:
    static const uint8_t kNumJoints = Math3D::kDoF;
    static const uint8_t kMaxWaypoints = 24;

    Arm();

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
     * @brief 前向运动学（输入关节角）。
     */
    Math3D::Mat4 fkine(const float q[kNumJoints]) const;

    /**
     * @brief 前向运动学（读取当前绑定关节角）。
     */
    Math3D::Mat4 fkineCurrent() const;

    /**
     * @brief 空间雅可比矩阵。
     */
    Math3D::Mat6 jacobian(const float q[kNumJoints]) const;

    /**
     * @brief 数值逆运动学（DLS）。
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
    bool addWaypoint(const float q_target[kNumJoints], uint32_t duration_ms);

    /**
     * @brief 以当前关节角作为起点启动路径执行。
     */
    bool startWaypointsFromCurrent();

    /**
     * @brief 停止路径执行。
     */
    void stopWaypoints();

    /**
     * @brief 路径是否正在运行。
     */
    bool waypointsRunning() const;

    /**
     * @brief 由外部 1kHz 任务周期调用。
     * @param dt_ms 本周期时间增量（ms）。
     */
    void update(uint32_t dt_ms);

private:
    Joint *joints_[kNumJoints];

    Math3D::Mat4 M_;
    Math3D::Mat6 Slist_;

    ArmWaypoint waypoints_[kMaxWaypoints];
    uint8_t waypoint_count_;

    bool running_;
    uint8_t segment_index_;
    uint32_t local_t_ms_;

    float segment_q_start_[kNumJoints];
    float segment_q_end_[kNumJoints];
    uint32_t segment_duration_ms_;

    /**
     * @brief 采集当前关节角。
     */
    void captureCurrentJointPosition(float q_out[kNumJoints]) const;

    /**
     * @brief 准备当前段插值起终点。
     */
    void prepareSegment(uint8_t index, const float start_q[kNumJoints]);

    /**
     * @brief Quintic 插值核：10u^3-15u^4+6u^5。
     */
    static float quinticBlend(float u);
};

#endif
