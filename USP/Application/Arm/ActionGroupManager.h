#ifndef __ACTION_GROUP_MANAGER_H__
#define __ACTION_GROUP_MANAGER_H__

#include <cstdint>

#include "Arm.h"

/**
 * @brief 动作组描述（只读视图，不拥有内存）。
 */
struct ArmActionGroup
{
    const ArmWaypoint *points;
    uint8_t count;
};

/**
 * @brief Arm 动作组管理器（多动作组注册 + 队列连播）。
 * @note 不使用动态内存，内部均为固定容量数组。
 */
class ActionGroupManager
{
public:
    static const uint8_t kInvalidGroupId = 0xFF;
    static const uint8_t kMaxGroups = 16;
    static const uint8_t kMaxQueue = 16;

    ActionGroupManager();
    explicit ActionGroupManager(Arm *arm);

    void bindArm(Arm *arm);

    bool registerGroup(uint8_t group_id, const ArmWaypoint *points, uint8_t count);
    bool registerGroup(uint8_t group_id, const ArmActionGroup &group);
    bool unregisterGroup(uint8_t group_id);
    void clearGroups();

    bool playNow(uint8_t group_id);
    bool enqueue(uint8_t group_id);
    void clearQueue();
    void stop();

    /**
     * @brief 轮询更新；内部会驱动 Arm::update() 并自动切换下一动作组。
     * @return true 当前存在正在运行的动作组，否则 false。
     */
    bool update();

    bool isRunning() const;
    bool hasQueuedGroup() const;
    uint8_t currentGroupId() const;

private:
    struct GroupEntry
    {
        bool used;
        uint8_t id;
        const ArmWaypoint *points;
        uint8_t count;
    };

    Arm *arm_;
    GroupEntry groups_[kMaxGroups];

    uint8_t queue_[kMaxQueue];
    uint8_t queue_head_;
    uint8_t queue_size_;

    bool running_;
    uint8_t current_group_id_;

    const GroupEntry *findGroup(uint8_t group_id) const;
    GroupEntry *findGroup(uint8_t group_id);
    GroupEntry *findFreeEntry();

    bool loadAndStart(const GroupEntry &entry);
    bool popNextGroupId(uint8_t *group_id);
};

#endif
