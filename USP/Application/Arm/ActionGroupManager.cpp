#include "ActionGroupManager.h"

ActionGroupManager::ActionGroupManager()
    : arm_(nullptr),
      queue_head_(0),
      queue_size_(0),
      running_(false),
      current_group_id_(kInvalidGroupId)
{
    clearGroups();
}

ActionGroupManager::ActionGroupManager(Arm *arm)
    : arm_(arm),
      queue_head_(0),
      queue_size_(0),
      running_(false),
      current_group_id_(kInvalidGroupId)
{
    clearGroups();
}

void ActionGroupManager::bindArm(Arm *arm)
{
    arm_ = arm;
}

bool ActionGroupManager::registerGroup(uint8_t group_id, const ArmWaypoint *points, uint8_t count)
{
    if (points == nullptr || count == 0 || count > Arm::kMaxWaypoints)
    {
        return false;
    }

    GroupEntry *entry = findGroup(group_id);
    if (entry == nullptr)
    {
        entry = findFreeEntry();
        if (entry == nullptr)
        {
            return false;
        }
    }

    entry->used = true;
    entry->id = group_id;
    entry->points = points;
    entry->count = count;
    return true;
}

bool ActionGroupManager::registerGroup(uint8_t group_id, const ArmActionGroup &group)
{
    return registerGroup(group_id, group.points, group.count);
}

bool ActionGroupManager::unregisterGroup(uint8_t group_id)
{
    GroupEntry *entry = findGroup(group_id);
    if (entry == nullptr)
    {
        return false;
    }

    entry->used = false;
    entry->id = kInvalidGroupId;
    entry->points = nullptr;
    entry->count = 0;
    return true;
}

void ActionGroupManager::clearGroups()
{
    for (uint8_t i = 0; i < kMaxGroups; ++i)
    {
        groups_[i].used = false;
        groups_[i].id = kInvalidGroupId;
        groups_[i].points = nullptr;
        groups_[i].count = 0;
    }
}

bool ActionGroupManager::playNow(uint8_t group_id)
{
    if (arm_ == nullptr)
    {
        return false;
    }

    const GroupEntry *entry = findGroup(group_id);
    if (entry == nullptr)
    {
        return false;
    }

    clearQueue();
    arm_->stopWaypoints();
    return loadAndStart(*entry);
}

bool ActionGroupManager::enqueue(uint8_t group_id)
{
    if (queue_size_ >= kMaxQueue)
    {
        return false;
    }

    if (findGroup(group_id) == nullptr)
    {
        return false;
    }

    const uint8_t tail = static_cast<uint8_t>((queue_head_ + queue_size_) % kMaxQueue);
    queue_[tail] = group_id;
    ++queue_size_;
    return true;
}

void ActionGroupManager::clearQueue()
{
    queue_head_ = 0;
    queue_size_ = 0;
}

void ActionGroupManager::stop()
{
    if (arm_ != nullptr)
    {
        arm_->stopWaypoints();
    }

    running_ = false;
    current_group_id_ = kInvalidGroupId;
    clearQueue();
}

bool ActionGroupManager::update()
{
    if (arm_ == nullptr)
    {
        return false;
    }

    if (running_)
    {
        arm_->update();
        if (arm_->waypointsRunning())
        {
            return true;
        }

        running_ = false;
        current_group_id_ = kInvalidGroupId;
    }

    uint8_t next_group_id = kInvalidGroupId;
    if (popNextGroupId(&next_group_id))
    {
        const GroupEntry *entry = findGroup(next_group_id);
        if (entry != nullptr)
        {
            return loadAndStart(*entry);
        }
    }

    return false;
}

bool ActionGroupManager::isRunning() const
{
    return running_;
}

bool ActionGroupManager::hasQueuedGroup() const
{
    return queue_size_ > 0;
}

uint8_t ActionGroupManager::currentGroupId() const
{
    return current_group_id_;
}

const ActionGroupManager::GroupEntry *ActionGroupManager::findGroup(uint8_t group_id) const
{
    for (uint8_t i = 0; i < kMaxGroups; ++i)
    {
        if (groups_[i].used && groups_[i].id == group_id)
        {
            return &groups_[i];
        }
    }
    return nullptr;
}

ActionGroupManager::GroupEntry *ActionGroupManager::findGroup(uint8_t group_id)
{
    for (uint8_t i = 0; i < kMaxGroups; ++i)
    {
        if (groups_[i].used && groups_[i].id == group_id)
        {
            return &groups_[i];
        }
    }
    return nullptr;
}

ActionGroupManager::GroupEntry *ActionGroupManager::findFreeEntry()
{
    for (uint8_t i = 0; i < kMaxGroups; ++i)
    {
        if (!groups_[i].used)
        {
            return &groups_[i];
        }
    }
    return nullptr;
}

bool ActionGroupManager::loadAndStart(const GroupEntry &entry)
{
    if (arm_ == nullptr || entry.points == nullptr || entry.count == 0)
    {
        return false;
    }

    arm_->clearWaypoints();
    for (uint8_t i = 0; i < entry.count; ++i)
    {
        if (!arm_->addWaypoint(entry.points[i].q, entry.points[i].duration_ms))
        {
            arm_->clearWaypoints();
            running_ = false;
            current_group_id_ = kInvalidGroupId;
            return false;
        }
    }

    if (!arm_->startWaypointsFromCurrent())
    {
        running_ = false;
        current_group_id_ = kInvalidGroupId;
        return false;
    }

    running_ = true;
    current_group_id_ = entry.id;
    return true;
}

bool ActionGroupManager::popNextGroupId(uint8_t *group_id)
{
    if (group_id == nullptr || queue_size_ == 0)
    {
        return false;
    }

    *group_id = queue_[queue_head_];
    queue_head_ = static_cast<uint8_t>((queue_head_ + 1) % kMaxQueue);
    --queue_size_;
    return true;
}
