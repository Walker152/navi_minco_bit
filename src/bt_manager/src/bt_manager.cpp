#include "bt_manager/bt_manager.hpp"

namespace Sentry_BT
{
    BTManager::BTManager()
    {
    }

    bool BTManager::initialize(const std::string & xml_file_path, std::shared_ptr<BT::Blackboard> blackboard)
    {
        // 注册自定义节点
        factory_.registerNodeType<CheckRetreatCondition>("CheckRetreatCondition");
        factory_.registerNodeType<CheckFortBonusActive>("CheckFortBonusActive");
        factory_.registerNodeType<CheckTargetLocked>("CheckTargetLocked");
        factory_.registerNodeType<PublishNavigationGoal>("PublishNavigationGoal");
        factory_.registerNodeType<SelectPatrolPoint>("SelectPatrolPoint");
        factory_.registerNodeType<SetTargetCoordinate>("SetTargetCoordinate");
        factory_.registerNodeType<CheckNavStatus>("CheckNavStatus");
        factory_.registerNodeType<CheckIfRetreating>("CheckIfRetreating");
        factory_.registerNodeType<SetCoordinate>("SetCoordinate");
        factory_.registerNodeType<WaitUntilStopped>("WaitUntilStopped");
        factory_.registerNodeType<Wait>("Wait");

        // 创建行为树
        try
        {
            tree_ = factory_.createTreeFromFile(xml_file_path, blackboard);
        }
        catch (const std::exception & e)
        {
            std::cerr << "Error creating behavior tree: " << e.what() << std::endl;
            return false;
        }

        return true;
    }

    void BTManager::execute()
    {
        tree_.tickRoot();
    }

    BT::NodeStatus BTManager::getStatus() const
    {
        return tree_.rootNode()->status();
    }
}  // namespace Sentry_BT