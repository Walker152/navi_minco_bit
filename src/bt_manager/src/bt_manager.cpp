#include "bt_manager/bt_manager.hpp"

namespace Sentry_BT
{
    BTManager::BTManager()
    : blackboard_(std::make_shared<Blackboard>())
    {
    }

    bool BTManager::initialize(const std::string & xml_file_path)
    {
        // 注册自定义节点
        factory_.registerNodeType<CheckRetreatCondition>("CheckRetreatCondition");
        factory_.registerNodeType<CheckFortBonusActive>("CheckFortBonusActive");
        factory_.registerNodeType<CheckTargetLocked>("CheckTargetLocked");
        factory_.registerNodeType<SetHomeCoordinate>("SetHomeCoordinate");
        factory_.registerNodeType<PublishNavigationGoal>("PublishNavigationGoal");
        factory_.registerNodeType<SetBonusCoordinate>("SetBonusCoordinate");
        factory_.registerNodeType<SetTargetCoordinate>("SetTargetCoordinate");
        factory_.registerNodeType<SelectInspectionArea>("SelectInspectionArea");
        factory_.registerNodeType<SetAreaCoordinate>("SetAreaCoordinate");
        factory_.registerNodeType<SelectPatrolPoint>("SelectPatrolPoint");

        // 创建行为树
        try
        {
            tree_ = factory_.createTreeFromFile(xml_file_path, blackboard_->getBTBlackboard());
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
        tree_.rootNode()->executeTick();
    }

    BT::NodeStatus BTManager::getStatus() const
    {
        return tree_.rootNode()->status();
    }

    std::shared_ptr<Blackboard> BTManager::getBlackboard()
    {
        return blackboard_;
    }
}  // namespace Sentry_BT