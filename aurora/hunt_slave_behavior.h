#ifndef ROBORTS_DECISION_EXCUTEHUNTACTION_H
#define ROBORTS_DECISION_EXCUTEHUNTACTION_H
#include <unistd.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "../goal_factory.h"
#include "../executor/chassis_executor.h"
namespace roborts_decision
{
    class ExcuteHuntAction : public ActionNode
    {
    private:
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;

    public:
        ExcuteHuntAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) : ActionNode::ActionNode("excute_hunt", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
        {
            chassis_executor_ptr_ = blackboard_ptr_->chassis_executor_ptr_;
        }
        virtual ~ExcuteHuntAction() = default;
        virtual void OnInitialize()
        {
            // ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
            ROS_INFO("Start Hunt action at %f time", ros::Time::now().toSec());
        }
        virtual BehaviorState Update()
        {
            ROS_INFO("Update Variables:");
            goal_factory_ptr_->E_Hunt();
            // ROS_INFO("E_hunt");
            if (!blackboard_ptr_->is_master_) //（失败0 完成1 运行2）
                blackboard_ptr_->feedback = 2;
            blackboard_ptr_->action_state_ = chassis_executor_ptr_->Update();
            return blackboard_ptr_->action_state_;
        }
        virtual void OnTerminate(BehaviorState state)
        {
            switch (state)
            {
            case BehaviorState::IDLE:
                goal_factory_ptr_->CancelGoal();
                ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                if (!blackboard_ptr_->is_master_)
                    blackboard_ptr_->feedback = 1;
                ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
                break;
            case BehaviorState::FAILURE:
                if (!blackboard_ptr_->is_master_)
                    blackboard_ptr_->feedback = 0;
                ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
                break;
            default:
                ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
                return;
            }
        }
    };
}

#endif