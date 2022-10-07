#ifndef ROBORTS_DECISION_ESCAPEACTION_H
#define ROBORTS_DECISION_ESCAPEACTION_H
#include <unistd.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../blackboard/EscapeGrid.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "../goal_factory.h"
#include "../executor/chassis_executor.h"

namespace roborts_decision
{
    class EsCape : public ActionNode
    {
    public:
        EsCape(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) : ActionNode::ActionNode("escape", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr),escapeGrid(blackboard_ptr,0.08)
        {
            chassis_executor_ptr_ = blackboard_ptr_->chassis_executor_ptr_;
        }
        virtual ~EsCape() = default;

    private:
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
        ros::Time start_time_;
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
        EscapeGrid escapeGrid;
        
        virtual void OnInitialize()
        {
            start_time_ = ros::Time::now();
            ROS_INFO("Start Escape action at %f time", ros::Time::now().toSec());
            escapeGrid.UpdateEscapeGrid();
            blackboard_ptr_->readytoshoot = 0;
            if(blackboard_ptr_->is_master_)
                blackboard_ptr_->command = 5;
        }

        virtual BehaviorState Update()
        {
            blackboard_ptr_->readytoshoot=0;
            ros::Time now_time = ros::Time::now();
            ROS_INFO("Update Variables:");
            ROS_INFO("--->Escape time:%f", (now_time - start_time_).toSec());
            if ((now_time - start_time_).toSec() > 0.5)
            {
                goal_factory_ptr_->CancelGoal();
                return BehaviorState::SUCCESS;
            }
            if (blackboard_ptr_->is_master_)
            {
                goal_factory_ptr_->Escape(escapeGrid);
                //goal_factory_ptr_->Escape();
            }
            else
            {
                goal_factory_ptr_->GoGoal(blackboard_ptr_->goalPose);
            }
            // ROS_INFO("escape");
            blackboard_ptr_->action_state_ = chassis_executor_ptr_->Update();
            return blackboard_ptr_->action_state_;
        }

        virtual void OnTerminate(BehaviorState state)
        {
            blackboard_ptr_->readytoshoot = 1;
            switch (state)
            {
            case BehaviorState::IDLE:
                goal_factory_ptr_->CancelGoal();
                ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                blackboard_ptr_->damage_type_ = -1;
                ROS_INFO("%s %s SUCCESS!", name_.c_str(), __FUNCTION__);
                break;
            case BehaviorState::FAILURE:
                ROS_INFO("%s %s FAILURE!", name_.c_str(), __FUNCTION__);
                break;
            default:
                ROS_INFO("%s %s ERROR!", name_.c_str(), __FUNCTION__);
                return;
            }
        }
    };
} //namespace roborts_decision
#endif