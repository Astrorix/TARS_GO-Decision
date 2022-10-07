#ifndef ROBORTS_DECISION_GIMBALLIMITED_H
#define ROBORTS_DECISION_GIMBALLIMITED_H
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

namespace roborts_decision{
    class GimbalLimited : public ActionNode {
    public:
        GimbalLimited(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
            ActionNode::ActionNode("gimbal_limited", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr),escapeGrid(blackboard_ptr,0.08) {
                chassis_executor_ptr_ = blackboard_ptr_->chassis_executor_ptr_;
        }

        virtual ~GimbalLimited() = default;

    private:
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
        ros::Time start_time_;
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
        EscapeGrid escapeGrid;

        virtual void OnInitialize() {              
            start_time_=ros::Time::now();
            ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
            escapeGrid.UpdateEscapeGrid();
        }

        virtual BehaviorState Update() {
            ros::Time now_time=ros::Time::now();
            ROS_INFO("gimbal limited time:%f",(now_time-start_time_).toSec());
            if((now_time-start_time_).toSec()>10.0){
	            goal_factory_ptr_->CancelGoal();
                return BehaviorState::SUCCESS;
            }
            goal_factory_ptr_->Escape(escapeGrid);
            ROS_INFO("under punishment and escape ！");
            blackboard_ptr_-> action_state_ = chassis_executor_ptr_->Update();
            return blackboard_ptr_->action_state_;
        }

        virtual void OnTerminate(BehaviorState state) {
            switch (state){
            case BehaviorState::IDLE:
                goal_factory_ptr_->CancelGoal();
                ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                ROS_INFO("%s %s SUCCESS!",name_.c_str(),__FUNCTION__);
                break;
            case BehaviorState::FAILURE:
                ROS_INFO("%s %s FAILURE!",name_.c_str(),__FUNCTION__);
                break;
            default:
                ROS_INFO("%s %s ERROR!",name_.c_str(),__FUNCTION__);
                return;
            }
        }



    }; // class GimbalAction
} //namespace roborts_decision

#endif
//ROBORTS_DECISION_GIMBALLIMITED_H