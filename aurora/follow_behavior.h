#ifndef ROBORTS_DECISION_FOLLOW_H
#define ROBORTS_DECISION_FOLLOW_H
#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "../goal_factory.h"
#include "../executor/chassis_executor.h"
namespace roborts_decision{
    class Follow : public ActionNode {
    public:
        Follow(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
            ActionNode::ActionNode("follow_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
                chassis_executor_ptr_ = blackboard_ptr_-> chassis_executor_ptr_;
        }

        virtual ~Follow() = default;
    private:
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
        ros::Time start_time_;
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
        virtual void OnInitialize() {      
            blackboard_ptr_->readytoshoot=1;        
            start_time_=ros::Time::now();
            blackboard_ptr_->is_follow_=true;
            ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
        }
        virtual BehaviorState Update() {
            blackboard_ptr_->readytoshoot=1;
            ros::Time now_time=ros::Time::now();
            ROS_INFO("follow time:%f",(now_time-start_time_).toSec());
            if((now_time-start_time_).toSec()>0.02){
	            blackboard_ptr_->is_follow_=false;
	            goal_factory_ptr_->CancelGoal();
                return BehaviorState::SUCCESS;
            }
           goal_factory_ptr_->GoStopPose();
            ROS_INFO("gobuff！");
            blackboard_ptr_-> action_state_ = chassis_executor_ptr_->Update();
            return blackboard_ptr_-> action_state_;
        }
        virtual void OnTerminate(BehaviorState state) {
            blackboard_ptr_->readytoshoot=1;
            switch (state){
            case BehaviorState::IDLE:
                goal_factory_ptr_->CancelGoal();
                ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                blackboard_ptr_->is_follow_=false;
                blackboard_ptr_->has_followed_=true;
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

    }; 
} //namespace roborts_decision
#endif