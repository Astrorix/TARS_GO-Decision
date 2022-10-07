#ifndef ROBORTS_DECISION_CHASSISLIMITED_H
#define ROBORTS_DECISION_CHASSISLIMITED_H
#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "../behavior_tree/behavior_tree.h"
#include "../blackboard/blackboard.h"
#include "../behavior_tree/behavior_state.h"
#include "../behavior_tree/behavior_node.h"
#include "../goal_factory.h"
#include "../executor/chassis_executor.h"
#include "roborts_msgs/GimbalSwingAction.h"


namespace roborts_decision{
    class ChassisLimited : public ActionNode {
    public:
        ChassisLimited(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) :
            ActionNode::ActionNode("chassis_limited", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr) {
                chassis_executor_ptr_ = blackboard_ptr_->chassis_executor_ptr_;
            
        }
                
        virtual ~ChassisLimited() = default;

    private:
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
        ros::Time start_time_;
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
        typedef actionlib::SimpleActionClient<roborts_msgs::GimbalSwingAction> Client;


        virtual void OnInitialize() {              
            start_time_=ros::Time::now();
            goal_factory_ptr_->CancelChassis();
            blackboard_ptr_->is_scan_=true;
            blackboard_ptr_->readytoshoot = 0;
            ROS_INFO("publish actionlib");
            ROS_INFO("%s %s",name_.c_str(),__FUNCTION__);
        }

        virtual BehaviorState Update() {
            ros::Time now_time=ros::Time::now();
            blackboard_ptr_->readytoshoot = 1;
            ROS_INFO("chassis limited time:%f",(now_time-start_time_).toSec());
            if((now_time-start_time_).toSec()>10.0){
	            goal_factory_ptr_->CancelGoal();
                goal_factory_ptr_->CancelScan();
                blackboard_ptr_->is_scan_=false;
                return BehaviorState::SUCCESS;
            }
        }
        
        virtual void OnTerminate(BehaviorState state) {
            switch (state){
            case BehaviorState::IDLE:
                goal_factory_ptr_->CancelGoal();
                ROS_INFO("%s %s IDLE!",name_.c_str(),__FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                goal_factory_ptr_->CancelScan();
                blackboard_ptr_->is_scan_=false;
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



    }; // class ShootAction
} //namespace roborts_decision

#endif
//ROBORTS_DECISION_CHASSISLIMITED_H