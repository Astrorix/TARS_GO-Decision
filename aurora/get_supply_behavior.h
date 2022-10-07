#ifndef ROBORTS_DECISION_EXCUTEGETSUPPLYACTION_H
#define ROBORTS_DECISION_EXCUTEGETSUPPLYACTION_H
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
    class GetSupplyBehavior : public ActionNode //副车特有
    {
    private:
        /* data */
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

    public:
        GetSupplyBehavior(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) : ActionNode::ActionNode("excute_get_supply", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
        {
        }
        virtual ~GetSupplyBehavior() = default;

        virtual void OnInitialize()
        {
            blackboard_ptr_->readytoshoot=0;
            ROS_INFO("Start Get Supply action at %f time", ros::Time::now().toSec());
            // ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
            // if(!is_master){
            //     if(blackboard_ptr_->command==0){
            //         geometry_msgs::PoseStamped selfPose = blackboard_ptr_->GetRobotMapPose();
            //         blackboard_ptr_->
            //     }
            // }
            // geometry_msgs::PoseStamped selfPose = blackboard_ptr_->GetRobotMapPose();

            // blackboard_ptr_ ->bullet_pose_;
            // blackboard_ptr_ ->blood_pose_;
            // bullet_pose_.pose.position.x;
            // bullet_pose_.pose.position.y;
        }
        virtual BehaviorState Update()
        {
            blackboard_ptr_->readytoshoot=0;
            ROS_INFO("Update Variables:");
            if (blackboard_ptr_->command == 0)
            {
                goal_factory_ptr_->GainBullet();
            }
            else if (blackboard_ptr_->command == 1)
            {
                goal_factory_ptr_->GainBlood();
            }
            else if (blackboard_ptr_->command == 2)
            {
                // geometry_msgs::PoseStamped pose;
                // pose.pose.position.x = blackboard_ptr_->goalpose_x;
                // pose.pose.position.y = blackboard_ptr_->goalpose_y;
                // tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, blackboard_ptr_->goalpose_yaw);
                // pose.pose.orientation.x = quaternion.x();
                // pose.pose.orientation.y = quaternion.y();
                // pose.pose.orientation.z = quaternion.z();
                // pose.pose.orientation.w = quaternion.w();
                goal_factory_ptr_->TakeCover(blackboard_ptr_->goalPose);
            }
            if (!blackboard_ptr_->is_master_) //（失败0 完成1 运行2）
                blackboard_ptr_->feedback = 2;
            else{
                if(blackboard_ptr_->feedback!=2)
                    blackboard_ptr_->command=-1;
            }
            blackboard_ptr_->action_state_ = blackboard_ptr_->chassis_executor_ptr_->Update();
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
                blackboard_ptr_->readytoshoot=1;
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