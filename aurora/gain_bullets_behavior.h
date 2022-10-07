#ifndef ROBORTS_DECISION_GAIN_BULLETS_H
#define ROBORTS_DECISION_GAIN_BULLETS_H
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
    class GainBullets : public ActionNode
    {
    public:
        GainBullets(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) : ActionNode::ActionNode("gain_bullets_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
        {
        }

        virtual ~GainBullets() = default;

    private:
        ros::Time start_time_;
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;

        virtual void OnInitialize()
        {
            blackboard_ptr_->readytoshoot=0;
            blackboard_ptr_->is_go_buff_ = true;
            start_time_=ros::Time::now();
            ROS_INFO("Start Get Supply(bullet) action at %f time", ros::Time::now().toSec());
            if (blackboard_ptr_->is_connected)
            {
                geometry_msgs::PoseStamped selfPose = blackboard_ptr_->GetRobotMapPose();
                float mDistance = blackboard_ptr_->GetDistance(selfPose, blackboard_ptr_->bullet_pose_);
                float sDistance = blackboard_ptr_->GetDistance(blackboard_ptr_->matePose, blackboard_ptr_->bullet_pose_);

                if (mDistance < sDistance) //如果主车距离buff点更近
                {
                    blackboard_ptr_->command = 2; // take cover
                    blackboard_ptr_->goalpose_x = blackboard_ptr_->bullet_pose_.pose.position.x;
                    blackboard_ptr_->goalpose_y = blackboard_ptr_->bullet_pose_.pose.position.y;
                    double yaw = tf::getYaw(blackboard_ptr_->bullet_pose_.pose.orientation);
                    blackboard_ptr_->goalpose_yaw = yaw;
                    //副车掩护bullet_pose_
                }
                else
                {
                      blackboard_ptr_->command = 0; // 副车 get bullet  主车 take cover
                }
            }
        }

        virtual BehaviorState Update()
        {
            blackboard_ptr_->readytoshoot=0;
                  ROS_INFO("Update Variables:");
            ros::Time now_time = ros::Time::now();
            ROS_INFO("--->Gain bullets action time:%f", (now_time - start_time_).toSec());
            if ((now_time - start_time_).toSec() > 20 || !blackboard_ptr_->bullet_active_)
            {
                goal_factory_ptr_->CancelGoal();
                return BehaviorState::SUCCESS;
            }

            if (blackboard_ptr_->is_connected)
            {

                if (blackboard_ptr_->command == 0)
                {
                    goal_factory_ptr_->TakeCover(blackboard_ptr_->bullet_pose_);
                }
                if (blackboard_ptr_->command == 2)
                {
                    goal_factory_ptr_->GainBullet();
                }
            }
            else
            {
                goal_factory_ptr_->GainBullet();
            }

            // ROS_INFO("gain_bullets_action");
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
                // if (blackboard_ptr_->bullet_active_)
                // {
                // goal_factory_ptr_->Swing();
                // ROS_INFO("gain_bullets_action   and   swing");
                // }
                blackboard_ptr_->readytoshoot=1;
                blackboard_ptr_->gain_bullets_flag = true;
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
