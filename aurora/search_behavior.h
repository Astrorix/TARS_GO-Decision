#ifndef ROBORTS_DECISION_SEARCHACTION_H
#define ROBORTS_DECISION_SEARCHACTION_H
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
    class SearchAction : public ActionNode
    {
    public:
        SearchAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr)
            : ActionNode::ActionNode("search_action", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
        {
            chassis_executor_ptr_ = blackboard_ptr_->chassis_executor_ptr_;
        }
        virtual ~SearchAction() = default;

    private:
        std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
        ros::Time start_time_;
        GoalFactory::GoalFactoryPtr goal_factory_ptr_;
        virtual void OnInitialize()
        {
            blackboard_ptr_->command = 4;
            blackboard_ptr_->readytoshoot =1;
            geometry_msgs::PoseStamped current_pose = blackboard_ptr_->GetRobotMapPose();
            double distance = 1000000;
            int best_index = -1;
            blackboard_ptr_->is_search_ = true;
            for (int i = 0; i < blackboard_ptr_->search_points_size_; i++)
            {
                geometry_msgs::PoseStamped search_pose;
                if (blackboard_ptr_->is_connected && blackboard_ptr_->is_master_)
                {
                    search_pose = blackboard_ptr_->c_search_point[i];
                }
                else if (blackboard_ptr_->is_connected && !blackboard_ptr_->is_master_)
                {
                    search_pose = blackboard_ptr_->e_search_point[i];
                }
                else
                {
                    search_pose = blackboard_ptr_->search_point[i];
                }
                auto dx = current_pose.pose.position.x - search_pose.pose.position.x;
                auto dy = current_pose.pose.position.y - search_pose.pose.position.y;
                if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) < distance)
                {
                    distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
                    best_index = i;
                }
                blackboard_ptr_->search_count_ = best_index;
            }
            ROS_INFO("Start Search action at %f time", ros::Time::now().toSec());
        }

        virtual BehaviorState Update()
        {
            blackboard_ptr_->readytoshoot=1;
            ros::Time now_time = ros::Time::now();
            ROS_INFO("Update Variables:");
            ROS_INFO("--->search time:%f", (now_time - start_time_).toSec());
            if ((now_time - start_time_).toSec() > 6)
            {
                start_time_ = ros::Time::now();
                int size = blackboard_ptr_->search_points_size_;
                int count = blackboard_ptr_->search_count_;
                blackboard_ptr_->search_count_ = (count + 1) % size;
                // ROS_INFO("--->search count:%d", blackboard_ptr_->search_count_);
            }
            goal_factory_ptr_->SearchGoal();
            // ROS_INFO("search");
            if (!blackboard_ptr_->is_master_) //（失败0 完成1 运行2）
                blackboard_ptr_->feedback = 2;
            blackboard_ptr_->action_state_ = chassis_executor_ptr_->Update();
            return blackboard_ptr_->action_state_;
        }
        virtual void OnTerminate(BehaviorState state)
        {
            blackboard_ptr_->readytoshoot=1;
            switch (state)
            {
            case BehaviorState::IDLE:
                goal_factory_ptr_->CancelGoal();
                ROS_INFO("%s %s IDLE!", name_.c_str(), __FUNCTION__);
                break;
            case BehaviorState::SUCCESS:
                if (!blackboard_ptr_->is_master_)
                    blackboard_ptr_->feedback = 1;
                blackboard_ptr_->is_search_ = false;
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
} // namespace roborts_decision

#endif