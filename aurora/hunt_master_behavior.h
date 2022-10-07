#ifndef ROBORTS_DECISION_commandHUNT_H
#define ROBORTS_DECISION_commandHUNT_H
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
      class commandHuntAction : public ActionNode
      {
      public:
            commandHuntAction(const Blackboard::Ptr &blackboard_ptr, GoalFactory::GoalFactoryPtr &goal_factory_ptr) : ActionNode::ActionNode("command_hunt", blackboard_ptr), goal_factory_ptr_(goal_factory_ptr)
            {
                  chassis_executor_ptr_ = blackboard_ptr_->chassis_executor_ptr_;
            }
            virtual ~commandHuntAction() = default;
            virtual void OnInitialize()
            {
                  blackboard_ptr_->command = 3;
                  blackboard_ptr_->matePose_x = 0;
                  blackboard_ptr_->matePose_y = 0;
                  ROS_INFO("Start Hunt action at %f time", ros::Time::now().toSec());
                  // ROS_INFO("%s %s", name_.c_str(), __FUNCTION__);
            }
            virtual BehaviorState Update()
            {
                  ROS_INFO("Update Variables:");
                  goal_factory_ptr_->C_Hunt();
                  ROS_INFO("---> C_hunt");
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

      private:
            GoalFactory::GoalFactoryPtr goal_factory_ptr_;
            std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
      };
}
#endif