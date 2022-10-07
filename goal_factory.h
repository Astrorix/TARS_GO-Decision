#ifndef ROBORT_GOAL_FACTORY_H
#define ROBORT_GOAL_FACTORY_H

#include <ros/ros.h>
#include <random>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include "blackboard/blackboard.h"
#include "blackboard/EscapeGrid.h"
#include "executor/chassis_executor.h"
#include "roborts_msgs/ChassisCtrAction.h"
#include "behavior_tree/behavior_node.h"
#include "behavior_tree/behavior_state.h"
#include <ctime>
#include "io/io.h"
#include "proto/decision.pb.h"

#include <vector>
#include <cmath>

namespace roborts_decision
{
  class GoalFactory
  {
  public:
    GoalFactory(const Blackboard::Ptr &blackboard_ptr) : blackboard_ptr_(blackboard_ptr) {}
    ~GoalFactory() = default;

    void BackToBootArea()
    {
      geometry_msgs::PoseStamped boot_pose;
      if (blackboard_ptr_->id_ > 10)
        boot_pose = blackboard_ptr_->boot_position_[1];
      else
        boot_pose = blackboard_ptr_->boot_position_[0];
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();

      auto dx = boot_pose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = boot_pose.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(boot_pose.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(boot_pose.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw = rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
      {
        blackboard_ptr_->chassis_executor_ptr_->Execute(boot_pose);
      }
    }

    void CancelChassis()
    {
      blackboard_ptr_->chassis_executor_ptr_->Cancel();
    }

    void CancelGimbal()
    {
      blackboard_ptr_->gimbal_executor_ptr_->Cancel();
    }

    void CancelGoal()
    {
      ROS_INFO("Cancel Goal!");
      blackboard_ptr_->cancel_flag_ = false;
      blackboard_ptr_->action_state_ = BehaviorState::IDLE;
    }

    void CancelScan()
    {
      if (blackboard_ptr_->is_scan_)
      {
        blackboard_ptr_->is_scan_ = false;
        ROS_INFO("cancel gimbal-scan action.");
      }
    }

    void Chase() // 追击 huntpose_x
    {
      unsigned xx, yy;
      geometry_msgs::PoseStamped self_pose = blackboard_ptr_->GetRobotMapPose();
      // geometry_msgs::PoseStamped static_pose;
      geometry_msgs::PoseStamped pose;
      int chasecount = 6;
      double chaseradius = 1.2;
      std::vector<geometry_msgs::PoseStamped> chase_pose;
      blackboard_ptr_->PowerMap();
      double self_distance = 1000000;

      int self_index = -1;
      // int slave_index = -1;
      self_index = -1;
      // slave_index = -1;
      for (int i = 0; i < chasecount; i++)
      {
        // pose.pose.position.x = blackboard_ptr_->enemy_pose1.pose.position.x + blackboard_ptr_->huntradius * std::cos(i * M_PI / blackboard_ptr_->huntcount);
        // pose.pose.position.y = blackboard_ptr_->enemy_pose1.pose.position.y + blackboard_ptr_->huntradius * std::sin(i * M_PI / blackboard_ptr_->huntcount);
        pose.pose.position.x = blackboard_ptr_->huntpose_x + chaseradius * std::cos(i * M_PI / chasecount);
        pose.pose.position.y = blackboard_ptr_->huntpose_y + chaseradius * std::sin(i * M_PI / chasecount);
        blackboard_ptr_->costmap_2d_->World2Map(pose.pose.position.x, pose.pose.position.y, xx, yy);
        ROS_INFO("---> Current X :%f,Current Y :%f", pose.pose.position.x, pose.pose.position.y);
        ROS_INFO("---> Current Cost :%d", (blackboard_ptr_->costmap_2d_->GetCost(xx, yy)));
        ROS_INFO("---> Chasepose_x :%f,Chasepose_y :%f", blackboard_ptr_->huntpose_x, blackboard_ptr_->huntpose_y);
        ROS_INFO(
            "---> Radius:%f  Count:%d", chaseradius, chasecount);
        if ((blackboard_ptr_->costmap_2d_->GetCost(xx, yy) < 253))
        {
          chase_pose.push_back(pose);
          ROS_INFO("---> Current Chase Size :%zd", chase_pose.size());
          if (blackboard_ptr_->GetDistance(self_pose, pose) < self_distance)
          {
            self_distance = blackboard_ptr_->GetDistance(self_pose, pose);
            self_index = i;
          }
        }
      }

      ROS_INFO("---> Total Chase Size :%zd", chase_pose.size());

      if (chase_pose.size())
      {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, self_index * M_PI / chasecount + M_PI);
        chase_pose[self_index].pose.orientation.x = quaternion.x();
        chase_pose[self_index].pose.orientation.y = quaternion.y();
        chase_pose[self_index].pose.orientation.z = quaternion.z();
        chase_pose[self_index].pose.orientation.w = quaternion.w();
        GoGoal(chase_pose[self_index]);
      }
      // geometry_msgs::PoseStamped enemy_pose;
      // enemy_pose = blackboard_ptr_->HandleEnemyPose();
      // geometry_msgs::PoseStamped robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      // float threa, self_threa;
      // threa = blackboard_ptr_->GetEnemyThrea();
      // self_threa = blackboard_ptr_->GetRobotThrea();
      // auto dx = enemy_pose.pose.position.x - robot_map_pose.pose.position.x;
      // auto dy = enemy_pose.pose.position.y - robot_map_pose.pose.position.y;
      // tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, threa);
      // enemy_pose.pose.orientation.x = quaternion.x();
      // enemy_pose.pose.orientation.y = quaternion.y();
      // enemy_pose.pose.orientation.z = quaternion.z();
      // enemy_pose.pose.orientation.w = quaternion.w();
      // if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2 && abs(self_threa - threa) <= 0.1)
      // {
      //   blackboard_ptr_->chassis_executor_ptr_->Cancel();
      // }
      // else
      // {
      //   if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2)
      //   {
      //     robot_map_pose.pose.orientation.x = quaternion.x();
      //     robot_map_pose.pose.orientation.y = quaternion.y();
      //     robot_map_pose.pose.orientation.z = quaternion.z();
      //     robot_map_pose.pose.orientation.w = quaternion.w();
      //     blackboard_ptr_->chassis_executor_ptr_->Execute(robot_map_pose);
      //   }
      //   else
      //   {
      //     blackboard_ptr_->chassis_executor_ptr_->Execute(enemy_pose);
      //   }
      // }
    }

    void C_Hunt()
    {
      unsigned xx, yy;
      geometry_msgs::PoseStamped master_pose = blackboard_ptr_->GetRobotMapPose();
      geometry_msgs::PoseStamped static_pose;
      geometry_msgs::PoseStamped pose;
      std::vector<geometry_msgs::PoseStamped> hunt_pose;
      blackboard_ptr_->PowerMap();
      // std::cout << "please input" << std::endl;
      // std::cin >> blackboard_ptr_->huntcount >> enemy_pose.pose.position.x >> enemy_pose.pose.position.y;
      // ROS_INFO("OK");
      double master_distance = 1000000;
      double slave_distance = 1000000;
      int master_index = -1;
      int slave_index = -1;
      master_index = -1;
      slave_index = -1;
      for (int i = 0; i < blackboard_ptr_->huntcount; i++)
      {
        // pose.pose.position.x = blackboard_ptr_->enemy_pose1.pose.position.x + blackboard_ptr_->huntradius * std::cos(i * M_PI / blackboard_ptr_->huntcount);
        // pose.pose.position.y = blackboard_ptr_->enemy_pose1.pose.position.y + blackboard_ptr_->huntradius * std::sin(i * M_PI / blackboard_ptr_->huntcount);
        pose.pose.position.x = blackboard_ptr_->huntpose_x + blackboard_ptr_->huntradius * std::cos(i * M_PI / blackboard_ptr_->huntcount);
        pose.pose.position.y = blackboard_ptr_->huntpose_y + blackboard_ptr_->huntradius * std::sin(i * M_PI / blackboard_ptr_->huntcount);
        blackboard_ptr_->costmap_2d_->World2Map(pose.pose.position.x, pose.pose.position.y, xx, yy);
        ROS_INFO("---> Current X:%f,Current Y:%f", pose.pose.position.x, pose.pose.position.y);
        ROS_INFO("---> Current Cost:%d", (blackboard_ptr_->costmap_2d_->GetCost(xx, yy)));
        ROS_INFO("---> Huntpose_x:%f,Huntpose_y:%f", blackboard_ptr_->huntpose_x, blackboard_ptr_->huntpose_y);
        ROS_INFO(
            "---> Radius:%f  Count:%d", blackboard_ptr_->huntradius, blackboard_ptr_->huntcount);
        if ((blackboard_ptr_->costmap_2d_->GetCost(xx, yy) < 253))
        {
          hunt_pose.push_back(pose);
          ROS_INFO("---> Current Hunt Size :%zd", hunt_pose.size());
          if (blackboard_ptr_->GetDistance(master_pose, pose) < master_distance)
          {
            master_distance = blackboard_ptr_->GetDistance(master_pose, pose);
            master_index = i;
          }
        }
      }
      ROS_INFO("---> Total Hunt Size:%zd", hunt_pose.size());
      for (int i = 0; i < hunt_pose.size(); i++)
      {
        if (std::sqrt(std::pow((blackboard_ptr_->matePose_x - pose.pose.position.x), 2) + std::pow((blackboard_ptr_->matePose_y - pose.pose.position.y), 2)) < slave_distance && i != master_index)
        {
          slave_distance = std::sqrt(std::pow((blackboard_ptr_->matePose_x - pose.pose.position.x), 2) + std::pow((blackboard_ptr_->matePose_y - pose.pose.position.y), 2));
          slave_index = i;
        }
      }
      if (hunt_pose.size())
      {
        if (hunt_pose.size() > 1)
        {
          ROS_INFO("---> S_index: %d", slave_index);
          if (slave_index != -1)
          {
            blackboard_ptr_->goalpose_x = hunt_pose[slave_index].pose.position.x;
            blackboard_ptr_->goalpose_y = hunt_pose[slave_index].pose.position.y;
          }
          blackboard_ptr_->goalpose_yaw = slave_index * M_PI / blackboard_ptr_->huntcount + M_PI;
        }
        try
        {

          ROS_INFO("---> M_index: %d", master_index);
          static_pose.header.frame_id = "map";
          if (master_index != -1)
          {
            static_pose.pose.position.x = hunt_pose[master_index].pose.position.x;
            static_pose.pose.position.y = hunt_pose[master_index].pose.position.y;
          }
          static_pose.pose.position.z = 0;
          tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, master_index * M_PI / blackboard_ptr_->huntcount + M_PI);
          static_pose.pose.orientation.x = quaternion.x();
          static_pose.pose.orientation.y = quaternion.y();
          static_pose.pose.orientation.z = quaternion.z();
          static_pose.pose.orientation.w = quaternion.w();
          auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
          auto dx = static_pose.pose.position.x - robot_map_pose.pose.position.x;
          auto dy = static_pose.pose.position.y - robot_map_pose.pose.position.y;

          auto static_yaw = tf::getYaw(static_pose.pose.orientation);
          auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

          tf::Quaternion rot1, rot2;
          tf::quaternionMsgToTF(static_pose.pose.orientation, rot1);
          tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
          auto d_yaw = rot1.angleShortestPath(rot2);

          if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
          {
            blackboard_ptr_->chassis_executor_ptr_->Execute(static_pose);
          }
        }
        catch (std::exception &e)
        {
          ROS_WARN("hunt execute error %s: ", e.what());
        }
      }
    }

    void Escape()
    {
    }

    void Escape(EscapeGrid &escapeGrid)
    {
      GoGoal(escapeGrid.GetEscapePose(blackboard_ptr_->GetRobotMapPose()));
      geometry_msgs::PoseStamped pose = escapeGrid.GetEscapePose(blackboard_ptr_->matePose);
      blackboard_ptr_->goalpose_x = pose.pose.position.x;
      blackboard_ptr_->goalpose_y = pose.pose.position.y;
    }

    void E_Hunt()
    {
      ROS_INFO("Update Variables:");
      ROS_INFO("--->Update Postion %f , %f ", blackboard_ptr_->goalpose_x, blackboard_ptr_->goalpose_y);
      geometry_msgs::PoseStamped hunt_pose;
      try
      {
        hunt_pose.header.frame_id = "map";
        hunt_pose.pose.position.x = blackboard_ptr_->goalpose_x;
        hunt_pose.pose.position.y = blackboard_ptr_->goalpose_y;
        hunt_pose.pose.position.z = 0;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, blackboard_ptr_->goalpose_yaw);
        hunt_pose.pose.orientation.x = quaternion.x();
        hunt_pose.pose.orientation.y = quaternion.y();
        hunt_pose.pose.orientation.z = quaternion.z();
        hunt_pose.pose.orientation.w = quaternion.w();
        auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
        auto dx = hunt_pose.pose.position.x - robot_map_pose.pose.position.x;
        auto dy = hunt_pose.pose.position.y - robot_map_pose.pose.position.y;

        auto hunt_yaw = tf::getYaw(hunt_pose.pose.orientation);
        auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(hunt_pose.pose.orientation, rot1);
        tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
        auto d_yaw = rot1.angleShortestPath(rot2);
        if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
        {
          blackboard_ptr_->chassis_executor_ptr_->Execute(hunt_pose);
        }
      }
      catch (std::exception &e)
      {
        ROS_WARN("hunt execute error %s: ", e.what());
      }
    }

    void Follow()
    {
      blackboard_ptr_->chassis_executor_ptr_->Execute(blackboard_ptr_->follow_pose_);
    }

    void GainBlood()
    {
      // geometry_msgs::PoseStamped buff_goal;
      // buff_goal.header.frame_id = "map";
      // buff_goal.pose.position.x = 0;
      // buff_goal.pose.position.y = 0;
      // buff_goal.pose.position.z = 0;
      // tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      // buff_goal.pose.orientation.x = quaternion.x();
      // buff_goal.pose.orientation.y = quaternion.y();
      // buff_goal.pose.orientation.z = quaternion.z();
      // buff_goal.pose.orientation.w = quaternion.w();
      // buff_goal = blackboard_ptr_->blood_pose_;
      // blackboard_ptr_->chassis_executor_ptr_->Execute(buff_goal);
      GoGoal(blackboard_ptr_->blood_pose_);
    }

    void GainBullet()
    {
      // blackboard_ptr_->chassis_executor_ptr_->Execute(blackboard_ptr_->bullet_pose_);
      GoGoal(blackboard_ptr_->bullet_pose_);
    }

    // 一旦出现异常会导致函数没有返回值  注意处理
    bool GoGoal(geometry_msgs::PoseStamped goal)
    {
      ROS_INFO("--->Update Postion %f , %f", goal.pose.position.x, goal.pose.position.y);
      try
      {
        geometry_msgs::PoseStamped current_pose = blackboard_ptr_->GetRobotMapPose();
        auto dxx = goal.pose.position.x - current_pose.pose.position.x;
        auto dyy = goal.pose.position.y - current_pose.pose.position.y;
        auto static_yaw = tf::getYaw(goal.pose.orientation);
        auto robot_yaw = tf::getYaw(current_pose.pose.orientation);
        tf::Quaternion rot1, rot2;
        tf::quaternionMsgToTF(goal.pose.orientation, rot1);
        tf::quaternionMsgToTF(current_pose.pose.orientation, rot2);
        auto d_yaw = rot1.angleShortestPath(rot2);
        ROS_INFO("---> Current X:%lf", current_pose.pose.position.x);
        ROS_INFO("---> Current Y:%lf", current_pose.pose.position.y);
        ROS_INFO("---> Distance:%f  , DeltaYaw:%f", std::sqrt(std::pow(dxx, 2) + std::pow(dyy, 2)), d_yaw);
        if (std::sqrt(std::pow(dxx, 2) + std::pow(dyy, 2)) > 0.2 || d_yaw > 0.5)
        {
          blackboard_ptr_->chassis_executor_ptr_->Execute(goal);
          return false;
        }
        else
          return true;
      }
      catch (std::exception &e)
      {
        ROS_WARN("search execute error %s: ", e.what());
      }
    }

    void GoGoal()
    {
      geometry_msgs::PoseStamped go_goal;
      go_goal.header.frame_id = "map";
      go_goal.pose.position.x = blackboard_ptr_->go_goal_x_;
      go_goal.pose.position.y = blackboard_ptr_->go_goal_y_;
      go_goal.pose.position.z = 0;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      go_goal.pose.orientation.x = quaternion.x();
      go_goal.pose.orientation.y = quaternion.y();
      go_goal.pose.orientation.z = quaternion.z();
      go_goal.pose.orientation.w = quaternion.w();
      blackboard_ptr_->chassis_executor_ptr_->Execute(go_goal);
    }

    void GoStaticPose()
    {
      geometry_msgs::PoseStamped static_pose;
      static_pose.header.frame_id = "map";
      static_pose.pose.position.x = 0;
      static_pose.pose.position.y = 0;
      static_pose.pose.position.z = 0;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
      static_pose.pose.orientation.x = quaternion.x();
      static_pose.pose.orientation.y = quaternion.y();
      static_pose.pose.orientation.z = quaternion.z();
      static_pose.pose.orientation.w = quaternion.w();
      static_pose = blackboard_ptr_->GetLeastStaticPose();
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto dx = static_pose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = static_pose.pose.position.y - robot_map_pose.pose.position.y;

      auto static_yaw = tf::getYaw(static_pose.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(static_pose.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw = rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
      {
        blackboard_ptr_->chassis_executor_ptr_->Execute(static_pose);
      }
    }

    void GoStopPose()
    {
      // geometry_msgs::PoseStamped pose;
      // pose = blackboard_ptr_->GetStopPose();
      // blackboard_ptr_->chassis_executor_ptr_->Execute(pose);
    }

    void RollingDefend()
    {
      geometry_msgs::Twist speed;
      std::random_device rd;
      std::mt19937_64 eng(rd());
      std::uniform_int_distribution<unsigned long long> distr(400, 600);
      double angle = (double)distr(eng) / 100;
      speed.linear.x = 0;
      speed.linear.y = 0;
      speed.linear.z = 0;
      speed.angular.x = 0;
      speed.angular.y = 0;
      speed.angular.z = angle;
      blackboard_ptr_->chassis_executor_ptr_->Execute(speed);
    }

    // void C_SearchGoal()
    // {
    //   geometry_msgs::PoseStamped path;
    //   int count = blackboard_ptr_->c_search_count_;
    //   ROS_INFO("search count:%d", count);
    //   int size = blackboard_ptr_->search_points_size_;
    //   ROS_INFO("search size:%d", size);
    //   path = blackboard_ptr_->c_search_point[count];
    //   geometry_msgs::PoseStamped current_pose = blackboard_ptr_->GetRobotMapPose();
    //   auto dx = current_pose.pose.position.x - path.pose.position.x;
    //   auto dy = current_pose.pose.position.y - path.pose.position.y;
    //   double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    //   if (s_distance < 0.8)
    //   {
    //     blackboard_ptr_->c_search_count_ = (count + 1) % size;
    //   }
    //   count = blackboard_ptr_->c_search_count_;
    //   path = blackboard_ptr_->c_search_point[count];
    //   path.pose.orientation.x = current_pose.pose.orientation.x;
    //   path.pose.orientation.y = current_pose.pose.orientation.y;
    //   path.pose.orientation.z = current_pose.pose.orientation.z;
    //   path.pose.orientation.w = current_pose.pose.orientation.w;
    //   try
    //   {
    //     blackboard_ptr_->chassis_executor_ptr_->Execute(path);
    //   }
    //   catch (std::exception &e)
    //   {
    //     ROS_WARN("search execute error %s: ", e.what());
    //   }
    // }

    // void E_SearchGoal()
    // {
    //   geometry_msgs::PoseStamped path;
    //   int count = blackboard_ptr_->e_search_count_;
    //   ROS_INFO("search count:%d", count);
    //   int size = blackboard_ptr_->search_points_size_;
    //   ROS_INFO("search size:%d", size);
    //   path = blackboard_ptr_->e_search_point[count];
    //   geometry_msgs::PoseStamped current_pose = blackboard_ptr_->GetRobotMapPose();
    //   auto dx = current_pose.pose.position.x - path.pose.position.x;
    //   auto dy = current_pose.pose.position.y - path.pose.position.y;
    //   double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    //   if (s_distance < 0.8)
    //   {
    //     blackboard_ptr_->e_search_count_ = (count + 1) % size;
    //   }
    //   count = blackboard_ptr_->e_search_count_;
    //   // ROS_INFO("e_search_count_ %d", blackboard_ptr_->e_search_count_);
    //   path = blackboard_ptr_->e_search_point[count];
    //   path.pose.orientation.x = current_pose.pose.orientation.x;
    //   path.pose.orientation.y = current_pose.pose.orientation.y;
    //   path.pose.orientation.z = current_pose.pose.orientation.z;
    //   path.pose.orientation.w = current_pose.pose.orientation.w;
    //   try
    //   {
    //     blackboard_ptr_->chassis_executor_ptr_->Execute(path);
    //   }
    //   catch (std::exception &e)
    //   {
    //     ROS_WARN("search execute error %s: ", e.what());
    //   }
    // }

    void SearchGoal()
    {
      int count = 0;
      geometry_msgs::PoseStamped path;
      int size = blackboard_ptr_->search_points_size_;
      geometry_msgs::PoseStamped current_pose = blackboard_ptr_->GetRobotMapPose();
      if (blackboard_ptr_->is_connected && blackboard_ptr_->is_master_) // 主车
      {
        // count = blackboard_ptr_->c_search_count_;
        count = blackboard_ptr_->search_count_;
        path = blackboard_ptr_->c_search_point[count];
      }
      else if (blackboard_ptr_->is_connected && !blackboard_ptr_->is_master_) //副车
      {
        // count = blackboard_ptr_->e_search_count_;
        count = blackboard_ptr_->search_count_;
        path = blackboard_ptr_->e_search_point[count];
      }
      else // 单车
      {
        count = blackboard_ptr_->search_count_;
        path = blackboard_ptr_->search_point[count];
      }
      ROS_INFO("Search Count: %d ", blackboard_ptr_->search_count_);
      // auto boot_yaw = tf::getYaw(boot_pose.pose.orientation);
      // auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);
      // tf::Quaternion rot1, rot2;
      // tf::quaternionMsgToTF(boot_pose.pose.orientation, rot1);
      // tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      // auto d_yaw = rot1.angleShortestPath(rot2);
      // if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5)
      // {
      //   blackboard_ptr_->chassis_executor_ptr_->Execute(boot_pose);
      // }
      // auto dx = current_pose.pose.position.x - path.pose.position.x;
      // auto dy = current_pose.pose.position.y - path.pose.position.y;
      // auto dz = current_pose.pose.orientation.z - path.pose.orientation.z;
      // auto dw = current_pose.pose.orientation.w - path.pose.orientation.w;
      // double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      // if (s_distance < 0.8 && dw < 0.5)
      // {
      //   blackboard_ptr_->search_count_ = (count + 1) % size;
      // }
      // try
      // {
      //   auto dxx = blackboard_ptr_->search_point[count].pose.position.x - current_pose.pose.position.x;
      //   auto dyy = blackboard_ptr_->search_point[count].pose.position.y - current_pose.pose.position.y;
      //   auto static_yaw = tf::getYaw(blackboard_ptr_->search_point[count].pose.orientation);
      //   auto robot_yaw = tf::getYaw(current_pose.pose.orientation);
      //   tf::Quaternion rot1, rot2;
      //   tf::quaternionMsgToTF(blackboard_ptr_->search_point[count].pose.orientation, rot1);
      //   tf::quaternionMsgToTF(current_pose.pose.orientation, rot2);
      //   auto d_yaw = rot1.angleShortestPath(rot2);
      //   ROS_INFO("CURRENT x:%lf", current_pose.pose.position.x);
      //   ROS_INFO("CURRENT y:%lf", current_pose.pose.position.y);
      //   ROS_INFO("S:%f  , D_yaw:%f", std::sqrt(std::pow(dxx, 2) + std::pow(dyy, 2)), d_yaw);
      //   if (std::sqrt(std::pow(dxx, 2) + std::pow(dyy, 2)) > 0.2 || d_yaw > 0.5)
      //   {
      //     blackboard_ptr_->chassis_executor_ptr_->Execute(blackboard_ptr_->search_point[count]);
      //   }
      //   else
      //     blackboard_ptr_->search_count_ = (count + 1) % size;
      // }
      // catch (std::exception &e)
      // {
      //   ROS_WARN("search execute error %s: ", e.what());
      // }
      if (GoGoal(path))
        blackboard_ptr_->search_count_ = (count + 1) % size;
    }

    void Swing()
    {
      geometry_msgs::Twist speed;
      ros::Rate rate(3);
      for (int i = 0; i <= 16; i++)
      {
        switch (i % 8)
        {
        case 0:
          speed.linear.x = 0.5;
          speed.linear.y = 0;
          break;
        case 1:
          speed.linear.x = -0.5;
          speed.linear.y = 0;
          break;
        case 2:
          speed.linear.x = -0.5;
          speed.linear.y = 0;
          break;
        case 3:
          speed.linear.x = 0.5;
          speed.linear.y = 0;
          break;
        case 4:
          speed.linear.x = 0;
          speed.linear.y = 0.5;
          break;
        case 5:
          speed.linear.x = 0;
          speed.linear.y = -0.5;
          break;
        case 6:
          speed.linear.x = 0;
          speed.linear.y = -0.5;
          break;
        case 7:
          speed.linear.x = 0;
          speed.linear.y = 0.5;
          break;
        }
        speed.linear.z = 0;
        speed.angular.x = 0;
        speed.angular.y = 0;
        speed.angular.z = 0;
        blackboard_ptr_->chassis_executor_ptr_->Execute(speed);
        rate.sleep();
      }
    }

    void SwingDefend()
    {
      Client client("chassisdefend", true);
      ROS_INFO("WAITING FOR ACTION SERVER TO START !");
      client.waitForServer();
      ROS_INFO("ACTION SERVER START !");
      roborts_msgs::ChassisCtrGoal flag;
      flag.model = 1;
      client.sendGoal(flag);
      ros::spin();
    }

    void TakeCover(geometry_msgs::PoseStamped takeCoverPose)
    {
      geometry_msgs::PoseStamped pose;
      std::vector<geometry_msgs::PoseStamped> cover_pose;
      double cover_distance = 1000000;
      int cover_index = -1;
      int cover_count = 2;
      double cover_radius = 1.2;
      unsigned xx, yy;
      while (!cover_pose.size())
      {
        for (int i = 0; i < cover_count; i++)
        {
          // pose.pose.position.x = blackboard_ptr_->enemy_pose1.pose.position.x + blackboard_ptr_->huntradius * std::cos(i * M_PI / blackboard_ptr_->huntcount);
          // pose.pose.position.y = blackboard_ptr_->enemy_pose1.pose.position.y + blackboard_ptr_->huntradius * std::sin(i * M_PI / blackboard_ptr_->huntcount);
          pose.pose.position.x = takeCoverPose.pose.position.x + cover_radius * std::cos(i * M_PI / cover_count);
          pose.pose.position.y = takeCoverPose.pose.position.y + cover_radius * std::sin(i * M_PI / cover_count);
          blackboard_ptr_->costmap_2d_->World2Map(pose.pose.position.x, pose.pose.position.y, xx, yy);
          ROS_INFO("---> Current X:%f,Y:%f", takeCoverPose.pose.position.x, takeCoverPose.pose.position.y);
          ROS_INFO("---> Current Cost:%d", (blackboard_ptr_->costmap_2d_->GetCost(xx, yy)));
          ROS_INFO("---> Coverpose_x:%f,Coverpose_y:%f", pose.pose.position.x, pose.pose.position.y);
          ROS_INFO("---> Radius:%f  Count:%d", cover_radius, cover_count);
          if ((blackboard_ptr_->costmap_2d_->GetCost(xx, yy) < 253))
          {
            cover_pose.push_back(pose);
            ROS_INFO("---> Current Size :%zd", cover_pose.size());
            if (blackboard_ptr_->GetDistance(pose, blackboard_ptr_->enemyPose1) + blackboard_ptr_->GetDistance(pose, blackboard_ptr_->enemyPose2) < cover_distance)
            {
              cover_distance = blackboard_ptr_->GetDistance(pose, blackboard_ptr_->enemyPose1) + blackboard_ptr_->GetDistance(pose, blackboard_ptr_->enemyPose2);
              cover_index = i;
            }
          }
        }
        if (!cover_pose.size())
          cover_count++;
      }
      float cover_yaw = TowardsEnemy();
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, cover_yaw);
      cover_pose[cover_index].pose.orientation.x = quaternion.x();
      cover_pose[cover_index].pose.orientation.y = quaternion.y();
      cover_pose[cover_index].pose.orientation.z = quaternion.z();
      cover_pose[cover_index].pose.orientation.w = quaternion.w();
      GoGoal(cover_pose[cover_index]);
    }

    //功能函数 配合所有跑位 使底盘朝向敌人
    float TowardsEnemy()
    {
      geometry_msgs::PoseStamped pose = blackboard_ptr_->GetRobotMapPose();
      float enemyDistance1 = blackboard_ptr_->GetDistance(blackboard_ptr_->enemyPose1, pose);
      float enemyDistance2 = blackboard_ptr_->GetDistance(blackboard_ptr_->enemyPose2, pose);
      float tan1 = (blackboard_ptr_->enemypose1_y - pose.pose.position.y) / (blackboard_ptr_->enemypose1_x - pose.pose.position.x);
      float tan2 = (blackboard_ptr_->enemypose2_y - pose.pose.position.y) / (blackboard_ptr_->enemypose2_x - pose.pose.position.x);
      int enemyOJ1 = blackboard_ptr_->ObstacleJudge(pose.pose.position.x, pose.pose.position.y, enemyDistance1, tan1);
      int enemyOJ2 = blackboard_ptr_->ObstacleJudge(pose.pose.position.x, pose.pose.position.y, enemyDistance2, tan2);
      //在两个都有障碍物的时候 以距离优先 所以enemyOJ前乘以一个大于 8（假设的最远距离） 的数字就行
      if (enemyDistance1 + 200 * enemyOJ1 < enemyDistance2 + 200 * enemyOJ2)
      {
        float dy = blackboard_ptr_->enemyPose1.pose.position.y - pose.pose.position.y;
        float dx = blackboard_ptr_->enemyPose1.pose.position.x - pose.pose.position.x;
        float sin = (dy) / enemyDistance1;
        float yaw = std::asin(sin);
        // -pi/2<yaw<pi/2
        // (0-pi/2) yaw    (pi/2-pi) pi-yaw
        // (pi-3/2pi) pi-yaw (-3/2pi-0) yaw
        if (dx < 0)
          return 3.14 - yaw;
        else
          return yaw;
      }
      else
      {
        float dy = blackboard_ptr_->enemyPose2.pose.position.y - pose.pose.position.y;
        float dx = blackboard_ptr_->enemyPose2.pose.position.x - pose.pose.position.x;
        float sin = (dy) / enemyDistance2;
        float yaw = std::asin(sin);
        if (dx < 0)
          return 3.14 - yaw;
        else
          return yaw;
      }
    }

    void TurnToDetectedDirection()
    {
      double d_yaw = 3.14;
      try
      {
        geometry_msgs::Twist speed;
        speed.linear.x = 0;
        speed.linear.y = 0;
        speed.linear.z = 0;
        speed.angular.x = 0;
        speed.angular.y = 0;
        speed.angular.z = 4.5;
        blackboard_ptr_->chassis_executor_ptr_->Execute(speed);
      }
      catch (std::exception &e)
      {
        // OS_ERROR("ERROR: %S", e.what());
      }
      blackboard_ptr_->damage_source_ = -1;
    }

    typedef std::shared_ptr<GoalFactory> GoalFactoryPtr;

  private:
    Blackboard::Ptr blackboard_ptr_;
    typedef actionlib::SimpleActionClient<roborts_msgs::ChassisCtrAction> Client;
  };
}

#endif