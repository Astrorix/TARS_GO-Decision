/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DECISION_BEHAVIOR_TREE_H
#define ROBORTS_DECISION_BEHAVIOR_TREE_H

#include <chrono>

#include <ros/ros.h>
#include "behavior_node.h"

namespace roborts_decision
{
  /**
   * @brief Behavior tree class to initialize and execute the whole tree
   */
  class BehaviorTree
  {
  public:
    /**
     * @brief Constructor of BehaviorTree
     * @param root_node root node of the behavior tree
     * @param cycle_duration tick duration of the behavior tree (unit ms)
     */
    BehaviorTree(BehaviorNode::Ptr root_node, int cycle_duration) : root_node_(root_node),
                                                                    cycle_duration_(cycle_duration)
    {
      roborts_connect_pub_ = interact_nh.advertise<roborts_msgs::interact>("roborts_connect_pub", 30, this);
      roborts_qtmsg_pub = qtmsg_nh.advertise<roborts_msgs::QTMsg>("roborts_qtmsg_pub", 30, this);
      roborts_vision_pub = interact_nh.advertise<roborts_msgs::VisionPub>("roborts_vision_pub", 30, this);
      //roborts_qtmsg_pub = interact_nh.advertise<roborts_msgs::localization>("roborts_locolization_pub", 30, this);
    }
    /**
     * @brief Loop to tick the behavior tree
     */

    void localization_pub(auto black_ptr)
    {
      geometry_msgs::PoseStamped self_pose = black_ptr->GetRobotMapPose();
      roborts_msgs::localization msg;
      msg.selfpose_x = black_ptr->selfpose1_x;
      msg.selfpose_y = black_ptr->selfpose1_y;
      msg.selfpose_yaw = tf::getYaw(self_pose.pose.orientation);
      roborts_local_pub.publish(msg);
      ros::spinOnce();
      ROS_INFO("Localization Pub!");
    }

    void robort_connect(auto black_ptr)
    {
      roborts_msgs::interact msg;
      geometry_msgs::PoseStamped self_pose = black_ptr->GetRobotMapPose();
      msg.goalpose_x = black_ptr->goalpose_x;
      msg.goalpose_y = black_ptr->goalpose_y;
      msg.goalpose_yaw = black_ptr->goalpose_yaw;
      msg.matePose_x = self_pose.pose.position.x;
      msg.matePose_y = self_pose.pose.position.y;
      msg.command = black_ptr->command;
      msg.feedback = black_ptr->feedback;
      roborts_connect_pub_.publish(msg);
      ros::spinOnce();
      ROS_INFO("Connection Pub!");
    }

    void roborts_vision(auto black_ptr)
    {
      roborts_msgs::VisionPub msg;
      msg.command = black_ptr->command;
      msg.readytoshoot = black_ptr->readytoshoot;
      roborts_vision_pub.publish(msg);
      ROS_INFO("Vision Pub!");
    }

    void robort_qsmsg(auto black_ptr)
    {
      roborts_msgs::QTMsg msg;
      geometry_msgs::PoseStamped self_pose = black_ptr->GetRobotMapPose();
      int id = black_ptr->id_;
      if (id <= 2)
      {
        if (id == 1)
        {
          msg.redOnePose_x = self_pose.pose.position.x;
          msg.redOnePose_y = self_pose.pose.position.y;
          msg.redTwoPose_x = black_ptr->matePose_x;
          msg.redTwoPose_y = black_ptr->matePose_y;
        }
        else
        {
          msg.redOnePose_x = black_ptr->matePose_x;
          msg.redOnePose_y = black_ptr->matePose_y;
          msg.redTwoPose_x = self_pose.pose.position.x;
          msg.redTwoPose_y = self_pose.pose.position.y;
        }
        msg.blueOnePose_x = black_ptr->enemypose1_x;
        msg.blueOnePose_y = black_ptr->enemypose1_y;

        msg.blueTwoPose_x = black_ptr->enemypose2_x;
        msg.blueTwoPose_y = black_ptr->enemypose2_y;
      }
      else
      {
        if (id == 101)
        {
          msg.blueOnePose_x = self_pose.pose.position.x;
          msg.blueOnePose_y = self_pose.pose.position.y;
          msg.blueTwoPose_x = black_ptr->matePose_x;
          msg.blueTwoPose_y = black_ptr->matePose_y;
        }
        else
        {
          msg.blueTwoPose_x = self_pose.pose.position.x;
          msg.blueTwoPose_y = self_pose.pose.position.y;
          msg.blueOnePose_x = black_ptr->matePose_x;
          msg.blueOnePose_y = black_ptr->matePose_y;
        }
        msg.redOnePose_x = black_ptr->enemypose1_x;
        msg.redOnePose_y = black_ptr->enemypose1_y;

        msg.redTwoPose_x = black_ptr->enemypose2_x;
        msg.redTwoPose_y = black_ptr->enemypose2_y;
      }

      msg.huntCount = black_ptr->huntcount;
      msg.huntRadius = black_ptr->huntradius;
      msg.huntPose_x = black_ptr->huntpose_x;
      msg.huntPose_y = black_ptr->huntpose_y;

      if (black_ptr->is_master_)
      {
        msg.searchCount = black_ptr->c_search_count_;
      }
      else
      {
        msg.searchCount = black_ptr->e_search_count_;
      }
      msg.master = black_ptr->is_master_;
      roborts_qtmsg_pub.publish(msg);
      ros::spinOnce();
      ROS_INFO("QTMsg Pub!");
    }

    void Run()
    {
      // test
      std::string config_file_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
      auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
      unsigned int frame = 0;
      while (ros::ok())
      {
        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        ros::spinOnce();
        ROS_INFO("Frame : %d", frame);
        root_node_->Run();
        localization_pub(blackboard_ptr_);
        robort_connect(blackboard_ptr_);
        roborts_vision(blackboard_ptr_);
        robort_qsmsg(blackboard_ptr_);
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::milliseconds execution_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::chrono::milliseconds sleep_time = cycle_duration_ - execution_duration;

        if (sleep_time > std::chrono::microseconds(0))
        {

          std::this_thread::sleep_for(sleep_time);
          ROS_INFO("Excution Duration: %ld / %ld ms", cycle_duration_.count(), cycle_duration_.count());
        }
        else
        {

          ROS_WARN("The time tick once is %ld beyond the expected time %ld", execution_duration.count(), cycle_duration_.count());
        }
        ROS_INFO("----------------------------------");
        frame++;
      }
    }

    void Run(auto blackboard_ptr_)
    {
      unsigned int frame = 0;
      while (ros::ok())
      {

        std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
        ros::spinOnce();
        ROS_INFO("Frame : %d", frame);
        root_node_->Run();
        localization_pub(blackboard_ptr_);
        robort_connect(blackboard_ptr_);
        roborts_vision(blackboard_ptr_);
        robort_qsmsg(blackboard_ptr_);
        std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
        std::chrono::milliseconds execution_duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::chrono::milliseconds sleep_time = cycle_duration_ - execution_duration;

        if (sleep_time > std::chrono::microseconds(0))
        {

          std::this_thread::sleep_for(sleep_time);
          ROS_INFO("Excution Duration: %ld / %ld ms", cycle_duration_.count(), cycle_duration_.count());
        }
        else
        {

          ROS_WARN("The time tick once is %ld beyond the expected time %ld", execution_duration.count(), cycle_duration_.count());
        }
        ROS_INFO("----------------------------------");
        frame++;
      }
    }

  private:
    //! root node of the behavior tree
    BehaviorNode::Ptr root_node_;
    //! tick duration of the behavior tree (unit ms)
    std::chrono::milliseconds cycle_duration_;
    ros::Publisher roborts_connect_pub_;
    ros::Publisher roborts_qtmsg_pub;
    ros::Publisher roborts_vision_pub;
    ros::Publisher roborts_local_pub;
    ros::NodeHandle interact_nh;
    ros::NodeHandle qtmsg_nh;
  };

} // namespace roborts_decision

#endif // ROBORTS_DECISION_BEHAVIOR_TREE_H
