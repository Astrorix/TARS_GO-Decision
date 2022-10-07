#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "costmap/costmap_interface.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/GameRobotHP.h"
#include "roborts_msgs/GameRobotBullet.h"
#include "roborts_msgs/ShootInfo.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/PunishInfo.h"
#include "roborts_msgs/RobotInfo.h"
#include "roborts_msgs/TreeStatus.h"
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/GameZone.h"
#include "roborts_msgs/PyArmorInfo.h"
#include "roborts_msgs/interact.h"
#include "roborts_msgs/GoGoal.h"
#include "roborts_msgs/SentryPostMessage.h"
#include "roborts_msgs/QTMsg.h"
#include "roborts_msgs/Vision.h"
#include "roborts_msgs/VisionPub.h"
#include "roborts_msgs/LurkStatus.h"
#include "roborts_msgs/localization.h"
#include "../proto/decision.pb.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
namespace roborts_decision
{
  class Blackboard
  {
  public:
    Blackboard(const std::string &proto_file_path) : proto_file_path_(proto_file_path)
    {
      tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
      chassis_executor_ptr_ = std::make_shared<ChassisExecutor>();

      std::string map_path = ros::package::getPath("roborts_costmap") +
                             "/config/costmap_parameter_config_for_decision.prototxt";
      costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                               map_path);
      charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
      costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

      ros::NodeHandle referee_nh; //共十个
      game_status_sub_ = referee_nh.subscribe<roborts_msgs::GameStatus>("game_status", 30, &Blackboard::GameStatusCallback, this);
      game_result_sub_ = referee_nh.subscribe<roborts_msgs::GameResult>("game_result", 30, &Blackboard::GameResultCallback, this);
      robot_hp_sub_ = referee_nh.subscribe<roborts_msgs::GameRobotHP>("game_robot_hp", 30, &Blackboard::RobotHPCallback, this);
      robot_bullet_sub_ = referee_nh.subscribe<roborts_msgs::GameRobotBullet>("game_robot_bullet", 30, &Blackboard::RobotBulletCallback, this);
      game_zone_array_status_sub_ = referee_nh.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 30, &Blackboard::GameZoneArrayCallback, this);
      ros_robot_status_sub_ = referee_nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 30, &Blackboard::RobotStatusCallback, this);
      robot_heat_sub_ = referee_nh.subscribe<roborts_msgs::RobotHeat>("robot_heat", 30, &Blackboard::RobotHeatCallback, this);
      robot_damage_sub_ = referee_nh.subscribe<roborts_msgs::RobotDamage>("robot_damage", 30, &Blackboard::RobotDamageCallback, this);
      robot_shoot_sub_ = referee_nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 30, &Blackboard::RobotShootCallback, this);
      robot_lurk_sub_ = referee_nh.subscribe<roborts_msgs::LurkStatus>("lurk_status", 30, &Blackboard::LurkStatusCallback, this);

      ros::NodeHandle interact_nh; //共两个
      roborts_connect_sub_ = interact_nh.subscribe<roborts_msgs::interact>("roborts_connect_sub", 30, &Blackboard::RobortsConnectCallback, this);
      sentry_posts_sub_ = interact_nh.subscribe<roborts_msgs::SentryPostMessage>("sentry_posts_sub", 30, &Blackboard::SentryPostsCallback, this);

      ros::NodeHandle vision_nh; //共一个
      vision_sub_ = vision_nh.subscribe<roborts_msgs::Vision>("vision_talker", 30, &Blackboard::VisionCallback, this);

      LoadParam(proto_file_path);
    }
    ~Blackboard() = default;

    void GameResultCallback(const roborts_msgs::GameResult::ConstPtr &game_result)
    {
      game_result_ = static_cast<unsigned int>(game_result->result);
    }

    void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr &game_info)
    {
      game_status_ = static_cast<int>(game_info->game_status);
      remaining_time_ = static_cast<unsigned int>(game_info->remaining_time);
      if (remaining_time_ == 121 || remaining_time_ == 61)
      {
        LoadParam(proto_file_path_);
        gain_bullets_flag = false;
        gain_blood_flag = false;
        has_chase_enemy_ = false;
        has_followed_ = false;
      }
    }

    void GameZoneArrayCallback(const roborts_msgs::GameZoneArray::ConstPtr &game_zone_array)
    {
      for (int i = 0; i <= 5; i++)
        Zone_status[i] = static_cast<int>(game_zone_array->zone[i].type);
      for (int i = 0; i <= 5; i++)
        Zone_active[i] = static_cast<bool>(game_zone_array->zone[i].active);
      if (id_ <= 2)
      {
        for (int i = 0; i <= 5; i++)
        {
          if (Zone_status[i] == 1)
          {

            blood_pose_ = GetBuffPoints(i);
            blood_active_ = Zone_active[i];
          }
          if (Zone_status[i] == 2)
          {

            bullet_pose_ = GetBuffPoints(i);
            ROS_INFO("bulletPoint: %.3lf %.3lf", bullet_pose_.pose.position.x, bullet_pose_.pose.position.y);
            bullet_active_ = Zone_active[i];
          }
        }
      }
      else
      {
        for (int i = 0; i <= 5; i++)
        {
          if (Zone_status[i] == 3)
          {

            blood_pose_ = GetBuffPoints(i);
            blood_active_ = Zone_active[i];
          }
          if (Zone_status[i] == 4)
          {
            bullet_pose_ = GetBuffPoints(i);
            ROS_INFO("bulletPoint: %.3lf %.3lf", bullet_pose_.pose.position.x, bullet_pose_.pose.position.y);
            bullet_active_ = Zone_active[i];
          }
        }
      }
    }

    void LurkStatusCallback(const roborts_msgs::LurkStatus::ConstPtr &lurk_status)
    {
      lurk_mode = static_cast<unsigned int>(lurk_status->lurk_mode);
    }

    void RobotBulletCallback(const roborts_msgs::GameRobotBullet::ConstPtr &robot_bullet)
    {
      int robot_id_ = id_;
      red1_bullet = static_cast<int>(robot_bullet->red1);
      red2_bullet = static_cast<int>(robot_bullet->red2);
      blue1_bullet = static_cast<int>(robot_bullet->blue1);
      blue2_bullet = static_cast<int>(robot_bullet->blue2);
      if (1 == robot_id_)
      {
        remain_bullet_ = red1_bullet;
        teammate_bullet_ = red2_bullet;
        enemy1_bullet_ = blue1_bullet;
        enemy2_bullet_ = blue2_bullet;
      }
      if (2 == robot_id_)
      {
        remain_bullet_ = red2_bullet;
        teammate_bullet_ = red1_bullet;
        enemy1_bullet_ = blue1_bullet;
        enemy2_bullet_ = blue2_bullet;
      }
      if (101 == robot_id_)
      {
        remain_bullet_ = blue1_bullet;
        teammate_bullet_ = blue2_bullet;
        enemy1_bullet_ = red1_bullet;
        enemy2_bullet_ = red2_bullet;
      }
      if (102 == robot_id_)
      {
        remain_bullet_ = blue2_bullet;
        teammate_bullet_ = blue1_bullet;
        enemy1_bullet_ = red1_bullet;
        enemy2_bullet_ = red2_bullet;
      }
    }
    void RobortsConnectCallback(const roborts_msgs::interact::ConstPtr &robot_connect)
    {
      if (is_master_)
      {
        matePose_x = robot_connect->matePose_x;
        matePose_y = robot_connect->matePose_y;
      }
      else
      {
        goalpose_x = robot_connect->goalpose_x;
        goalpose_y = robot_connect->goalpose_y;
        goalpose_yaw = robot_connect->goalpose_yaw;
        command = robot_connect->command;
        matePose_x = robot_connect->matePose_x;
        matePose_y = robot_connect->matePose_y;
      }
    }
    void RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr &robot_damage)
    {
      damage_type_ = static_cast<int>(robot_damage->damage_type);
      damage_source_ = static_cast<int>(robot_damage->damage_source);
    }

    void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr &robot_heat)
    {
      robot_heat_ = static_cast<int>(robot_heat->shooter_heat);
    }

    void RobotHPCallback(const roborts_msgs::GameRobotHP::ConstPtr &robot_hp)
    { // 101、102蓝方  1、2红方  1为主车
      ros::Time now_time_ = ros::Time::now();
      int robot_id_ = id_;
      red1_hp = static_cast<int>(robot_hp->red1);
      red2_hp = static_cast<int>(robot_hp->red2);
      blue1_hp = static_cast<int>(robot_hp->blue1);
      blue2_hp = static_cast<int>(robot_hp->blue2);
      if (1 == robot_id_)
      {
        self_hp_delta_ = remain_hp_ - red1_hp;
        remain_hp_ = red1_hp;
        mate_hp_delta_ = teammate_hp_ - red2_hp;
        teammate_hp_ = red2_hp;
        enemy1_hp_delta_ = enemy1_hp_ - blue1_hp;
        enemy1_hp_ = blue1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - blue2_hp;
        enemy2_hp_ = blue2_hp;
      }
      if (2 == robot_id_)
      {
        self_hp_delta_ = remain_hp_ - red2_hp;
        remain_hp_ = red2_hp;
        mate_hp_delta_ = teammate_hp_ - red1_hp;
        teammate_hp_ = red1_hp;
        enemy1_hp_delta_ = enemy1_hp_ - blue1_hp;
        enemy1_hp_ = blue1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - blue2_hp;
        enemy2_hp_ = blue2_hp;
      }
      if (101 == robot_id_)
      {
        self_hp_delta_ = remain_hp_ - blue1_hp;
        remain_hp_ = blue1_hp;
        mate_hp_delta_ = teammate_hp_ - blue2_hp;
        teammate_hp_ = blue2_hp;
        enemy1_hp_delta_ = enemy1_hp_ - red1_hp;
        enemy1_hp_ = red1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - red2_hp;
        enemy2_hp_ = red2_hp;
      }
      if (102 == robot_id_)
      {
        self_hp_delta_ = remain_hp_ - blue2_hp;
        remain_hp_ = blue2_hp;
        mate_hp_delta_ = teammate_hp_ - blue1_hp;
        teammate_hp_ = blue1_hp;
        enemy1_hp_delta_ = enemy1_hp_ - red1_hp;
        enemy1_hp_ = red1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - red2_hp;
        enemy2_hp_ = red2_hp;
      }
      time_delta_ = (now_time_ - last_time_).toSec();
      last_time_ = now_time_;
      self_blood_loss_per_second_[hp_count] = self_hp_delta_ / time_delta_;
      mate_blood_loss_per_second_[hp_count] = mate_hp_delta_ / time_delta_;
      enemy1_blood_loss_per_second_[hp_count] = enemy1_hp_delta_ / time_delta_;
      enemy2_blood_loss_per_second_[hp_count] = enemy2_hp_delta_ / time_delta_;
      hp_count = (hp_count + 1) % 2;
    }

    void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr &robot_shoot)
    {
      robot_shoot_frequency_ = static_cast<int>(robot_shoot->frequency);
      robot_shoot_speed_ = static_cast<double>(robot_shoot->speed);
    }

    void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &robot_status)
    {
      id_ = static_cast<int>(robot_status->id);
      if (id_ == 1 || id_ == 101)
      {
        is_master_ = true;
      }
      else
      {
        is_master_ = false;
      }
      gimbal_punished_ = !(static_cast<bool>(robot_status->gimbal_enable));
      chassis_punished_ = !(static_cast<bool>(robot_status->chassis_enable));
      if (gimbal_punished_ || chassis_punished_)
      {
        if_punished_ = true;
      }
    }

    void SentryPostsCallback(const roborts_msgs::SentryPostMessage::ConstPtr &sentry_post_message)
    {
      // 101、102蓝方  1、2红方  1为主车
      //  顺序红1 红2 蓝1 蓝2
      sentry_connected = sentry_post_message->connectionstate;
      red1 = sentry_post_message->self1;
      red2 = sentry_post_message->self2;
      blue1 = sentry_post_message->enemy1;
      blue2 = sentry_post_message->enemy2;
      sentry_lost = true;
      if (lurk_mode == 0)
      {
        if (id_ > 2) //我方是蓝
        {
          if (red1)
          {
            sentry_lost = false;
            enemypose1_x = sentry_post_message->selfpose1_x;
            enemypose1_y = sentry_post_message->selfpose1_y;
            enemyPose1.pose.position.x = enemypose1_x;
            enemyPose1.pose.position.y = enemypose1_y;
          }
          if (red2)
          {
            sentry_lost = false;
            enemypose2_x = sentry_post_message->selfpose2_x;
            enemypose2_y = sentry_post_message->selfpose2_y;
            enemyPose2.pose.position.x = enemypose2_x;
            enemyPose2.pose.position.y = enemypose2_y;
          }
        }
        else //我方是红
        {
          if (blue1)
          {
            sentry_lost = false;
            enemypose1_x = sentry_post_message->enemypose1_x;
            enemypose1_y = sentry_post_message->enemypose1_y;
            enemyPose1.pose.position.x = enemypose1_x;
            enemyPose1.pose.position.y = enemypose1_y;
          }
          if (blue2)
          {
            sentry_lost = false;
            enemypose2_x = sentry_post_message->enemypose2_x;
            enemypose2_y = sentry_post_message->enemypose2_y;
            enemyPose2.pose.position.x = enemypose2_x;
            enemyPose2.pose.position.y = enemypose2_y;
          }
        }
      }
      else
      {

        std::vector<double> x;
        std::vector<double> y;

        if (red1)
        {
          x.push_back(sentry_post_message->selfpose1_x);
          y.push_back(sentry_post_message->selfpose1_y);
        }
        if (red2)
        {
          x.push_back(sentry_post_message->selfpose2_x);
          y.push_back(sentry_post_message->selfpose2_y);
        }
        if (blue1)
        {
          x.push_back(sentry_post_message->enemypose1_x);
          y.push_back(sentry_post_message->enemypose1_y);
        }
        if (blue2)
        {
          x.push_back(sentry_post_message->enemypose2_x);
          y.push_back(sentry_post_message->enemypose2_y);
        }

        geometry_msgs::PoseStamped selfPose = GetRobotMapPose();
        float x_mirror = 8.08 - selfPose.pose.position.x;
        float y_mirror = 4.48 - selfPose.pose.position.y;
        float matex_mirror = 8.08 - matePose_x;
        float matey_mirror = 4.48 - matePose_y;
        int selfPose_index = -1;
        int matePose_index = -1;

        for (int i = 0; i < x.size(); i++)
        {
          //找到自己的坐标(镜像坐标 + 实时坐标)并交付定位用以消除重定位
          if ((std::sqrt(std::pow(x[i] - selfPose.pose.position.x, 2) + std::pow(y[i] - selfPose.pose.position.y, 2)) < 0.3) ||
              (std::sqrt(std::pow(x[i] - x_mirror, 2) + std::pow(y[i] - y_mirror, 2)) < 0.3))
          {
            selfPose_index = i;
          }
          //找到另一辆车的坐标
          else if ((std::sqrt(std::pow(x[i] - matePose_x, 2) + std::pow(y[i] - matePose_y, 2)) < 0.3) ||
                   (std::sqrt(std::pow(x[i] - matex_mirror, 2) + std::pow(y[i] - matey_mirror, 2)) < 0.3))
          {
            matePose_index = i;
          }
        }
        if (selfPose_index != -1)
        {
          selfpose1_x = x[selfPose_index];
          selfpose1_y = y[selfPose_index];
        }
        if (matePose_index != -1)
        {
          selfpose2_x = x[matePose_index];
          selfpose2_y = y[matePose_index];
          matePose_x = x[matePose_index];
          matePose_y = x[matePose_index];
        }
        bool flag = false;
        for (int i = 0; i < x.size(); i++)
        {
          if (i != selfPose_index && i != matePose_index)
          {
            if (!flag)
            {
              sentry_lost = false;
              enemypose1_x = x[i];
              enemypose1_y = y[i];
              enemyPose1.pose.position.x = enemypose1_x;
              enemyPose1.pose.position.y = enemypose1_y;
              flag = true;
            }
            else
            {
              sentry_lost = false;
              enemypose2_x = x[i];
              enemypose2_y = y[i];
              enemyPose2.pose.position.x = enemypose2_x;
              enemyPose2.pose.position.y = enemypose2_y;
            }
          }
        }
      }
    }

    void VisionCallback(const roborts_msgs::Vision::ConstPtr &vision_message)
    {
      is_shoot = static_cast<unsigned int>(vision_message->is_shoot); // 0代表不射击, 1代表射击
      armor_x = vision_message->armor_x / 1000;                       //返回击打装甲板相对底盘的左右偏移距离, 单位m
      armor_z = vision_message->armor_z / 1000;                       //返回击打装甲板相对底盘的距离, 单位m
    }

    bool ObstacleJudge(float x, float y, float distance, float tan)
    {
      float path_x = x;
      float path_y = y;
      for (int i = 0; i < int(distance * 10); i++)
      {
        path_x += 0.1;
        path_y += 0.1 * tan;
        unsigned xx;
        unsigned yy;
        costmap_2d_->World2Map(path_x, path_y, xx, yy);
        if (costmap_2d_->GetCost(xx, yy) > 253)
        {
          return true; //有障碍
        }
      }
      return false; //无障碍
    }

    void PowerMap()
    {
      //确定夹击目标比较score确定目标 (60%障碍物、25%敌方血量、15%子弹数)
      geometry_msgs::PoseStamped pose = GetRobotMapPose();
      float enemy1_distance1 = enemypose1_x - pose.pose.position.x;
      float enemy2_distance1 = enemypose2_x - pose.pose.position.x;
      float enemy1_distance2 = enemypose1_x - matePose_x;
      float enemy2_distance2 = enemypose2_x - matePose_x;

      float tan1 = (enemypose1_y - pose.pose.position.y) / (enemypose1_x - pose.pose.position.x);
      float tan2 = (enemypose2_y - pose.pose.position.y) / (enemypose2_x - pose.pose.position.x);
      float tan3 = (enemypose1_y - matePose_y) / (enemypose1_x - matePose_x);
      float tan4 = (enemypose2_y - matePose_y) / (enemypose2_x - matePose_x);

      float enemy1_score = enemy1_hp_ / 2000.0 * 0.25 + (200 - enemy1_bullet_) / 200.0 * 0.15 + 0.3 * ObstacleJudge(pose.pose.position.x, pose.pose.position.y, enemy1_distance1, tan1) + 0.3 * ObstacleJudge(matePose_x, matePose_y, enemy1_distance2, tan3);
      float enemy2_score = enemy2_hp_ / 2000.0 * 0.25 + (200 - enemy2_bullet_) / 200.0 * 0.15 + 0.3 * ObstacleJudge(pose.pose.position.x, pose.pose.position.y, enemy2_distance1, tan2) + 0.3 * ObstacleJudge(matePose_x, matePose_y, enemy2_distance2, tan4);
      if (enemy1_score > enemy2_score)
      {
        huntpose_x = enemypose1_x;
        huntpose_y = enemypose1_y;
      }
      else
      {
        huntpose_x = enemypose2_x;
        huntpose_y = enemypose2_y;
      }

      //确定夹击半径和位姿
      if (huntpose_x > 3.34 && huntpose_x < 4.74)
      {
        if (huntpose_y > 0 && huntpose_y < 0.935 || huntpose_y > 3.4 && huntpose_y < 4.04)
        {
          huntradius = 1;
          huntcount = 2;
        }
      }
    }

    /*——————————————————FUNCTION_Public_Use—————————————————————*/
    void LoadParam(const std::string &proto_file_path)
    {
      if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config))
      {
        ROS_ERROR("Load param failed !");
        return;
      }
      /*  ——————————————————boot参数————————————————*/
      if (is_master_)
      {
        boot_position_.resize(decision_config.master_bot().size());
        for (int i = 0; i != decision_config.master_bot().size(); i++)
        {
          boot_position_[i].header.frame_id = "map";
          boot_position_[i].pose.position.x = decision_config.master_bot(i).x();
          boot_position_[i].pose.position.z = decision_config.master_bot(i).z();
          boot_position_[i].pose.position.y = decision_config.master_bot(i).y();
          tf::Quaternion master_quaternion = tf::createQuaternionFromRPY(decision_config.master_bot(i).roll(),
                                                                         decision_config.master_bot(i).pitch(),
                                                                         decision_config.master_bot(i).yaw());
          boot_position_[i].pose.orientation.x = master_quaternion.x();
          boot_position_[i].pose.orientation.y = master_quaternion.y();
          boot_position_[i].pose.orientation.z = master_quaternion.z();
          boot_position_[i].pose.orientation.w = master_quaternion.w();
        }
      }
      else
      {
        boot_position_.resize(decision_config.auxe_bot().size());
        for (int i = 0; i != decision_config.auxe_bot().size(); i++)
        {
          boot_position_[i].header.frame_id = "map";
          boot_position_[i].pose.position.x = decision_config.auxe_bot(i).x();
          boot_position_[i].pose.position.z = decision_config.auxe_bot(i).z();
          boot_position_[i].pose.position.y = decision_config.auxe_bot(i).y();
          tf::Quaternion auxe_quaternion = tf::createQuaternionFromRPY(decision_config.auxe_bot(i).roll(),
                                                                       decision_config.auxe_bot(i).pitch(),
                                                                       decision_config.auxe_bot(i).yaw());
          boot_position_[i].pose.orientation.x = auxe_quaternion.x();
          boot_position_[i].pose.orientation.y = auxe_quaternion.y();
          boot_position_[i].pose.orientation.z = auxe_quaternion.z();
          boot_position_[i].pose.orientation.w = auxe_quaternion.w();
        }
      }
      /*  ——————————————————站桩输出位置参数————————————————*/
      static_points_size_ = decision_config.point().size();
      static_position_.resize(static_points_size_);
      for (int i = 0; i != static_points_size_; i++)
      {
        static_position_[i].header.frame_id = "map";
        static_position_[i].pose.position.x = decision_config.point(i).x();
        static_position_[i].pose.position.z = decision_config.point(i).z();
        static_position_[i].pose.position.y = decision_config.point(i).y();
        tf::Quaternion master_quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                                       decision_config.point(i).pitch(),
                                                                       decision_config.point(i).yaw());
        static_position_[i].pose.orientation.x = master_quaternion.x();
        static_position_[i].pose.orientation.y = master_quaternion.y();
        static_position_[i].pose.orientation.z = master_quaternion.z();
        static_position_[i].pose.orientation.w = master_quaternion.w();
      }
      /*  ——————————————————search参数————————————————*/
      search_points_size_ = decision_config.search_path().size();
      search_point.resize(search_points_size_);
      for (int i = 0; i != search_points_size_; i++)
      {
        search_point[i].header.frame_id = "map";
        search_point[i].pose.position.x = decision_config.search_path(i).x();
        search_point[i].pose.position.y = decision_config.search_path(i).y();
        search_point[i].pose.position.z = decision_config.search_path(i).z();

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.search_path(i).roll(),
                                                                decision_config.search_path(i).pitch(),
                                                                decision_config.search_path(i).yaw());
        search_point[i].pose.orientation.x = quaternion.x();
        search_point[i].pose.orientation.y = quaternion.y();
        search_point[i].pose.orientation.z = quaternion.z();
        search_point[i].pose.orientation.w = quaternion.w();
        // ROS_INFO("get search x:%f,search y:%f",search_point[i].pose.position.x,search_point[i].pose.position.y);
      }
      /*  ——————————————————e_search参数————————————————*/
      search_points_size_ = decision_config.search_path().size();
      e_search_point.resize(search_points_size_);
      for (int i = 0; i != search_points_size_; i++)
      {
        e_search_point[i].header.frame_id = "map";
        e_search_point[i].pose.position.x = decision_config.e_search_path(i).x();
        e_search_point[i].pose.position.y = decision_config.e_search_path(i).y();
        e_search_point[i].pose.position.z = decision_config.e_search_path(i).z();

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.e_search_path(i).roll(),
                                                                decision_config.e_search_path(i).pitch(),
                                                                decision_config.e_search_path(i).yaw());
        e_search_point[i].pose.orientation.x = quaternion.x();
        e_search_point[i].pose.orientation.y = quaternion.y();
        e_search_point[i].pose.orientation.z = quaternion.z();
        e_search_point[i].pose.orientation.w = quaternion.w();
      }
      /*  ——————————————————c_search参数————————————————*/
      search_points_size_ = decision_config.search_path().size();
      c_search_point.resize(search_points_size_);
      for (int i = 0; i != search_points_size_; i++)
      {
        c_search_point[i].header.frame_id = "map";
        c_search_point[i].pose.position.x = decision_config.c_search_path(i).x();
        c_search_point[i].pose.position.y = decision_config.c_search_path(i).y();
        c_search_point[i].pose.position.z = decision_config.c_search_path(i).z();

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.c_search_path(i).roll(),
                                                                decision_config.c_search_path(i).pitch(),
                                                                decision_config.c_search_path(i).yaw());
        c_search_point[i].pose.orientation.x = quaternion.x();
        c_search_point[i].pose.orientation.y = quaternion.y();
        c_search_point[i].pose.orientation.z = quaternion.z();
        c_search_point[i].pose.orientation.w = quaternion.w();
      }

      /*  ——————————————————F1-F6参数————————————————*/
      buff_debuff_points_.resize((unsigned int)(decision_config.buff_point().size()));
      for (int i = 0; i != (unsigned int)(decision_config.buff_point().size()); i++)
      {
        buff_debuff_points_[i].header.frame_id = "map";
        buff_debuff_points_[i].pose.position.x = decision_config.buff_point(i).x();
        buff_debuff_points_[i].pose.position.y = decision_config.buff_point(i).y();
        buff_debuff_points_[i].pose.position.z = decision_config.buff_point(i).z();

        tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                                decision_config.buff_point(i).pitch(),
                                                                decision_config.buff_point(i).yaw());
        buff_debuff_points_[i].pose.orientation.x = quaternion.x();
        buff_debuff_points_[i].pose.orientation.y = quaternion.y();
        buff_debuff_points_[i].pose.orientation.z = quaternion.z();
        buff_debuff_points_[i].pose.orientation.w = quaternion.w();
        // ROS_INFO("get buff x:%f,buff y:%f",buff_debuff_points_[i].pose.position.x,buff_debuff_points_[i].pose.position.y);
      }
      int region = GetCurrentRegion();
      if (region == 1 || region == 2)
      {
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 3.14);
        buff_debuff_points_[2].pose.orientation.x = quaternion.x();
        buff_debuff_points_[2].pose.orientation.y = quaternion.y();
        buff_debuff_points_[2].pose.orientation.z = quaternion.z();
        buff_debuff_points_[2].pose.orientation.w = quaternion.w();
        buff_debuff_points_[3].pose.orientation.x = quaternion.x();
        buff_debuff_points_[3].pose.orientation.y = quaternion.y();
        buff_debuff_points_[3].pose.orientation.z = quaternion.z();
        buff_debuff_points_[3].pose.orientation.w = quaternion.w();
      }
    }

    int GetCurrentRegion()
    {
      geometry_msgs::PoseStamped current_pose = GetRobotMapPose();
      double x, y;
      x = current_pose.pose.position.x;
      y = current_pose.pose.position.y;
      if (x <= 4.04)
      {
        if (y > 2.24)
        {
          return 0;
        }
        else
        {
          return 3;
        }
      }
      else
      {
        if (y > 2.4)
        {
          return 1;
        }
        else
        {
          return 2;
        }
      }
    }

    double GetDistance(const geometry_msgs::PoseStamped &pose1,
                       const geometry_msgs::PoseStamped &pose2)
    //从两个位置获取距离
    {
      const geometry_msgs::Point point1 = pose1.pose.position;
      const geometry_msgs::Point point2 = pose2.pose.position;
      const double dx = point1.x - point2.x;
      const double dy = point1.y - point2.y;
      return std::sqrt(dx * dx + dy * dy);
    }

    double GetAngle(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2)
    {
      const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation; //将四元数 msg 转换为四元数,
      const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(quaternion1, rot1);
      tf::quaternionMsgToTF(quaternion2, rot2);
      return rot1.angleShortestPath(rot2);
    }

    geometry_msgs::PoseStamped GetRobotMapPose()
    {
      UpdateRobotPose();
      return robot_map_pose_;
    }

    void UpdateRobotPose()
    {
      tf::Stamped<tf::Pose> robot_tf_pose;
      robot_tf_pose.setIdentity();
      robot_tf_pose.frame_id_ = "base_link";
      robot_tf_pose.stamp_ = ros::Time();
      try
      {
        geometry_msgs::PoseStamped robot_pose;
        tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
        tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
      }
      catch (tf::LookupException &e)
      {
        ROS_ERROR("Transform Error looking up robot pose: %s", e.what());
      }
    }

    /*——————————————————FUNCTION_Condition_Use—————————————————————*/
    // Buff_About_Position
    bool IsAllBuffDistanceNear()
    {
      double distance[6];
      geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
      for (int i = 0; i <= 5; i++)
      {
        auto dx = robot_map_pose.pose.position.x - buff_debuff_points_[i].pose.position.x;
        auto dy = robot_map_pose.pose.position.y - buff_debuff_points_[i].pose.position.y;
        distance[i] = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      }
      for (int i = 0; i <= 5; i++)
      {
        if (distance[i] < 0.5)
        {
          return true;
        }
      }
      return false;
    }

    /*——————————————————Buff_About_Time—————————————————————*/
    bool IsNearRefresh()
    {
      if (blood_active_ || bullet_active_)
        return true;
      else
        return false;
    }

    bool IsNearRefreshTime()
    {
      if ((remaining_time_ > 120.5) && (remaining_time_ <= 122))
        return true;
      if ((remaining_time_ > 60.5) && (remaining_time_ <= 62))
        return true;
      return false;
    }
    // Buff_About_Blood
    bool IsBloodAdvantage()
    {
      if (((int)remain_hp_ + (int)teammate_hp_ - (int)enemy1_hp_ - (int)enemy2_hp_ >= 400) || (remain_hp_ > 500))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    bool IsBloodLossFast()
    {
      if (is_blood_lose_fast == true)
      {
        if ((self_blood_loss_per_second_[0] + self_blood_loss_per_second_[1]) <= 80.0)
        {
          is_blood_lose_fast = false;
          return false;
        }
        else
        {
          is_blood_lose_fast = true;
          return true;
        }
      }
      if ((self_blood_loss_per_second_[0] + self_blood_loss_per_second_[1]) >= 200.0 && (self_blood_loss_per_second_[0] + self_blood_loss_per_second_[1]) > EnemyHurted3Second())
      {
        is_blood_lose_fast = true;
        return true;
      }
      else
      {
        is_blood_lose_fast = false;
        return false;
      }
    }

    /*——————————————————Buff_About_Bullet—————————————————————*/
    bool IsBulletAdvantage()
    {
      if ((int)remain_bullet_ + (int)teammate_bullet_ - (int)enemy1_bullet_ - (int)enemy2_bullet_ >= 150)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /*——————————————————Buff_About_Partner—————————————————————*/
    bool IsMateDie()
    {
      if ((int)teammate_hp_ == 0)
        return true;
      else
      {
        return false;
      }
    }

    bool IsMoreBloodThanMate()
    {
      return (remain_hp_ >= teammate_hp_);
    }

    bool IsLessBulletThanMate()
    {
      return (remain_bullet_ <= teammate_bullet_);
    }

    bool IsMoreBulletThanMate()
    {
      return (remain_bullet_ >= teammate_bullet_);
    }

    /*——————————————————FUNCTION_Ation_Use—————————————————————*/
    geometry_msgs::PoseStamped GetBuffPoints(int b_count)
    {
      return buff_debuff_points_[b_count];
    }

    geometry_msgs::PoseStamped GetLeastStaticPose()
    {
      geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
      double distance[static_points_size_];
      for (int i = 0; i < static_points_size_; i++)
      {
        double dx = robot_map_pose.pose.position.x - static_position_[i].pose.position.x;
        double dy = robot_map_pose.pose.position.y - static_position_[i].pose.position.y;
        distance[i] = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      }
    }

    geometry_msgs::PoseStamped GetStopPose()
    {
      int robot_id_ = id_;
      if (1 == robot_id_)
      {
        stop_pose_.pose.position.x = 5.14;
        stop_pose_.pose.position.y = 1.59;
        geometry_msgs::PoseStamped enemy_pose = stop_pose_;
        geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
        double threa;
        if (enemy_pose.pose.position.x > robot_map_pose.pose.position.x)
          threa = 0;
        else
          threa = 3.14;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, threa);
        stop_pose_.pose.orientation.x = quaternion.x();
        stop_pose_.pose.orientation.y = quaternion.y();
        stop_pose_.pose.orientation.z = quaternion.z();
        stop_pose_.pose.orientation.w = quaternion.w();
      }
      if (2 == robot_id_)
      {
        stop_pose_.pose.position.x = 5.14;
        stop_pose_.pose.position.y = 2.89;
        geometry_msgs::PoseStamped enemy_pose = stop_pose_;

        geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
        double threa;
        if (enemy_pose.pose.position.x > robot_map_pose.pose.position.x)
          threa = 0;
        else
          threa = 3.14;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, threa);
        stop_pose_.pose.orientation.x = quaternion.x();
        stop_pose_.pose.orientation.y = quaternion.y();
        stop_pose_.pose.orientation.z = quaternion.z();
        stop_pose_.pose.orientation.w = quaternion.w();
      }
      if (101 == robot_id_)
      {
        stop_pose_.pose.position.x = 3.54;
        stop_pose_.pose.position.y = 2.89;
        geometry_msgs::PoseStamped enemy_pose = stop_pose_;
        geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
        double threa;
        if (enemy_pose.pose.position.x > robot_map_pose.pose.position.x)
          threa = 0;
        else
          threa = 3.14;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, threa);
        stop_pose_.pose.orientation.x = quaternion.x();
        stop_pose_.pose.orientation.y = quaternion.y();
        stop_pose_.pose.orientation.z = quaternion.z();
        stop_pose_.pose.orientation.w = quaternion.w();
      }
      if (102 == robot_id_)
      {
        stop_pose_.pose.position.x = 3.54;
        stop_pose_.pose.position.y = 1.59;
        geometry_msgs::PoseStamped enemy_pose = stop_pose_;
        geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
        double threa;
        if (enemy_pose.pose.position.x > robot_map_pose.pose.position.x)
          threa = 0;
        else
          threa = 3.14;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, threa);
        stop_pose_.pose.orientation.x = quaternion.x();
        stop_pose_.pose.orientation.y = quaternion.y();
        stop_pose_.pose.orientation.z = quaternion.z();
        stop_pose_.pose.orientation.w = quaternion.w();
      }
      return stop_pose_;
    }

    /*——————————————————FUNCTION_AboutBuff———————————————————*/
    double GetBloodBuffDistance()
    {
      geometry_msgs::PoseStamped buff_goal = blood_pose_;
      geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
      auto dx = robot_map_pose.pose.position.x - buff_goal.pose.position.x;
      auto dy = robot_map_pose.pose.position.y - buff_goal.pose.position.y;
      double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
      return distance;
    }

    double EnemyHurted3Second()
    {
      double enemy1 = enemy1_blood_loss_per_second_[0] + enemy1_blood_loss_per_second_[1];
      double enemy2 = enemy2_blood_loss_per_second_[0] + enemy2_blood_loss_per_second_[1];
      return (enemy1 > enemy2 ? enemy1 : enemy2);
    }

    /*——————————————————Shared Ptr—————————————————————*/
    typedef std::shared_ptr<Blackboard> Ptr;
    std::shared_ptr<tf::TransformListener> tf_ptr_;
    std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
    std::shared_ptr<GimbalExecutor> gimbal_executor_ptr_;

    /*——————————————————CostMap—————————————————————*/
    const std::string &proto_file_path_;
    typedef roborts_costmap::CostmapInterface CostMap;
    typedef roborts_costmap::Costmap2D CostMap2D;
    std::shared_ptr<CostMap> costmap_ptr_;
    CostMap2D *costmap_2d_;
    unsigned char *charmap_;

    /*——————————————————Execute basis variable—————————————————————*/
    int escape_region_ = 0;
    double go_goal_x_ = 6.28;
    double go_goal_y_ = 3.68;
    unsigned int search_count_ = 0;
    unsigned int e_search_count_ = 0;
    unsigned int c_search_count_ = 0;
    unsigned int search_points_size_;
    unsigned int static_points_size_;
    BehaviorState action_state_;
    roborts_decision::DecisionConfig decision_config;

    /*——————————————————————PowerMap—————————————————————*/
    float huntpose_x;
    float huntpose_y;
    float huntradius = 1.2;
    int huntcount = 6;

    /*——————————————————————Interact—————————————————————*/
    bool is_connected = false;
    bool is_master_ = true;
    bool mate_gimbal_;
    bool mate_chassis_;
    bool mate_punished_;
    bool mate_tree_running_;
    bool mate_chase_enemy_;
    double goalpose_yaw = 10;
    double goalpose_x = 10;
    double goalpose_y = 10; //传出的目标

    double matePose_x = 0;
    double matePose_y = 0; //收到的伙伴的位置
    geometry_msgs::PoseStamped matePose;
    geometry_msgs::PoseStamped goalPose;
    // unsigned int mate_status_ = 0;
    int feedback;     //失败0 完成1 运行2
    int command = -1; // The default is zero, so be careful when confirming tasks
    int readytoshoot = 0;
    int teammate_hp_;
    int mate_camera_lost;
    int teammate_bullet_;

    int sentry_connected;
    bool sentry_lost;

    bool red1 = false;
    bool red2 = false;
    bool blue1 = false;
    bool blue2 = false;

    double selfpose1_x = 2.44; //哨岗拿到的自己坐标 1
    double selfpose1_y = 2.44;
    double selfpose2_x = 2.44; //哨岗拿到的队友坐标 2
    double selfpose2_y = 2.44;

    double enemypose1_x = 2.44; //哨岗拿到的敌人坐标 1
    double enemypose1_y = 2.44;
    double enemypose2_x = 2.44; //哨岗拿到的敌人坐标 2
    double enemypose2_y = 2.44;

    geometry_msgs::PoseStamped enemyPose1;
    geometry_msgs::PoseStamped enemyPose2;

    int is_shoot;   //视觉是否射击
    double armor_x; //击打装甲板相对底盘的左右偏移距离
    double armor_z; //击打装甲板相对底盘的距离

    /*——————————————————Judgment basis variable—————————————————————*/
    bool is_search_;
    bool is_go_goal_;
    bool is_go_buff_;
    bool is_follow_;
    bool camera_lost_;
    bool cancel_flag_;
    bool is_swing_;
    bool if_punished_;
    bool gimbal_punished_;
    bool chassis_punished_;
    bool has_followed_;
    bool gain_bullets_flag;
    bool gain_blood_flag;
    bool has_chase_enemy_;
    bool is_blood_lose_fast;
    bool is_scan_;
    double self_blps_;
    ros::Time last_check_attacked_time_;
    ros::Time last_time_;
    unsigned int near_enemy_num_;

    /*——————————————————From The RefreeSystem—————————————————————*/
    int id_;
    int lurk_mode = 0;
    int red1_hp = 2000;
    int red2_hp = 2000;
    int blue1_hp = 2000;
    int blue2_hp = 2000;
    int red1_bullet = 0;
    int red2_bullet = 0;
    int blue1_bullet = 0;
    int blue2_bullet = 0;
    int enemy1_bullet_ = 0;
    int enemy2_bullet_ = 0;
    int enemy1_hp_ = 2000;
    int enemy2_hp_ = 2000;
    int remain_bullet_ = 0;
    int remain_hp_ = 2000;
    unsigned int game_result_ = 0;
    int red_bonus_status_;
    int blue_bonus_status_;
    int supplier_status_;
    int robot_heat_;
    int damage_type_;
    int damage_source_;
    int robot_shoot_frequency_;
    bool Zone_active[6];
    bool blood_active_ = false;
    bool bullet_active_ = false;
    bool debuff1_active_ = false;
    bool debuff2_active_ = false;
    bool red3_survival_;
    bool red4_survival_;
    bool blue3_survival_;
    bool blue4_survival_;
    bool robot_bonus_;
    double robot_shoot_speed_;
    unsigned int Zone_status[6];
    unsigned int game_status_;
    unsigned int remaining_time_;

    /*——————————————————Deal With The Data—————————————————————*/
    int hp_count = 0;
    double self_hp_delta_;
    double mate_hp_delta_;
    double enemy1_hp_delta_;
    double enemy2_hp_delta_;
    double time_delta_;
    double self_blood_loss_per_second_[2];
    double mate_blood_loss_per_second_[2];
    double enemy1_blood_loss_per_second_[2];
    double enemy2_blood_loss_per_second_[2];

    /*——————————————————关于位置的变量————————————*/
    /*vector*/
    std::vector<geometry_msgs::PoseStamped> boot_position_;
    std::vector<geometry_msgs::PoseStamped> buff_debuff_points_;
    std::vector<geometry_msgs::PoseStamped> search_point;
    std::vector<geometry_msgs::PoseStamped> e_search_point;
    std::vector<geometry_msgs::PoseStamped> c_search_point;
    std::vector<geometry_msgs::PoseStamped> static_position_;

    /*single*/
    geometry_msgs::PoseStamped blood_pose_;
    geometry_msgs::PoseStamped buff_pose_;
    geometry_msgs::PoseStamped bullet_pose_;
    geometry_msgs::PoseStamped debuff1_pose_;
    geometry_msgs::PoseStamped debuff2_pose_;
    geometry_msgs::PoseStamped follow_pose_;
    geometry_msgs::PoseStamped mate_goal_;
    geometry_msgs::PoseStamped robot_map_pose_;
    geometry_msgs::PoseStamped stop_pose_;

  private:
    ros::Subscriber bonus_status_sub_;
    ros::Subscriber camera_sub;
    ros::Subscriber companion_info_sub;
    ros::Subscriber enemy_pose_sub;
    ros::Subscriber game_result_sub_;
    ros::Subscriber game_status_sub_;
    ros::Subscriber game_zone_array_status_sub_;
    ros::Subscriber robot_bonus_sub_;
    ros::Subscriber robot_bullet_sub_;
    ros::Subscriber roborts_connect_sub_;
    ros::Subscriber robot_damage_sub_;
    ros::Subscriber robot_heat_sub_;
    ros::Subscriber robot_hp_sub_;
    ros::Subscriber robot_shoot_sub_;
    ros::Subscriber ros_robot_status_sub_;
    ros::Subscriber sentry_posts_sub_;
    ros::Subscriber supplier_status_sub_;
    ros::Subscriber robot_lurk_sub_;
    ros::Subscriber vision_sub_;
  };
}
#endif
