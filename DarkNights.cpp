#include <ros/ros.h>
#include <memory>
#include "behavior_tree/behavior_tree.h"
#include "behavior_tree/behavior_node.h"
#include "behavior_tree/behavior_state.h"
#include "aurora/back_boot_area_behavior.h"
#include "aurora/chase_behavior.h"
#include "aurora/chassis_limited_behavior.h"
#include "aurora/defend_behavior.h"
#include "aurora/escape_behavior.h"
#include "aurora/follow_behavior.h"
#include "aurora/frozen_behavior.h"
#include "aurora/gain_blood_behavior.h"
#include "aurora/gain_bullets_behavior.h"
#include "aurora/get_supply_behavior.h"
#include "aurora/gimbal_limited_behavior.h"
#include "aurora/hunt_master_behavior.h"
#include "aurora/hunt_slave_behavior.h"
#include "aurora/line_iterator.h"
#include "aurora/search_behavior.h"
#include "aurora/shoot_behavior.h"
#include "aurora/turn_defend_behavior.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");
    ros::Time::init();
    std::string config_file_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    auto blackboard_ptr = std::make_shared<roborts_decision::Blackboard>(config_file_path);
    auto goal_factory = std::make_shared<roborts_decision::GoalFactory>(blackboard_ptr);

    // The ActionNode
    auto back_boot_area_behavior = std::make_shared<roborts_decision::BackBootArea>(blackboard_ptr, goal_factory);
    auto chase_behavior = std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr, goal_factory);
    auto chassis_limited_behavior = std::make_shared<roborts_decision::ChassisLimited>(blackboard_ptr, goal_factory);
    auto defend_behavior = std::make_shared<roborts_decision::DefendAction>(blackboard_ptr, goal_factory);
    auto escape_behavior = std::make_shared<roborts_decision::EsCape>(blackboard_ptr, goal_factory);
    auto follow_behavior = std::make_shared<roborts_decision::Follow>(blackboard_ptr, goal_factory);
    auto frozen_behavior = std::make_shared<roborts_decision::FrozeAction>(blackboard_ptr, goal_factory);
    auto gain_blood_behavior = std::make_shared<roborts_decision::GainBlood>(blackboard_ptr, goal_factory);
    auto gain_bullets_behavior = std::make_shared<roborts_decision::GainBullets>(blackboard_ptr, goal_factory);
    auto get_supply_behavior = std::make_shared<roborts_decision::GetSupplyBehavior>(blackboard_ptr, goal_factory);
    auto gimbal_limited_behavior = std::make_shared<roborts_decision::GimbalLimited>(blackboard_ptr, goal_factory);
    auto hunt_master_behavior = std::make_shared<roborts_decision::commandHuntAction>(blackboard_ptr, goal_factory);
    auto hunt_slave_behavior = std::make_shared<roborts_decision::ExcuteHuntAction>(blackboard_ptr, goal_factory);
    auto search_behavior = std::make_shared<roborts_decision::SearchAction>(blackboard_ptr, goal_factory);
    auto shoot_behavior = std::make_shared<roborts_decision::ShootAction>(blackboard_ptr, goal_factory);
    auto turn_defend_behavior = std::make_shared<roborts_decision::TurnDefend>(blackboard_ptr, goal_factory);

    // the SelectorNode
    auto game_status_selector = std::make_shared<roborts_decision::SelectorNode>("game_status_selector", blackboard_ptr);
    auto game_start_selector = std::make_shared<roborts_decision::SelectorNode>("game_start_selector", blackboard_ptr);
    auto under_punish_selector = std::make_shared<roborts_decision::SelectorNode>("under_punish_selector", blackboard_ptr);
    auto buff_fresh_selector = std::make_shared<roborts_decision::SelectorNode>("buff_fresh_selector", blackboard_ptr);
    auto enemy_lost_selector = std::make_shared<roborts_decision::SelectorNode>("enemy_lost_selector", blackboard_ptr);
    auto enemy_distance_selector = std::make_shared<roborts_decision::SelectorNode>("enemy_distance_selector", blackboard_ptr);
    auto master_selector = std::make_shared<roborts_decision::SelectorNode>("master_selector", blackboard_ptr);
    auto client_action_selector = std::make_shared<roborts_decision::SelectorNode>("client_action_selector", blackboard_ptr);
    auto master_action_selector = std::make_shared<roborts_decision::SelectorNode>("master_action_selector", blackboard_ptr);
    // auto feedback_selector = std::make_shared<roborts_decision::SelectorNode>("master_action_selector", blackboard_ptr);
    auto emergent_selector = std::make_shared<roborts_decision::SelectorNode>("emergent_selector", blackboard_ptr);

    // the ConditionNode
    auto game_not_start_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "game_not_start_condition", blackboard_ptr,
        [&]()
        {
            return ((blackboard_ptr->game_status_ != 4) && (blackboard_ptr->game_status_ != 5));
        },
        roborts_decision::AbortType::BOTH);

    auto game_over_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "game_over_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->game_status_ == 5);
        },
        roborts_decision::AbortType::BOTH);

    auto game_start_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "game_start_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->game_status_ == 4);
        },
        roborts_decision::AbortType::BOTH);

    auto under_punish_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "under_punish_condition", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->if_punished_;
        },
        roborts_decision::AbortType::BOTH);

    auto gimbal_punished_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "gimbal_pubnished_condition", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->gimbal_punished_;
        },
        roborts_decision::AbortType::BOTH);

    auto chassis_punished_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "chassis_pubnished_condition", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->chassis_punished_;
        },
        roborts_decision::AbortType::BOTH);
    auto gain_blood_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "gain_blood_condition", blackboard_ptr,
        [&]()
        {
            return (!blackboard_ptr->IsBloodAdvantage() && blackboard_ptr->blood_active_ && ((blackboard_ptr->IsMoreBloodThanMate() || blackboard_ptr->IsMateDie())));
        },
        roborts_decision::AbortType::BOTH);
    auto gain_bullet_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "gain_bullet_condition", blackboard_ptr,
        [&]()
        {
            return (!blackboard_ptr->IsBulletAdvantage() && blackboard_ptr->bullet_active_ && ((blackboard_ptr->IsLessBulletThanMate() || blackboard_ptr->IsMateDie())));
        },
        roborts_decision::AbortType::BOTH);
    auto near_buff_refresh_time_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "near_buff_refresh_time_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->IsNearRefresh());
        },
        roborts_decision::AbortType::BOTH);

    auto is_near_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "is_near_buff_condition_", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->IsAllBuffDistanceNear() && blackboard_ptr->IsNearRefreshTime());
        },
        roborts_decision::AbortType::BOTH);
    auto blood_endangered_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "blood_endangered_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->remain_hp_ < 300);
        },
        roborts_decision::AbortType::BOTH);
    auto bullets_endangered_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "bullets_endangered_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->remain_bullet_ == 0);
        },
        roborts_decision::AbortType::BOTH);

    auto blood_loss_fast_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "blood_loss_fast_condition", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->IsBloodLossFast();
        },
        roborts_decision::AbortType::BOTH);

    auto enemy_lost_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "enemy_lost_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->sentry_lost && blackboard_ptr->is_shoot);
        },
        roborts_decision::AbortType::BOTH);

    auto enemy_not_lost_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "enemy_not_lost_condition", blackboard_ptr,
        [&]()
        {
            return (!(blackboard_ptr->sentry_lost && blackboard_ptr->is_shoot));
        },
        roborts_decision::AbortType::BOTH);

    auto enemy_distance_near_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "enemy_distance_near_condition", blackboard_ptr,
        [&]()
        {
            return (blackboard_ptr->is_shoot);
        },
        roborts_decision::AbortType::BOTH);

    auto enemy_distance_far_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "enemy_distance_far_condition", blackboard_ptr,
        [&]()
        {
            return (!blackboard_ptr->is_shoot);
        },
        roborts_decision::AbortType::BOTH);

    auto connected_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "connected_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->is_connected;
        },
        roborts_decision::AbortType::BOTH);

    auto ismaster_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "ismaster_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->is_master_;
        },
        roborts_decision::AbortType::BOTH);

    auto isclient_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "isclient_condition_", blackboard_ptr,
        [&]()
        {
            return !blackboard_ptr->is_master_;
        },
        roborts_decision::AbortType::BOTH);

    auto command_search_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "command_search_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->sentry_lost && !blackboard_ptr->is_shoot;
        },
        roborts_decision::AbortType::BOTH);

    auto command_hunt_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "command_hunt_condition_", blackboard_ptr,
        [&]()
        {
            return !blackboard_ptr->sentry_lost;
        },
        roborts_decision::AbortType::BOTH);

    auto command_gainblood_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "command_gainblood_condition_", blackboard_ptr,
        [&]()
        {
            return (!blackboard_ptr->IsBloodAdvantage() && blackboard_ptr->blood_active_ && ((blackboard_ptr->IsMoreBloodThanMate() || blackboard_ptr->IsMateDie())));
        },
        roborts_decision::AbortType::BOTH);

    auto command_gainbullet_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "command_gainbullet_condition_", blackboard_ptr,
        [&]()
        {
            return (!blackboard_ptr->IsBulletAdvantage() && blackboard_ptr->bullet_active_ && ((blackboard_ptr->IsLessBulletThanMate() || blackboard_ptr->IsMateDie())));
        },
        roborts_decision::AbortType::BOTH);

    auto excute_search_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "excute_search_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->command == 4;
        },
        roborts_decision::AbortType::BOTH);

    auto excute_hunt_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "excute_hunt_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->command == 3;
        },
        roborts_decision::AbortType::BOTH);

    auto excute_gainblood_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "excute_gainblood_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->command == 0;
        },
        roborts_decision::AbortType::BOTH);

    auto excute_gainbullet_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "excute_gainbullet_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->command == 1;
        },
        roborts_decision::AbortType::BOTH);

    // auto urgent_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
    //     "urgent_condition_", blackboard_ptr,
    //     [&]()
    //     {
    //         // FIXME
    //         return true;
    //     },
    //     roborts_decision::AbortType::BOTH);
    // auto nonurgent_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
    //     "nonurgent_condition_", blackboard_ptr,
    //     [&]()
    //     {
    //         // FIXME
    //         return true;
    //     },
    //     roborts_decision::AbortType::BOTH);
    // auto feedback_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
    //     "feedback_condition_", blackboard_ptr,
    //     [&]()
    //     {
    //         // FIXME
    //         return true;
    //     },
    //     roborts_decision::AbortType::BOTH);

    auto command_escape_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "command_escape_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->remain_hp_ < 300;
        },
        roborts_decision::AbortType::BOTH);

    auto command_defend_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "command_defend_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->remain_bullet_ == 0;
        },
        roborts_decision::AbortType::BOTH);

    auto excute_escape_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "excute_escape_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->command == 5;
        },
        roborts_decision::AbortType::BOTH);

    auto excute_defend_condition_ = std::make_shared<roborts_decision::PreconditionNode>(
        "excute_defend_condition_", blackboard_ptr,
        [&]()
        {
            return blackboard_ptr->command == 6;
        },
        roborts_decision::AbortType::BOTH);

    /*——————————————————DARK KNIGHTS BEHAVIOR TREE  ———————————————————*/

    game_status_selector->AddChildren(game_not_start_condition_);
    /****/ game_not_start_condition_->SetChild(frozen_behavior);
    game_status_selector->AddChildren(game_over_condition_);
    /****/ game_over_condition_->SetChild(back_boot_area_behavior);
    game_status_selector->AddChildren(game_start_condition_);
    /****/ game_start_condition_->SetChild(game_start_selector);
    /********/ game_start_selector->AddChildren(under_punish_condition_);
    /************/ under_punish_condition_->SetChild(under_punish_selector);
    /****************/ under_punish_selector->AddChildren(gimbal_punished_condition_);
    /********************/ gimbal_punished_condition_->SetChild(escape_behavior);
    /****************/ under_punish_selector->AddChildren(chassis_punished_condition_);
    /*******************/ chassis_punished_condition_->SetChild(frozen_behavior);
    /********/ game_start_selector->AddChildren(connected_condition_);
    /*************/ connected_condition_->SetChild(master_selector);

    /*****************/ master_selector->AddChildren(ismaster_condition_);
    /**********************/ ismaster_condition_->SetChild(master_action_selector);

    /*************************/ master_action_selector->AddChildren(command_gainblood_condition_); // 0,取血行为
    /*****************************/ command_gainblood_condition_->SetChild(gain_blood_behavior);

    /*************************/ master_action_selector->AddChildren(command_gainbullet_condition_); // 1，取弹行为
    /*****************************/ command_gainbullet_condition_->SetChild(gain_bullets_behavior);

    /*************************/ master_action_selector->AddChildren(command_hunt_condition_); // 3,夹击行为
    /*****************************/ command_hunt_condition_->SetChild(hunt_master_behavior);

    /*************************/ master_action_selector->AddChildren(command_search_condition_); // 4，搜索行为
    /*****************************/ command_search_condition_->SetChild(search_behavior);

    /*************************/ master_action_selector->AddChildren(command_escape_condition_); // 5，逃跑行为
    /*****************************/ command_gainblood_condition_->SetChild(escape_behavior);

    /*************************/ master_action_selector->AddChildren(command_defend_condition_); // 6,防御行为
    /*****************************/ command_gainbullet_condition_->SetChild(defend_behavior);

    /****************/ master_selector->AddChildren(isclient_condition_);
    /**********************/ isclient_condition_->SetChild(client_action_selector);

    /*************************/ client_action_selector->AddChildren(excute_search_condition_);
    /*****************************/ excute_search_condition_->SetChild(search_behavior);

    /*************************/ client_action_selector->AddChildren(excute_hunt_condition_);
    /*****************************/ excute_hunt_condition_->SetChild(hunt_slave_behavior);

    /*************************/ client_action_selector->AddChildren(excute_gainblood_condition_);
    /*****************************/ excute_gainblood_condition_->SetChild(get_supply_behavior);

    /*************************/ client_action_selector->AddChildren(excute_gainbullet_condition_);
    /*****************************/ excute_gainbullet_condition_->SetChild(get_supply_behavior);

    /*************************/ client_action_selector->AddChildren(excute_escape_condition_);
    /*****************************/ excute_gainblood_condition_->SetChild(escape_behavior);

    /*************************/ client_action_selector->AddChildren(excute_defend_condition_);
    /*****************************/ excute_gainbullet_condition_->SetChild(defend_behavior);

    /********/ game_start_selector->AddChildren(near_buff_refresh_time_condition_);
    /************/ near_buff_refresh_time_condition_->SetChild(buff_fresh_selector);
    /****************/ buff_fresh_selector->AddChildren(gain_blood_condition_);
    /********************/ gain_blood_condition_->SetChild(gain_blood_behavior);
    /****************/ buff_fresh_selector->AddChildren(gain_bullet_condition_);
    /********************/ gain_bullet_condition_->SetChild(gain_bullets_behavior);
    /********/ game_start_selector->AddChildren(is_near_buff_condition_);
    /************/ is_near_buff_condition_->SetChild(search_behavior);
    /********/ game_start_selector->AddChildren(blood_loss_fast_condition_);
    /************/ blood_loss_fast_condition_->SetChild(defend_behavior);
    /********/ game_start_selector->AddChildren(blood_endangered_condition_);
    /************/ blood_endangered_condition_->SetChild(escape_behavior);
    /********/ game_start_selector->AddChildren(bullets_endangered_condition_);
    /************/ bullets_endangered_condition_->SetChild(defend_behavior);
    /********/ game_start_selector->AddChildren(enemy_lost_selector);
    /************/ enemy_lost_selector->AddChildren(enemy_lost_condition_);
    /****************/ enemy_lost_condition_->SetChild(search_behavior);
    /************/ enemy_lost_selector->AddChildren(enemy_not_lost_condition_);
    /****************/ enemy_not_lost_condition_->SetChild(enemy_distance_selector);
    /********************/ enemy_distance_selector->AddChildren(enemy_distance_near_condition_);
    /************************/ enemy_distance_near_condition_->SetChild(shoot_behavior);
    /********************/ enemy_distance_selector->AddChildren(enemy_distance_far_condition_);
    /*******凸(艹皿艹 )*******/ enemy_distance_far_condition_->SetChild(search_behavior);

    ros::Rate rate(30);//这个可以改慢一点
    roborts_decision::BehaviorTree root_(game_status_selector, 100);
    while (ros::ok())
    {
        root_.Run();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
