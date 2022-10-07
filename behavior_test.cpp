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
    ros::init(argc, argv, "behavior_test_node");
    ros::Time::init();
    std::string file_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";
    auto black_ptr = std::make_shared<roborts_decision::Blackboard>(file_path);
    auto g_factory = std::make_shared<roborts_decision::GoalFactory>(black_ptr);

    auto back_boot_area_action = std::make_shared<roborts_decision::BackBootArea>(black_ptr, g_factory);
    auto chase_action = std::make_shared<roborts_decision::ChaseAction>(black_ptr, g_factory);
    auto chassis_limited_action = std::make_shared<roborts_decision::ChassisLimited>(black_ptr, g_factory);
    auto defend_action = std::make_shared<roborts_decision::DefendAction>(black_ptr, g_factory);
    auto escape_action = std::make_shared<roborts_decision::EsCape>(black_ptr, g_factory);
    auto follow_action = std::make_shared<roborts_decision::Follow>(black_ptr, g_factory);
    auto frozen_action = std::make_shared<roborts_decision::FrozeAction>(black_ptr, g_factory);
    auto gain_blood_action = std::make_shared<roborts_decision::GainBlood>(black_ptr, g_factory);
    auto gain_bullets_action = std::make_shared<roborts_decision::GainBullets>(black_ptr, g_factory);
    auto get_supply_action = std::make_shared<roborts_decision::GetSupplyBehavior>(black_ptr, g_factory);
    auto gimbal_limited_action = std::make_shared<roborts_decision::GimbalLimited>(black_ptr, g_factory);
    auto hunt_master_action = std::make_shared<roborts_decision::commandHuntAction>(black_ptr, g_factory);
    auto hunt_slave_action = std::make_shared<roborts_decision::ExcuteHuntAction>(black_ptr, g_factory);
    auto search_action = std::make_shared<roborts_decision::SearchAction>(black_ptr, g_factory);
    auto shoot_action = std::make_shared<roborts_decision::ShootAction>(black_ptr, g_factory);
    auto turn_defend_action = std::make_shared<roborts_decision::TurnDefend>(black_ptr, g_factory);
    auto game_status_selector = std::make_shared<roborts_decision::SelectorNode>("game_status_selector", black_ptr);

    roborts_decision::BehaviorTree root(game_status_selector, 100);

    ros::Rate rate(10);
    int model = 0;
    std::cout << "1:Back boot area" << std::endl
              << "2:Chase" << std::endl
              << "3:Chassis limited" << std::endl
              << "4:Defend" << std::endl
              << "5:Escape" << std::endl
              << "6:Follow" << std::endl
              << "7:Frozen" << std::endl
              << "8:Gain blood" << std::endl
              << "9:Gain bullet" << std::endl
              << " 10.Get supply" << std::endl
              << "11:Gimbal limited shoot" << std::endl
              << "12:Hunt Master" << std::endl
              << "13:Hunt Slave" << std::endl
              << "14:Search" << std::endl
              << "15:Shoot" << std::endl
              << "16:Turn defend" << std::endl;

    std::cin >> model;
    switch (model)
    {
    case 1:
    {
        game_status_selector->AddChildren(back_boot_area_action);
    }
    break;
    case 2:
    {
        game_status_selector->AddChildren(chase_action);
    }
    break;
    case 3:
    {
        game_status_selector->AddChildren(chassis_limited_action);
    }
    break;
    case 4:
    {
        game_status_selector->AddChildren(defend_action);
    }
    break;
    case 5:
    {
        game_status_selector->AddChildren(escape_action);
    }
    break;
    case 6:
    {
        game_status_selector->AddChildren(follow_action);
    }
    break;
    case 7:
    {
        game_status_selector->AddChildren(frozen_action);
    }
    break;
    case 8:
    {
        game_status_selector->AddChildren(gain_blood_action);
    }
    break;
    case 9:
    {
        game_status_selector->AddChildren(gain_bullets_action);
    }
    break;
    case 10:
    {
        game_status_selector->AddChildren(get_supply_action);
    }
    break;
    case 11:
    {
        game_status_selector->AddChildren(gimbal_limited_action);
    }
    break;
    case 12:
    {
        game_status_selector->AddChildren(hunt_master_action);
        break;
    }
    break;
    case 13:
    {
        game_status_selector->AddChildren(hunt_slave_action);
    }
    break;
    case 14:
    {
        game_status_selector->AddChildren(search_action);
    }
    break;
    case 15:
    {
        game_status_selector->AddChildren(shoot_action);
    }
    break;
    case 16:
    {
        game_status_selector->AddChildren(turn_defend_action);
    }
    break;
    }

    while (ros::ok())
    {
        //
        //
        ros::spinOnce();
        root.Run(black_ptr);
        rate.sleep();
    }
    return 0;
}
