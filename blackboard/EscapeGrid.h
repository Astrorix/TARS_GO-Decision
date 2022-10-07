#ifndef ROBORTS_DECISION_ESCAPEGRID_H
#define ROBORTS_DECISION_ESCAPEGRID_H

#include <fstream>
#include "blackboard.h"

namespace roborts_decision
{
    struct Point2D
    {
        double x;
        double y;
    };

    struct PointValue
    {
        Point2D point;
        double value = 0;
    };

    class HighObstacles
    {
    public:
        HighObstacles(float x1, float x2, float y1, float y2) : minx(x1), maxx(x2), miny(y1), maxy(y2) {}

        bool JudgeIN(float x, float y)
        {
            if (x >= minx && x <= maxx && y >= miny && y <= maxy)
                return true;
            return false;
        }

        float minx;
        float maxx;
        float miny;
        float maxy;
    };

    class EscapeGrid
    {
    private:
        Blackboard::Ptr blackboard_ptr;
        const int LENGTHUNIT;
        const int WIDTHUNIT;
        double discrete_unit;
        std::vector<std::vector<PointValue>> escapeGrid;

    public:
        EscapeGrid(const Blackboard::Ptr &blackboard_ptr, double discreteUnit) : blackboard_ptr(blackboard_ptr), discrete_unit(discreteUnit),
                                                                                 LENGTHUNIT(static_cast<const int>(8.0 / discreteUnit)), WIDTHUNIT(static_cast<const int>(4.0 / discreteUnit)),
                                                                                 escapeGrid(WIDTHUNIT, std::vector<PointValue>(LENGTHUNIT))
        {
            //     下面对矩阵进行赋值
            //     --------------
            //     ||||||||||||||
            //     ||||||||||||||  ++
            //     --------------  rol
            //     col++ ->

            double lineWidth = discrete_unit / 2;

            for (int i = 0; i < escapeGrid.size(); i++)
            {
                double columnWidth = discrete_unit / 2;

                for (int j = 0; j < escapeGrid.at(0).size(); j++)
                {
                    escapeGrid.at(i).at(j) = {{columnWidth, lineWidth}, 0};
                    columnWidth += discrete_unit;
                }
                lineWidth += discrete_unit;
            }
        };

        void UpdateEscapeGrid()
        {
            // 势场包括 4*障碍物信息的正势场 + 5*敌方车辆的负势场（距离、子弹、血量）
            // 1*buff的正负增益 + 3*双车时队友的负势场 + 4*自己到安全点的距离的正势场
            // 选取两个点   一个是地方的位置  另一个是我要选取的点  遍历 1000
            // 障碍物的位置是固定的 我可以 初始化出所有障碍物在示例图中的位置  这样的话
            // 取敌人位置  与敌人位置之间有障碍物的  优先选择
            /* 
            1.取敌人位置
            2.遍历势场   
                
                做一个多边形类 放入高障碍物的位置坐标 judgeOB 的时候 在遍历的时候 看这个点在不在类里就行
                
                minx < x < maxx 
                miny < y< maxy
                
                ->如果势场和敌人之间有障碍物 加分  没有则 不加
                
                ->对每一个点距离敌方的位置 血量 子弹数距离进行赋值  （80%距离、5%敌方血量、15%子弹数）
                
                ->如果该点在buff点周围 根据buff的种类进行加分和减分    
           
            3.建一个函数 传进自身坐标和查找半径 找到分数最大的点 输出位置
            4.再对势场进行改变 对第一辆车选择的位置进行减分 后调用函数选择另一辆车的位置
            */

            std::vector<HighObstacles> highobstacles;
            highobstacles.push_back(HighObstacles(1.5, 1.7, 0, 1));
            highobstacles.push_back(HighObstacles(6.38, 6.58, 3.48, 4.48));
            highobstacles.push_back(HighObstacles(7.08, 8.08, 1, 1.2));
            highobstacles.push_back(HighObstacles(0, 1, 3.28, 3.48));
            highobstacles.push_back(HighObstacles(3.54, 4.54, 0.953, 1.153));
            highobstacles.push_back(HighObstacles(3.54, 4.54, 3.327, 3.527));
            for (int i = 0; i < escapeGrid.size(); i++)
            {
                for (int j = 0; j < escapeGrid.at(0).size(); j++)
                {
                    escapeGrid.at(i).at(j).value = 0;

                    float path_x = escapeGrid.at(i).at(j).point.x;
                    float path_y = escapeGrid.at(i).at(j).point.y;

                    geometry_msgs::PoseStamped point;
                    point.pose.position.x = path_x;
                    point.pose.position.y = path_y;

                    float distance1 = blackboard_ptr->GetDistance(point, blackboard_ptr->enemyPose1);

                    float distance2 = blackboard_ptr->GetDistance(point, blackboard_ptr->enemyPose2);

                    float tan1 = (blackboard_ptr->enemypose1_y - path_y) / (blackboard_ptr->enemypose1_x - path_x);

                    for (int h = 0; h < int(distance1 * 10); h++)
                    {
                        path_x = path_x + 0.1;
                        path_y = path_y + 0.1 * tan1;
                        for (int k = 0; k < highobstacles.size(); k++)
                            {
                                if (highobstacles[k].JudgeIN(path_x, path_y))
                                {
                                    escapeGrid.at(i).at(j).value += 2.0 / 17;
                                }
                            }
                    }

                    path_x = escapeGrid.at(i).at(j).point.x;
                    path_y = escapeGrid.at(i).at(j).point.y;

                    float tan2 = (blackboard_ptr->enemypose2_y - path_y) / (blackboard_ptr->enemypose2_x - path_y);

                    for (int h = 0; h < int(distance2 * 10); h++)
                    {
                        path_x += 0.1;
                        path_y += 0.1 * tan2;
                        for (int k = 0; k < highobstacles.size(); k++)
                            {
                                if (highobstacles[k].JudgeIN(path_x, path_y))
                                {
                                    escapeGrid.at(i).at(j).value += 2.0 / 17;
                                }
                            }
                    }
                    // 血量 子弹数距离进行赋值  （80%距离、5%敌方血量、15%子弹数）
                    double enemySumValue = (-(distance1 + distance2) / 8.9 / 2 * 0.8 + (blackboard_ptr->enemy1_hp_ + blackboard_ptr->enemy2_hp_) / 4000.0 * 0.05 + (blackboard_ptr->enemy1_bullet_ + blackboard_ptr->enemy2_bullet_) / 400.0 * 0.15) * 5 / 17;

                    escapeGrid.at(i).at(j).value -= enemySumValue;

                    //如果该点在buff点周围 根据buff的种类进行加分和减分
                    //blood
                    HighObstacles bloodPose(blackboard_ptr->blood_pose_.pose.position.x - 0.3, blackboard_ptr->blood_pose_.pose.position.x + 0.3,
                                            blackboard_ptr->blood_pose_.pose.position.y - 0.3, blackboard_ptr->blood_pose_.pose.position.y + 0.3);
                    if (blackboard_ptr->blood_active_ && bloodPose.JudgeIN(escapeGrid.at(i).at(j).point.x, escapeGrid.at(i).at(j).point.y))
                    {
                        escapeGrid.at(i).at(j).value += 1.0 / 17;
                    }

                    //bullet
                    HighObstacles bulletPose(blackboard_ptr->bullet_pose_.pose.position.x - 0.3, blackboard_ptr->bullet_pose_.pose.position.x + 0.3,
                                             blackboard_ptr->bullet_pose_.pose.position.y - 0.3, blackboard_ptr->bullet_pose_.pose.position.y + 0.3);
                    if (blackboard_ptr->bullet_active_ && bulletPose.JudgeIN(escapeGrid.at(i).at(j).point.x, escapeGrid.at(i).at(j).point.y))
                    {
                        escapeGrid.at(i).at(j).value += 1.0 / 17;
                    }
                    //debuff
                    HighObstacles debuffPose1(blackboard_ptr->debuff1_pose_.pose.position.x - 0.3, blackboard_ptr->debuff1_pose_.pose.position.x + 0.3, blackboard_ptr->debuff1_pose_.pose.position.y - 0.3, blackboard_ptr->debuff1_pose_.pose.position.y + 0.3);
                    HighObstacles debuffPose2(blackboard_ptr->debuff2_pose_.pose.position.x - 0.3, blackboard_ptr->debuff2_pose_.pose.position.x + 0.3, blackboard_ptr->debuff2_pose_.pose.position.y - 0.3, blackboard_ptr->debuff2_pose_.pose.position.y + 0.3);
                    if (blackboard_ptr->debuff1_active_ && debuffPose1.JudgeIN(escapeGrid.at(i).at(j).point.x, escapeGrid.at(i).at(j).point.y))
                    {
                        escapeGrid.at(i).at(j).value -= 1.0 / 17;
                    }
                    if (blackboard_ptr->debuff2_active_ && debuffPose2.JudgeIN(escapeGrid.at(i).at(j).point.x, escapeGrid.at(i).at(j).point.y))
                    {
                        escapeGrid.at(i).at(j).value -= 1.0 / 17;
                    }
                }
            }

            std::ofstream gridData("/home/dji/RMGUI/src/RoboRTS-icra2019/roborts_decision/blackboard/Grid.data");
            if (gridData.is_open())
            {
                ROS_INFO("Open");
                gridData << escapeGrid.size() << std::endl;
                gridData << escapeGrid.at(0).size() << std::endl;
                gridData << discrete_unit << std::endl;
                for (int i = 0; i < escapeGrid.size(); i++)
                {
                    for (int j = 0; j < escapeGrid.at(0).size(); j++)
                    {
                        gridData << escapeGrid.at(i).at(j).value << std::endl;
                    }
                }
                gridData.close();
            }
            else ROS_ERROR("Error");
        }

        geometry_msgs::PoseStamped GetEscapePose(geometry_msgs::PoseStamped pose)
        {
            int rangeMultiples = 5; // 单次寻找的范围

            double x = pose.pose.position.x, y = pose.pose.position.y;
            int index_x = int(x / discrete_unit);
            int index_y = int(y / discrete_unit);
            double score = -10000;

            PointValue safe_point = {{-1, -1}, score}; //先声明一个特别小的值
            while (true)
            {
                for (int i = index_x - rangeMultiples; i < index_x + rangeMultiples; i++)
                {
                    for (int j = index_y - rangeMultiples; j < index_y + rangeMultiples; j++)
                    {
                        if (i >= 0 && i <= escapeGrid.size() && j >= 0 && j <= escapeGrid.at(0).size())
                        {
                            if (safe_point.value < escapeGrid.at(i).at(j).value)
                            {
                                safe_point = escapeGrid.at(i).at(j);
                            }
                        }
                    }
                }
                if (safe_point.value != -10000 || rangeMultiples > 4.04 / discrete_unit)
                    break;
                rangeMultiples++;
            }
            geometry_msgs::PoseStamped escapePose;
            escapePose.pose.position.x = safe_point.point.x;
            escapePose.pose.position.y = safe_point.point.y;
            index_x = int(safe_point.point.x / discrete_unit);
            index_y = int(safe_point.point.y / discrete_unit);

            for (int i = index_x - 2; i < index_x + 2; i++)
            {
                for (int j = index_y - 2; j < index_y + 2; j++)
                {
                    if (i >= 0 && i <= escapeGrid.size() && j >= 0 && j <= escapeGrid.at(0).size())
                    {
                        escapeGrid.at(i).at(j).value = -1000;
                    }
                }
            }
            return escapePose;
        }
    };
};
#endif