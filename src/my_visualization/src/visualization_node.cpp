#include <ros/ros.h>
#include <my_visualization/plan_visual.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"

using namespace my_planner;
my_planner::PlanVisual::Ptr visual;
nav_msgs::Odometry odom;

void goal_visual_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    Eigen::Vector3d goalPoint;
    goalPoint << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

    visual->displayGoalPoint(goalPoint, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);
}

void pos_cmd_visual_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    Eigen::Vector3d start, end;
    start(0) = odom.pose.pose.position.x;
    start(1) = odom.pose.pose.position.y;
    start(2) = odom.pose.pose.position.z;

    end(0) = start(0) + msg->velocity.x;
    end(1) = start(1) + msg->velocity.y;
    end(2) = start(2) + msg->velocity.z;

    visual->displayArrow(start, end, Eigen::Vector4d(0.3, 0.7, 0, 1), 0);

    end(0) = start(0) + msg->acceleration.x;
    end(1) = start(1) + msg->acceleration.y;
    end(2) = start(2) + msg->acceleration.z;

    visual->displayArrow(start, end, Eigen::Vector4d(0.3, 0, 0.7, 1), 1);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualization_node");
    ros::NodeHandle nh("~");

    ros::Rate rate(100.0);

    visual.reset(new my_planner::PlanVisual(nh));

    ros::Subscriber Goal_cmd_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal_point1", 10, goal_visual_cb);
    ros::Subscriber Goal_rviz_sub = nh.subscribe<geometry_msgs::PoseStamped>("/goal_point2", 10, goal_visual_cb);
    ros::Subscriber Pos_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 10, pos_cmd_visual_cb);
    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>("/odometry", 10, odom_cb);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}