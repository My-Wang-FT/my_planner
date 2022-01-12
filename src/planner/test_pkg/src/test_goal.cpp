#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

enum type
{
    End_Node = 0,
    Power_Loop,
    Barrel_Roll,
    Split_S,
};

const double PI=3.1415926;
int test_type = 0, id = 0;
double radius = 0;
double dir_x, dir_y, dis;
double center_x, center_y, center_z;
nav_msgs::Odometry odom;
geometry_msgs::Pose goal_pt;
geometry_msgs::PoseArray goal_list;

ros::Subscriber odom_sub;
ros::Subscriber trigger_sub;
ros::Publisher test_goal_pub;

void odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
}

void power_loop(void)
{
    double theta;
    for (int i = 0; i < 13; i++)
    {
        theta = PI / 2 * i;
        goal_pt.position.x = center_x + radius * sin(theta) * dir_x / dis;
        goal_pt.position.y = center_y + radius * sin(theta) * dir_y / dis;
        goal_pt.position.z = center_z - radius * cos(theta);
        goal_list.poses.push_back(goal_pt);
    }
}

void barrel_roll(void)
{
    double theta;
    for (int i = 0; i < 9; i++)
    {
        theta = PI / 2 * i;
        goal_pt.position.x = center_x + radius * sin(theta) * dir_y / dis;
        goal_pt.position.y = center_y - radius * sin(theta) * dir_x / dis;
        goal_pt.position.x += center_z * (i - 4) / 4 * dir_x / dis;
        goal_pt.position.y += center_z * (i - 4) / 4 * dir_y / dis;
        goal_pt.position.z = center_z - radius * cos(theta);
        goal_list.poses.push_back(goal_pt);
    }
}

void Split_s(void)
{
    goal_pt.position.x = center_x;
    goal_pt.position.y = center_y;
    goal_pt.position.z = center_z;
    goal_list.poses.push_back(goal_pt);
    for (int i = -1; i < 2; i += 2)
    {
        for (int j = -1; j < 2; j += 2)
        {
            goal_pt.position.x = center_x - 1.5 * radius * i * dir_x / dis;
            goal_pt.position.y = center_y - 1.5 * radius * i * dir_y / dis;
            goal_pt.position.z = center_z - radius * j;
            goal_list.poses.push_back(goal_pt);
        }
    }
    goal_pt.position.x = center_x;
    goal_pt.position.y = center_y;
    goal_pt.position.z = center_z;
    goal_list.poses.push_back(goal_pt);
}

void trigger_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    dir_x = msg->pose.position.x - odom.pose.pose.position.x;
    dir_y = msg->pose.position.y - odom.pose.pose.position.y;
    dis = sqrt(dir_x * dir_x + dir_y * dir_y);
    center_x = odom.pose.pose.position.x + dir_x / 2;
    center_y = odom.pose.pose.position.y + dir_y / 2;
    center_z = radius > 1 ? radius * 2 : 2;

    goal_pt.position.x = center_x - 2 * center_z / dis * dir_x;
    goal_pt.position.y = center_y - 2 * center_z / dis * dir_y;
    goal_pt.position.z = center_z;
    goal_list.poses.push_back(goal_pt);

    switch (test_type)
    {
    case Power_Loop:
        ROS_INFO("Power_Loop");
        power_loop();
        break;
    case Barrel_Roll:
        ROS_INFO("Barrel_Roll");
        barrel_roll();
        break;
    case Split_S:
        ROS_INFO("Split_S");
        Split_s();
        break;
    default:
        ROS_ERROR("Wrong Test Type");
    }

    goal_pt.position.x = center_x + 2 * center_z / dis * dir_x;
    goal_pt.position.y = center_y + 2 * center_z / dis * dir_y;
    goal_pt.position.z = center_z;
    goal_list.poses.push_back(goal_pt);

    goal_pt.position = msg->pose.position;
    goal_pt.position.z = center_z;
    goal_list.poses.push_back(goal_pt);
    goal_list.header.stamp = ros::Time::now();
    goal_list.header.frame_id = "world";
    goal_list.header.seq = id++;
    test_goal_pub.publish(goal_list);
    goal_list.poses.clear();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test_goal");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    odom_sub = nh.subscribe("/odom_topic", 10, odom_cb);
    trigger_sub = nh.subscribe("/rviz/2d_nav_goal", 1, trigger_cb);
    test_goal_pub = nh.advertise<geometry_msgs::PoseArray>("/test_goal_list", 10);

    ros::param::get("/test_goal/type", test_type);
    ros::param::get("/test_goal/radius", radius);

    if (test_type == End_Node)
    {
        ROS_INFO("End_Test_Node");
        return 0;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}