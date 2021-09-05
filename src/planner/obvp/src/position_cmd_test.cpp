#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <math.h>
#include <string>

/*
void Odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
void rc_in_cb(const mavros_msgs::RCIn::ConstPtr& rc_msg);
void servo_stat_cb(const std_msgs::String::ConstPtr& msg);
*/

nav_msgs::Odometry estimateOdom;
double DistanceThreshold;
// bool servo_busy = false;
double Control_Distance_Threshold;
double SPEED;
double wait_time;
double ACC_max;
double dt;

// publish the commanded local position
geometry_msgs::PoseStamped goal_point;
ros::Publisher Pose_cmd_pub;
ros::Publisher servo_cmd_pub;
ros::Publisher rg_pub;

// callback: save current state
void Odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    estimateOdom = *msg;
}

inline float getDistance(const geometry_msgs::Point &curPose,
                         const geometry_msgs::Point &targetPose)
{
    return sqrt(pow(curPose.x - targetPose.x, 2) +
                pow(curPose.y - targetPose.y, 2) +
                pow(curPose.z - targetPose.z, 2));
}

geometry_msgs::Point getPoint(const float &x, const float &y,
                              const float &z)
{
    geometry_msgs::Point res;
    res.x = x;
    res.y = y;
    res.z = z;
    return res;
}

quadrotor_msgs::PositionCommand get_cmd_from_target_pos(geometry_msgs::Point target_pos)
{
    quadrotor_msgs::PositionCommand res;
    float cur_x = estimateOdom.pose.pose.position.x;
    float cur_y = estimateOdom.pose.pose.position.y;
    float tag_x = target_pos.x;
    float tag_y = target_pos.y;

    const float dis = sqrt(pow(cur_x - tag_x, 2) + pow(cur_y - tag_y, 2));

    float K = dis / Control_Distance_Threshold;
    if (K > 1)
    {
        K = 1;
    }

    const float dx = target_pos.x - estimateOdom.pose.pose.position.x,
                dy = target_pos.y - estimateOdom.pose.pose.position.y,
                dz = target_pos.z - estimateOdom.pose.pose.position.z;
    res.position.x = estimateOdom.pose.pose.position.x + dx / dis * K * SPEED * dt;
    res.position.y = estimateOdom.pose.pose.position.y + dy / dis * K * SPEED * dt;
    res.position.z = estimateOdom.pose.pose.position.z + dz / dis * K * SPEED * dt;
    // res.position.z = estimateOdom.pose.pose.position.z - SPEEDZ * dt;
    res.velocity.x = dx / dis * K * SPEED;
    res.velocity.y = dy / dis * K * SPEED;
    res.velocity.z = dz / dis * K * SPEED;
    // res.velocity.z = - SPEEDZ;
    return res;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_cmd_test");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    // publish the commanded local position
    Pose_cmd_pub = nh.advertise<geometry_msgs::PoseStamped>("/planning/goal_cmd", 10);
    // servo_cmd_pub = nh.advertise<std_msgs::String>
    //     ("servo_command", 10);

    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, Odom_cb);
    // ros::Subscriber servo_info_sub = nh.subscribe<std_msgs::String>
    //     ("servo_stat", 10, servo_stat_cb);

    // the setpoint publishing rate MUST be faster than 2Hz
    nh.param("/position_cmd_test/DistanceThreshold", DistanceThreshold, 0.1);
    nh.param("/position_cmd_test/Control_Distance_Threshold", Control_Distance_Threshold, 0.2);
    nh.param("/position_cmd_test/SPEED", SPEED, 1.0);
    nh.param("/position_cmd_test/wait_time", wait_time, 5.0);
    nh.param("/position_cmd_test/ACC_max", ACC_max, 0.1);
    nh.param("/position_cmd_test/dt", dt, 0.05);

    quadrotor_msgs::PositionCommand cmd, real_cmd;
    std_msgs::Bool canStart;
    canStart.data = true;

    const float AX = 3, AY = 0, BX = 3.5, BY = 0, CX = 3.5, CY = 1, DX = -0.5, DY = 1, EX = -0.5, EY = 0, HX = 0, HY = 0;
    const float Height = 1.0;

    // Target of the Key Points
    // Remember to CHANGE TargetPointCount if change the points number
    const int TargetPointCount = 7;
    const geometry_msgs::Point targetPoints[TargetPointCount] = {
        getPoint(HX, HY, Height),
        getPoint(AX, AY, Height),
        getPoint(BX, BY, Height),
        getPoint(CX, CY, Height),
        getPoint(DX, DY, Height),
        getPoint(EX, EY, Height),
        getPoint(HX, HY, Height),
    };

    int currentWaypoint = 0;

    cmd.position = targetPoints[currentWaypoint];

    ros::Duration(wait_time).sleep();

    bool haveEverReached = false;
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        float dis = getDistance(estimateOdom.pose.pose.position, cmd.position);

        if (dis < DistanceThreshold)
        {
            haveEverReached = true;
            ROS_INFO("[test_goal] Reached Point%d (%.2f,%.2f,%.2f)", currentWaypoint, estimateOdom.pose.pose.position.x, estimateOdom.pose.pose.position.y, estimateOdom.pose.pose.position.z);
            // ROS_INFO("[test_goal] Time: %f  dis: %f",ros::Time::now()-last_request,dis);

            currentWaypoint++;

            if (currentWaypoint >= TargetPointCount)
            { // reachedHeight
                break;
            }

            haveEverReached = false;
            ROS_INFO("[test_goal] Heading to Point%d (%.2f,%.2f,%.2f)", currentWaypoint, targetPoints[currentWaypoint].x, targetPoints[currentWaypoint].y, targetPoints[currentWaypoint].z);
        }
        else
        {
            if (haveEverReached == false)
            {
                ROS_INFO("[test_goal] dis=%.2f (%.2f,%.2f,%.2f)", dis, estimateOdom.pose.pose.position.x, estimateOdom.pose.pose.position.y, estimateOdom.pose.pose.position.z);
                last_request = ros::Time::now();
            }
        }

        cmd.position = targetPoints[currentWaypoint];
        // real_cmd = get_cmd_from_target_pos(targetPoints[currentWaypoint]);
        ROS_INFO("[test_goal] currentWaypoint: %d", currentWaypoint);
        goal_point.header = cmd.header;
        goal_point.pose.position = cmd.position;
        Pose_cmd_pub.publish(goal_point);

        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok())
    {
        // real_cmd = get_cmd_from_target_pos(estimateOdom.pose.pose.position);
        ROS_INFO("[test_goal] Waiting for landing");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
