#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "geometry_msgs/PoseStamped.h"
#include <math.h>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

nav_msgs::Odometry Odom;
quadrotor_msgs::PositionCommand goal, pub_cmd, rcv_cmd;
ros::Publisher Pose_cmd_pub;
geometry_msgs::PoseStamped rviz_goal;
int ctrl_level;
enum CTRL {VEL=1, ACC, JERK, ELSE};

void publish_cmd(vector<Vector3d> Pos, vector<Vector3d> Vel)
{
    ROS_INFO("[snap] Pub cmd!");
    int size = Pos.size();
    for (int i = 0; i < size; i++)
    {
        Vector3d P = Pos.front();
        pub_cmd.position.x = P(0);
        pub_cmd.position.y = P(1);
        pub_cmd.position.z = P(2);
        Vector3d V = Vel.front();
        pub_cmd.velocity.x = V(0);
        pub_cmd.velocity.y = V(1);
        pub_cmd.velocity.z = V(2);
        Pos.erase(Pos.begin());
        Vel.erase(Vel.begin());
        Pose_cmd_pub.publish(pub_cmd);
    }
    ROS_INFO("[snap] Pub succeed!");
}

void publish_cmd(vector<Vector3d> Pos, vector<Vector3d> Vel, vector<Vector3d> Acc)
{
    ROS_INFO("[snap] Pub cmd!");
    int size = Pos.size();
    for (int i = 0; i < size; i++)
    {
        Vector3d P = Pos.front();
        pub_cmd.position.x = P(0);
        pub_cmd.position.y = P(1);
        pub_cmd.position.z = P(2);
        Vector3d V = Vel.front();
        pub_cmd.velocity.x = V(0);
        pub_cmd.velocity.y = V(1);
        pub_cmd.velocity.z = V(2);
        Vector3d A = Acc.front();
        pub_cmd.acceleration.x = A(0);
        pub_cmd.acceleration.y = A(1);
        pub_cmd.acceleration.z = A(2);
        Pos.erase(Pos.begin());
        Vel.erase(Vel.begin());
        Acc.erase(Acc.begin());
        Pose_cmd_pub.publish(pub_cmd);
    }
    ROS_INFO("[snap] Pub succeed!");
}

void publish_cmd(vector<Vector3d> Pos, vector<Vector3d> Vel, vector<Vector3d> Acc, vector<Vector3d> Jerk)
{
    ROS_INFO("[snap] Pub cmd!");
    int size = Pos.size();
    for (int i = 0; i < size; i++)
    {
        Vector3d P = Pos.front();
        pub_cmd.position.x = P(0);
        pub_cmd.position.y = P(1);
        pub_cmd.position.z = P(2);
        Vector3d V = Vel.front();
        pub_cmd.velocity.x = V(0);
        pub_cmd.velocity.y = V(1);
        pub_cmd.velocity.z = V(2);
        Vector3d A = Acc.front();
        pub_cmd.acceleration.x = A(0);
        pub_cmd.acceleration.y = A(1);
        pub_cmd.acceleration.z = A(2);
        Vector3d J = Jerk.front();
        pub_cmd.jerk.x = J(0);
        pub_cmd.jerk.y = J(1);
        pub_cmd.jerk.z = J(2);
        Pos.erase(Pos.begin());
        Vel.erase(Vel.begin());
        Acc.erase(Acc.begin());
        Jerk.erase(Jerk.begin());
        Pose_cmd_pub.publish(pub_cmd);
    }
    ROS_INFO("[snap] Pub succeed!");
}

void minimum_snap(const Vector3d sta_pos, const Vector3d sta_vel, const Vector3d sta_acc, const Vector3d tag_pos)
{
    Vector3d det_pos, det_vel, det_acc;
    double a, b, c, d, e;
    double T = 0;
    double dt = 0.01, t = 0;
    double min_J = 1000000, J;

    // get optimal T
    det_pos = tag_pos - sta_pos;
    det_vel = -sta_vel;
    det_acc = -sta_acc;

    /* minimum acc */
    // a =   4 * (det_vel(0) * det_vel(0) + det_vel(1) * det_vel(1) + det_vel(2) * det_vel(2))
    //    + 12 * (det_vel(0) * sta_vel(0) + det_vel(1) * sta_vel(1) + det_vel(2) * sta_vel(2))
    //    + 12 * (sta_vel(0) * sta_vel(0) + sta_vel(1) * sta_vel(1) + sta_vel(2) * sta_vel(2));
    // b =- 12 * (det_pos(0) * det_vel(0) + det_pos(1) * det_vel(1) + det_pos(2) * det_vel(2))
    //    - 24 * (det_pos(0) * sta_vel(0) + det_pos(1) * sta_vel(1) + det_pos(2) * sta_vel(2));
    // c =  12 * (det_pos(0) * det_pos(0) + det_pos(1) * det_pos(1) + det_pos(2) * det_pos(2));

    /* minimum jerk */
    a = 0;
    b = 0;
    c = 0;
    d = 0;
    e = 0;
    for (int i = 0; i < 3; i++)
    {
        a = a + 12 * sta_acc(i) * sta_acc(i) + 12 * det_acc(i) * sta_acc(i) + 9 * det_acc(i) * det_acc(i);
        b = b - 24 * sta_acc(i) * det_vel(i) - 72 * det_vel(i) * det_acc(i) - 120 * det_acc(i) * sta_vel(i);
        c = c + 120 * det_acc(i) * det_pos(i) + 192 * det_vel(i) * det_vel(i) + 720 * det_vel(i) * sta_vel(i) + 720 * sta_vel(i) * sta_vel(i);
        d = d - 720 * det_pos(i) * det_vel(i) - 1440 * det_pos(i) * sta_vel(i);
        e = e + 720 * det_pos(i) * det_pos(i);
    }

    for (double k = 0.01; k < 10; k = k + 0.01)
    {
        // J = k + a/k + b/k/k + c/k/k/k;
        J = k + a / k + b / k / k + c / k / k / k + d / k / k / k / k + e / k / k / k / k / k;
        if (J < min_J)
        {
            T = k;
            min_J = J;
        }
    }
    if (T == 0)
    {
        ROS_INFO("[snap] No Path!");
        return;
    }
    else
    {
        ROS_INFO("[snap] T=%.2f", T);
    }

    // get acc_input
    Vector3d A, B, C;
    double x, y, z;

    /* minimum acc */
    // A(0) = 6 *  det_vel(0) /T/T - 12 * ( det_pos(0) - T * det_vel(0) ) /T/T/T;
    // A(1) = 6 *  det_vel(1) /T/T - 12 * ( det_pos(1) - T * det_vel(1) ) /T/T/T;
    // A(2) = 6 *  det_vel(2) /T/T - 12 * ( det_pos(2) - T * det_vel(2) ) /T/T/T;
    // B(0) = 6 * ( det_pos(0) - T * det_vel(0) ) /T/T - 2 * det_vel(0) / T;
    // B(1) = 6 * ( det_pos(1) - T * det_vel(1) ) /T/T - 2 * det_vel(1) / T;
    // B(2) = 6 * ( det_pos(2) - T * det_vel(2) ) /T/T - 2 * det_vel(2) / T;

    /* minimum jerk */
    for (int i = 0; i < 3; i++)
    {
        x = det_acc(i);
        y = sta_acc(i) * T * T / 2 + sta_vel(i) * T - det_pos(i);
        z = det_vel(i) - T * sta_acc(i);

        A(i) = 60 * x / T / T / T - 720 * y / T / T / T / T / T - 360 * z / T / T / T / T;
        B(i) = -24 * x / T / T + 360 * y / T / T / T / T + 168 * z / T / T / T;
        C(i) = 3 * x / T - 60 * y / T / T / T - 24 * z / T / T;
    }

    Vector3d jerk_input;
    Vector3d pos, vel, acc;
    vector<Vector3d> Position;
    vector<Vector3d> Velocity;
    vector<Vector3d> Acceleration;
    vector<Vector3d> Jerk;
    pos = sta_pos;
    vel = sta_vel;
    acc = sta_acc;

    jerk_input = A * t * t / 2 + B * t + C;
    Position.push_back(pos);
    Velocity.push_back(vel);
    Acceleration.push_back(acc);
    t = t + dt;
    while (t < T)
    {
        jerk_input = A * t * t / 2 + B * t + C;

        // pos(0) = pos(0) + vel(0) * dt + acc(0) * pow(dt,2) / 2;
        // pos(1) = pos(1) + vel(1) * dt + acc(1) * pow(dt,2) / 2;
        // pos(2) = pos(2) + vel(2) * dt + acc(2) * pow(dt,2) / 2;
        // vel(0) = vel(0) + acc(0) * dt;
        // vel(1) = vel(1) + acc(1) * dt;
        // vel(2) = vel(2) + acc(2) * dt;
        
        for (int i = 0; i < 3; i++)
        {
            pos(i) = pos(i) + vel(i) * dt + acc(i) * dt * dt / 2 + jerk_input(i) * dt * dt * dt / 6;
            vel(i) = vel(i) + acc(i) * dt + jerk_input(i) * dt * dt / 2;
            acc(i) = acc(i) + jerk_input(i) * dt;
        }

        Position.push_back(pos);
        Velocity.push_back(vel);
        Acceleration.push_back(acc);
        Jerk.push_back(jerk_input);
        t = t + dt;
    }

    switch (ctrl_level)
    {
    case VEL:
        publish_cmd(Position, Velocity);
        break;
    case ACC:
        publish_cmd(Position, Velocity, Acceleration);
        break;
    case JERK:
        publish_cmd(Position, Velocity, Acceleration, Jerk);
        break;
    case ELSE:
        
        break;
    }
    // publish_cmd(Position, Velocity, Acceleration, Jerk);
    // publish_cmd(Position, Velocity, Acceleration);
}

void goal_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    goal = *msg;
    static Vector3d tag_pos;
    Vector3d start_pos, start_vel, start_acc;
    if (tag_pos(0) == goal.position.x && tag_pos(1) == goal.position.y && tag_pos(2) == goal.position.z)
    {
        return;
    }
    else
    {
        tag_pos << goal.position.x, goal.position.y, goal.position.z;
        start_pos << Odom.pose.pose.position.x, Odom.pose.pose.position.y, Odom.pose.pose.position.z;
        start_vel << 0, 0, 0;
        start_acc << 0, 0, 0;
        ROS_INFO("[snap] Get goal! (%.2f, %.2f, %.2f)", tag_pos(0), tag_pos(1), tag_pos(2));
        minimum_snap(start_pos, start_vel, start_acc, tag_pos);
    }
}

void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    rviz_goal = *msg;
    static Vector3d tag_pos;
    Vector3d start_pos, start_vel, start_acc;

    tag_pos << rviz_goal.pose.position.x, rviz_goal.pose.position.y, rviz_goal.pose.position.z;
    start_pos << Odom.pose.pose.position.x, Odom.pose.pose.position.y, Odom.pose.pose.position.z;
    start_vel << 0, 0, 0;
    start_acc << 0, 0, 0;
    ROS_INFO("[snap] Get rviz goal! (%.2f, %.2f, %.2f)", tag_pos(0), tag_pos(1), tag_pos(2));

    minimum_snap(start_pos, start_vel, start_acc, tag_pos);
}

void Odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    Odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "minimum_snap");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    Pose_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10);
    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, Odom_cb);
    ros::Subscriber Goal_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/goal_cmd", 10, goal_cmd_cb);
    ros::Subscriber Goal_rviz_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, rviz_goal_cb);
    // ros::Subscriber Rcv_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
    //     ("/planning/pos_cmd",  10, rcv_cmd_cb );

    ros::param::get("/min_snap/ctrl_level", ctrl_level);
    ROS_INFO("%d, %d, %d, %d, %d",VEL,ACC,JERK,ELSE,ctrl_level);
    if (ctrl_level != VEL && ctrl_level != ACC && ctrl_level != JERK) {
        ctrl_level = ELSE;
        ROS_WARN("[snap] Param /min_snap/ctrl_level is illegal!");
    } 

    ROS_INFO("[snap] Ready fot planning");
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}