#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "traj_utils/polynomial_traj.h"
#include <math.h>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace std;

quadrotor_msgs::PositionCommand goal;
nav_msgs::Odometry Odom;
PolynomialTraj Poly_traj;
quadrotor_msgs::PolynomialTrajectory poly_pub_topic;
ros::Publisher Poly_coef_pub;
geometry_msgs::PoseStamped rviz_goal;
Vector3d cur_vel, cur_acc;
bool get_cmd = false;

void pos_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg){

    get_cmd = true;
    
    cur_vel(0) = msg->velocity.x;
    cur_vel(1) = msg->velocity.y;
    cur_vel(2) = msg->velocity.z;

    // cur_acc(0) = msg->acceleration.x;
    // cur_acc(1) = msg->acceleration.y;
    // cur_acc(2) = msg->acceleration.z;
}

void minimum_snap(const Vector3d tag_pos)
{
    Vector3d det_pos, det_vel, det_acc;
    Vector3d sta_pos, sta_vel, sta_acc;

    if (get_cmd){
        sta_vel = cur_vel;
        // sta_acc = cur_acc;
    } else {
        sta_vel << 0, 0, 0;
    }
    sta_acc << 0, 0, 0;

    sta_pos(0) = Odom.pose.pose.position.x;
    sta_pos(1) = Odom.pose.pose.position.y;
    sta_pos(2) = Odom.pose.pose.position.z;

    double a, b, c, d, e;
    double T = 0;
    double min_J = 1000000, J;

    // get optimal T
    det_pos = tag_pos - sta_pos;
    det_vel = - sta_vel;
    det_acc = - sta_acc;

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
        ROS_INFO("[poly_gen] No Path!");
        return;
    }
    else
    {
        ROS_INFO("[poly_gen] T=%.2f", T);
    }

    // get acc_input
    Vector3d A, B, C;
    double x, y, z;

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

    poly_pub_topic.coef_x.clear();
    poly_pub_topic.coef_y.clear();
    poly_pub_topic.coef_z.clear();
    poly_pub_topic.time.clear();
    poly_pub_topic.trajectory_id += 1;

    poly_pub_topic.header.stamp = ros::Time::now();
    poly_pub_topic.header.frame_id = "world";

    MatrixXd coef(3, poly_pub_topic.num_order+1);

    for(int i=0; i<3; i++){
        coef(i, 0) = A(i)/120;
        coef(i, 1) = B(i)/24;
        coef(i, 2) = C(i)/6;
        coef(i, 3) = acc(i);
        coef(i, 4) = vel(i);
        coef(i, 5) = pos(i);
    }

    for(int j=0; j<poly_pub_topic.num_order+1; j++){
        poly_pub_topic.coef_x.push_back(coef(0, j));
        poly_pub_topic.coef_y.push_back(coef(1, j));
        poly_pub_topic.coef_z.push_back(coef(2, j));
    }
    poly_pub_topic.time.push_back(T);

    Poly_coef_pub.publish(poly_pub_topic);

    // jerk_input = A * t * t / 2 + B * t + C;
    // Position.push_back(pos);
    // Velocity.push_back(vel);
    // Acceleration.push_back(acc);


    // t = t + dt;
    // while (t < T)
    // {
    //     jerk_input = A * t * t / 2 + B * t + C;

    //     // pos(0) = pos(0) + vel(0) * dt + acc(0) * pow(dt,2) / 2;
    //     // pos(1) = pos(1) + vel(1) * dt + acc(1) * pow(dt,2) / 2;
    //     // pos(2) = pos(2) + vel(2) * dt + acc(2) * pow(dt,2) / 2;
    //     // vel(0) = vel(0) + acc(0) * dt;
    //     // vel(1) = vel(1) + acc(1) * dt;
    //     // vel(2) = vel(2) + acc(2) * dt;
        
    //     for (int i = 0; i < 3; i++)
    //     {
    //         pos(i) = pos(i) + vel(i) * dt + acc(i) * dt * dt / 2 + jerk_input(i) * dt * dt * dt / 6;
    //         vel(i) = vel(i) + acc(i) * dt + jerk_input(i) * dt * dt / 2;
    //         acc(i) = acc(i) + jerk_input(i) * dt;
    //     }

    //     Position.push_back(pos);
    //     Velocity.push_back(vel);
    //     Acceleration.push_back(acc);
    //     Jerk.push_back(jerk_input);
    //     t = t + dt;
    // }

}

void goal_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    goal = *msg;
    static Vector3d tag_pos;
    if (tag_pos(0) == goal.position.x && tag_pos(1) == goal.position.y && tag_pos(2) == goal.position.z)
    {
        return;
    }
    else
    {
        tag_pos << goal.position.x, goal.position.y, goal.position.z;
        
        ROS_INFO("[poly_gen] Get goal! (%.2f, %.2f, %.2f)", tag_pos(0), tag_pos(1), tag_pos(2));
        minimum_snap(tag_pos);
    }
}

void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    rviz_goal = *msg;
    static Vector3d tag_pos;

    tag_pos << rviz_goal.pose.position.x, rviz_goal.pose.position.y, rviz_goal.pose.position.z;
    ROS_INFO("[poly_gen] Get rviz goal! (%.2f, %.2f, %.2f)", tag_pos(0), tag_pos(1), tag_pos(2));

    minimum_snap(tag_pos);
}

void Odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{   
    Odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "poly_traj_generator");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    Poly_coef_pub = nh.advertise<quadrotor_msgs::PolynomialTrajectory>("/planning/poly_coefs", 10);
    
    ros::Subscriber Odom_sub = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 10, Odom_cb);
    ros::Subscriber Goal_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/goal_cmd", 10, goal_cmd_cb);
    ros::Subscriber Goal_rviz_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, rviz_goal_cb);
    ros::Subscriber Position_cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 10, pos_cmd_cb);

    poly_pub_topic.trajectory_id = 0;
    poly_pub_topic.num_order = 5;
    poly_pub_topic.num_segment = 0;
    poly_pub_topic.start_yaw = 0;
    poly_pub_topic.final_yaw = 0;
    poly_pub_topic.mag_coeff = 0;
    poly_pub_topic.order.push_back(0);

    ROS_INFO("[poly_gen] Ready fot planning");

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}