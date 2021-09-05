#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

ros::Publisher goal_list_pub;
ros::Subscriber rviz_goal_sub;
int id = 0;
geometry_msgs::Pose goal_pt;
geometry_msgs::PoseArray goal_list;

void rviz_goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    goal_pt = msg->pose;
    if (goal_pt.position.z < 0){
        goal_pt.position.z = 0;
        goal_list.poses.push_back(goal_pt);
        goal_list.header.stamp = ros::Time::now();
        goal_list.header.frame_id = "world";
        goal_list.header.seq = id ++;
        goal_list_pub.publish(goal_list);
    } else {
        goal_list.poses.push_back(goal_pt);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "get_waypoint");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    rviz_goal_sub = nh.subscribe("/rviz_goal", 10, rviz_goal_cb);
    goal_list_pub = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}