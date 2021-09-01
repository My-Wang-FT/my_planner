#include <my_visualization/plan_visual.h>

namespace my_planner
{
    PlanVisual::PlanVisual(ros::NodeHandle &node)
    {
        nh = node;

        goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
        arrow_pub = nh.advertise<visualization_msgs::Marker>("vector_dir", 2);
        traj_pub = nh.advertise<visualization_msgs::Marker>("poly_traj", 2);
        ROS_INFO("[Visual]: Init");
    }

    void PlanVisual::visualInit(ros::NodeHandle &node)
    {
        nh = node;

        goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 2);
        arrow_pub = nh.advertise<visualization_msgs::Marker>("vector_dir", 2);
        traj_pub = nh.advertise<visualization_msgs::Marker>("poly_traj", 2);

        ROS_INFO("[Visual]: Init");
    }

    void PlanVisual::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        sphere.pose.position.x = goal_point(0);
        sphere.pose.position.y = goal_point(1);
        sphere.pose.position.z = goal_point(2);
        sphere.lifetime = ros::Duration();
        goal_point_pub.publish(sphere);
    }

    void PlanVisual::displayArrow(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector4d color, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::ARROW;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        sphere.lifetime = ros::Duration();
        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = 0.1;
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.1;

        geometry_msgs::Point point;
        point.x = start(0);
        point.y = start(1);
        point.z = start(2);
        sphere.points.push_back(point);
        point.x = end(0);
        point.y = end(1);
        point.z = end(2);
        sphere.points.push_back(point);
        arrow_pub.publish(sphere);
    }

    void PlanVisual::displayTraj(Eigen::MatrixXd traj_pts, int id)
    {
        ROS_INFO("Not finished");
    }
}