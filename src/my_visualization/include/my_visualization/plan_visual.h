#ifndef _PLAN_VISUAL_H_
#define _PLAN_VISUAL_H_

// rviz\DisplayTypes\Marker: http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <istream>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace my_planner
{
    class PlanVisual
    {
    private:
        ros::NodeHandle nh;

        ros::Publisher goal_point_pub;
        ros::Publisher arrow_pub;
        ros::Publisher traj_pub;

    public:
        PlanVisual(){};
        ~PlanVisual(){};
        PlanVisual(ros::NodeHandle &node);
        typedef std::shared_ptr<PlanVisual> Ptr;
        void visualInit(ros::NodeHandle &node);
        void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
        void displayArrow(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector4d color, int id);
        void displayTraj(Eigen::MatrixXd traj_pts, int id);
    };
}

#endif