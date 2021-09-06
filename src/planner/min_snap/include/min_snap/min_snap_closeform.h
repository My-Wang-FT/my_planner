#ifndef _MIN_SNAP_CLOSEFORM_H_
#define _MIN_SNAP_CLOSEFORM_H_

#include <Eigen/Eigen>
#include <iostream>

using std::vector;
using namespace std;

namespace my_planner
{
    class minsnapCloseform
    {
    private:
        vector<Eigen::Vector3d> wps;
        int n_order, n_seg, n_per_seg;
        Eigen::VectorXd ts;
        Eigen::VectorXd poly_coef_x, poly_coef_y, poly_coef_z;
        Eigen::MatrixXd Q, M, Ct;
        Eigen::MatrixXd sta_vaj;

        int fact(int n);
        void init_ts(int init_type);
        Eigen::VectorXd MinSnapCloseFormServer(Eigen::VectorXd &wp, Eigen::Vector3d &vaj);
        void calQ();
        void calM();
        void calCt();

    public:
        minsnapCloseform(){};
        ~minsnapCloseform(){};
        minsnapCloseform(const vector<Eigen::Vector3d> &waypoints);
        void Init(const vector<Eigen::Vector3d> &waypoints);
        void set_sta_state(Eigen::MatrixXd vaj);
        void calMinsnap_polycoef();
        Eigen::MatrixXd getPolyCoef();
        Eigen::VectorXd getTime();
    };
}
#endif