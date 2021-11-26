#ifndef _MIN_SNAP_OPTIMIZATION_H_
#define _MIN_SNAP_OPTIMIZATION_H_

#include "lbfgs.hpp"
#include <Eigen/Eigen>
#include <iostream>

using std::vector;
using namespace std;

namespace my_planner
{
    class minsnapOptimization
    {
    private:
        vector<Eigen::Vector3d> wps;
        int n_order, n_seg, n_per_seg, n_all_poly, n_lambda;
        double mean_vel;
        Eigen::VectorXd ts, Beq, E;
        Eigen::VectorXd poly_coef_x, poly_coef_y, poly_coef_z;
        Eigen::VectorXd dec_vel_x, dec_vel_y, dec_vel_z;
        Eigen::VectorXd lambda, lambda_x, lambda_y, lambda_z;
        Eigen::MatrixXd sta_vaj;
        Eigen::MatrixXd Q, M, Aeq, F;

        static double _evaluate(void *instance, const double *x, double *g, const int n);
        double evaluate(const double *x, double *g, const int n);
        static int _progress(void *instance, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
        int progress(const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);

        int fact(int n);
        void init_ts(int init_type);
        void getQ(void);
        void getM(void);
        void getAbeq(const Eigen::VectorXd &waypoints, const Eigen::MatrixXd &cond);
        Eigen::VectorXd calDecVel(const Eigen::VectorXd decvel);
        std::pair<Eigen::VectorXd, Eigen::VectorXd> minsnapOptServer(const Eigen::VectorXd &wp, Eigen::Vector3d &vaj);

    public:
        minsnapOptimization(){};
        ~minsnapOptimization(){};
        minsnapOptimization(const vector<Eigen::Vector3d> &waypoints, double meanvel = 1.0);
        void Init(const vector<Eigen::Vector3d> &waypoints, double meanvel = 1.0);
        void set_sta_state(const Eigen::MatrixXd &vaj);
        Eigen::MatrixXd getPolyCoef();
        Eigen::VectorXd getTime();
        Eigen::MatrixXd getDecVel();
        Eigen::MatrixXd getLambda();
        void calMinsnap_polycoef();
    };
}

#endif