#include <min_snap/min_snap_optimization.h>

namespace my_planner
{
    minsnapOptimization::minsnapOptimization(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        this->Init(waypoints, meanvel);
    }

    void minsnapOptimization::Init(const vector<Eigen::Vector3d> &waypoints, double meanvel)
    {
        n_order = 7;
        wps = waypoints;
        n_seg = int(wps.size()) - 1;
        n_per_seg = n_order + 1;
        n_all_poly = n_seg * n_per_seg;
        n_lambda = 5 * (n_seg - 1) + 8;
        sta_vaj = Eigen::MatrixXd::Zero(3, 3);
        mean_vel = meanvel;
    }

    void minsnapOptimization::set_sta_state(const Eigen::MatrixXd &vaj)
    {
        sta_vaj = vaj;
    }

    void minsnapOptimization::init_ts(int init_type)
    {
        const double dist_min = 2.0;
        ts = Eigen::VectorXd::Zero(n_seg);
        if (init_type)
        {
            Eigen::VectorXd dist(n_seg);
            double dist_sum = 0, t_sum = 0;
            for (int i = 0; i < n_seg; i++)
            {
                dist(i) = 0;
                for (int j = 0; j < 3; j++)
                {
                    dist(i) += pow(wps[i + 1](j) - wps[i](j), 2);
                }
                dist(i) = sqrt(dist(i));
                if ((dist(i)) < dist_min)
                {
                    dist(i) = sqrt(dist(i)) * 2;
                }
                dist_sum += dist(i);
            }
            dist(0) += mean_vel;
            dist(n_seg - 1) += mean_vel;
            dist_sum += 2 * mean_vel;
            double T = dist_sum / mean_vel;
            for (int i = 0; i < n_seg - 1; i++)
            {
                ts(i) = dist(i) / dist_sum * T;
                t_sum += ts(i);
            }
            ts(n_seg - 1) = T - t_sum;
        }
        else
        {
            for (int i = 0; i < n_seg; i++)
            {
                ts(i) = 1;
            }
        }
        cout << "ts: " << ts.transpose() << endl;
    }

    int minsnapOptimization::fact(int n)
    {
        if (n < 0)
        {
            cout << "ERROR fact(" << n << ")" << endl;
            return 0;
        }
        else if (n == 0)
        {
            return 1;
        }
        else
        {
            return n * fact(n - 1);
        }
    }

    Eigen::MatrixXd minsnapOptimization::getPolyCoef()
    {
        Eigen::MatrixXd poly_coef(poly_coef_x.size(), 3);
        poly_coef << poly_coef_x, poly_coef_y, poly_coef_z;
        return poly_coef;
    }

    Eigen::MatrixXd minsnapOptimization::getDecVel()
    {
        Eigen::MatrixXd dec_vel(dec_vel_x.size(), 3);
        dec_vel << dec_vel_x, dec_vel_y, dec_vel_z;
        return dec_vel;
    }

    Eigen::VectorXd minsnapOptimization::getTime()
    {
        return ts;
    }

    void minsnapOptimization::getQ(void)
    {
        Q = Eigen::MatrixXd::Zero(n_seg * n_per_seg, n_seg * n_per_seg);
        for (int k = 0; k < n_seg; k++)
        {
            Eigen::MatrixXd Q_k(Eigen::MatrixXd::Zero(n_per_seg, n_per_seg));
            for (int i = 4; i <= n_order; i++)
            {
                for (int j = 4; j <= n_order; j++)
                {
                    Q_k(i, j) = fact(i) / fact(i - 4) *
                                fact(j) / fact(j - 4) /
                                (i + j - 7) * pow(ts(k), i + j - 7);
                }
            }
            Q.block(k * n_per_seg, k * n_per_seg, n_per_seg, n_per_seg) = Q_k;
        }
    }

    void minsnapOptimization::getM()
    {
        M = Eigen::MatrixXd::Zero(n_seg * n_per_seg, n_seg * n_per_seg);
        for (int k = 0; k < n_seg; k++)
        {
            Eigen::MatrixXd M_k(Eigen::MatrixXd::Zero(n_per_seg, n_per_seg));
            M_k(0, 0) = 1;
            M_k(1, 1) = 1;
            M_k(2, 2) = 2;
            M_k(3, 3) = 6;
            for (int i = 0; i <= n_order; i++)
            {
                for (int j = 0; j <= 3; j++)
                {
                    if (i >= j)
                    {
                        M_k(j + 4, i) = fact(i) / fact(i - j) * pow(ts(k), i - j);
                    }
                }
            }
            M.block(k * n_per_seg, k * n_per_seg, n_per_seg, n_per_seg) = M_k;
        }
    }

    Eigen::MatrixXd minsnapOptimization::getLambda()
    {
        Eigen::MatrixXd lambda(lambda_y.size(), 3);
        lambda << lambda_x, lambda_y, lambda_z;
        return lambda;
    }

    void minsnapOptimization::getAbeq(const Eigen::VectorXd &waypoints, const Eigen::MatrixXd &cond)
    {
        // p,v,a,j constraint in start
        Eigen::MatrixXd Aeq_start = Eigen::MatrixXd::Zero(4, n_all_poly);
        Eigen::VectorXd beq_start = Eigen::Vector4d::Zero(4);

        for (int i = 0; i < n_per_seg; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (i >= j)
                {
                    Aeq_start(j, i) = fact(i) / fact(i - j) * pow(0, j - j);
                }
            }
        }
        beq_start = cond.row(0).transpose();

        // p,v,a constraint in end
        Eigen::MatrixXd Aeq_end = Eigen::MatrixXd::Zero(4, n_all_poly);
        Eigen::VectorXd beq_end = Eigen::VectorXd::Zero(4);

        for (int i = 0; i < n_per_seg; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                if (i >= j)
                {
                    Aeq_end(j, (n_seg - 1) * n_per_seg + i) = fact(i) / fact(i - j) * pow(ts(ts.size() - 1), i - j);
                }
            }
        }
        beq_end = cond.row(1).transpose();

        // position constrain in all middle waypoints
        Eigen::MatrixXd Aeq_wp = Eigen::MatrixXd::Zero(n_seg - 1, n_all_poly);
        Eigen::VectorXd beq_wp = Eigen::VectorXd::Zero(n_seg - 1);

        for (int i = 1; i < n_seg; i++)
        {
            Aeq_wp(i - 1, i * n_per_seg) = 1;
        }
        for (int i = 1; i < waypoints.size() - 1; i++)
        {
            beq_wp(i - 1) = waypoints(i);
        }

        // position continuity constraint of Aeq_com_p and beq_con_p
        Eigen::MatrixXd Aeq_con_p = Eigen::MatrixXd::Zero(n_seg - 1, n_all_poly);

        for (int j = 1; j < n_seg; j++)
        {
            for (int i = 0; i < n_per_seg; i++)
            {
                Aeq_con_p(j - 1, (j - 1) * n_per_seg + i) = pow(ts(j - 1), i);
                Aeq_con_p(j - 1, j * n_per_seg + i) = -pow(0, i);
            }
        }

        // velocity continuity constrain between each 2 segments
        Eigen::MatrixXd Aeq_con_v = Eigen::MatrixXd::Zero(n_seg - 1, n_all_poly);

        for (int j = 1; j < n_seg; j++)
        {
            for (int i = 0; i < n_per_seg; i++)
            {
                if (i >= 1)
                {
                    Aeq_con_v(j - 1, (j - 1) * n_per_seg + i) = fact(i) / fact(i - 1) * pow(ts(j - 1), i - 1);
                    Aeq_con_v(j - 1, j * n_per_seg + i) = -fact(i) / fact(i - 1) * pow(0, i - 1);
                }
            }
        }

        // acceleration continuity constrain between each 2 sements
        Eigen::MatrixXd Aeq_con_a = Eigen::MatrixXd::Zero(n_seg - 1, n_all_poly);

        for (int j = 1; j < n_seg; j++)
        {
            for (int i = 0; i < n_per_seg; i++)
            {
                if (i >= 2)
                {
                    Aeq_con_a(j - 1, (j - 1) * n_per_seg + i) = fact(i) / fact(i - 2) * pow(ts(j - 1), i - 2);
                    Aeq_con_a(j - 1, j * n_per_seg + i) = -fact(i) / fact(i - 2) * pow(0, i - 2);
                }
            }
        }

        // jerk continuity constraint between each 2 segments
        Eigen::MatrixXd Aeq_con_j = Eigen::MatrixXd::Zero(n_seg - 1, n_all_poly);

        for (int j = 1; j < n_seg; j++)
        {
            for (int i = 0; i < n_per_seg; i++)
            {
                if (i >= 3)
                {
                    Aeq_con_j(j - 1, (j - 1) * n_per_seg + i) = fact(i) / fact(i - 3) * pow(ts(j - 1), i - 3);
                    Aeq_con_j(j - 1, j * n_per_seg + i) = -fact(i) / fact(i - 3) * pow(0, i - 3);
                }
            }
        }

        // combine all components to form Aeq and Beq

        Aeq = Eigen::MatrixXd::Zero(5 * (n_seg - 1) + 8, n_all_poly);
        Beq = Eigen::VectorXd::Zero(5 * (n_seg - 1) + 8, 1);

        Aeq.middleRows(0, 4) = Aeq_start;
        Aeq.middleRows(4, 4) = Aeq_end;
        Aeq.middleRows(8, n_seg - 1) = Aeq_wp;
        Aeq.middleRows((n_seg - 1) * 1 + 8, n_seg - 1) = Aeq_con_p;
        Aeq.middleRows((n_seg - 1) * 2 + 8, n_seg - 1) = Aeq_con_v;
        Aeq.middleRows((n_seg - 1) * 3 + 8, n_seg - 1) = Aeq_con_a;
        Aeq.middleRows((n_seg - 1) * 4 + 8, n_seg - 1) = Aeq_con_j;

        Beq.segment(0, 4) = beq_start;
        Beq.segment(4, 4) = beq_end;
        Beq.segment(8, n_seg - 1) = beq_wp;
    }

    Eigen::VectorXd minsnapOptimization::calDecVel(const Eigen::VectorXd decvel)
    {
        Eigen::VectorXd temp(Eigen::VectorXd::Zero((n_seg + 1) * 4));
        for (int i = 0; i < (n_seg + 1) / 2; i++)
        {
            temp.segment(i * 8, 8) = decvel.segment((i * 2) * 8, 8);
        }
        if (n_seg % 2 != 1)
        {
            temp.tail(4) = decvel.tail(4);
        }

        return temp;
    }

    double minsnapOptimization::_evaluate(void *instance,
                                          const double *x,
                                          double *g,
                                          const int n)
    {
        return reinterpret_cast<minsnapOptimization *>(instance)->evaluate(x, g, n);
    }

    double minsnapOptimization::evaluate(const double *x,
                                         double *g,
                                         const int n)
    {
        double min_value = 0.0;
        Eigen::VectorXd poly_coef = Eigen::VectorXd::Zero(n);
        for (int i = 0; i < n; i++)
        {
            poly_coef(i) = x[i];
        }

        min_value = poly_coef.transpose() * F * poly_coef - 2 * poly_coef.transpose().dot(E);

        Eigen::VectorXd gredient = (F + F.transpose()) * poly_coef - 2 * E;
        for (int i = 0; i < n; i++)
        {
            g[i] = gredient(i);
        }
        return min_value;
    }

    int minsnapOptimization::_progress(void *instance,
                                       const double *x,
                                       const double *g,
                                       const double fx,
                                       const double xnorm,
                                       const double gnorm,
                                       const double step,
                                       int n,
                                       int k,
                                       int ls)
    {
        return reinterpret_cast<minsnapOptimization *>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
    }

    int minsnapOptimization::progress(const double *x,
                                      const double *g,
                                      const double fx,
                                      const double xnorm,
                                      const double gnorm,
                                      const double step,
                                      int n,
                                      int k,
                                      int ls)
    {
        cout << endl;
        cout << "*************** Iteration = " << k << ", fx = " << fx << "****************" << endl;
        cout << "poly_coef = " << endl;
        for (int i = 0; i < n; i++)
        {
            cout << "x[i]" << x[i] << endl;
        }
        cout << "xnorm = " << xnorm << ", gnorm = " << gnorm << ", step = " << step << endl;
        cout << endl;
        return 0;
    }

    std::pair<Eigen::VectorXd, Eigen::VectorXd> minsnapOptimization::minsnapOptServer(const Eigen::VectorXd &wp, Eigen::Vector3d &vaj)
    {
        std::pair<Eigen::VectorXd, Eigen::VectorXd> return_vel;
        Eigen::VectorXd poly_coef = Eigen::VectorXd::Zero(n_all_poly);
        Eigen::VectorXd dec_vel = Eigen::VectorXd::Zero(n_all_poly);
        Eigen::MatrixXd cond = Eigen::MatrixXd::Zero(2, 4);
        Eigen::MatrixXd G, H;

        cond(0, 0) = wp(0);
        cond(1, 0) = wp(n_seg);
        cond.block(0, 1, 1, 3) = vaj.transpose();
        getAbeq(wp, cond);
        G = M.inverse().transpose() * Q * M.inverse();
        H = Aeq * M.inverse();
        F = G + H.transpose() * H;
        E = H.transpose() * Beq;

        double minsnap_value;
        double *dec = (double *)malloc(sizeof(double) * n_all_poly);

        for (int i = 0; i < n_all_poly; i++)
        {
            dec[i] = 1;
        }

        for (int i = 0; i < 4; i++)
        {
            dec[i] = cond(0, i) / fact(i);
        }

        lambda = Eigen::VectorXd::Zero(n_lambda);
        for (int i = 0; i < n_lambda; i++)
        {
            lambda(i) = 100;
        }

        lbfgs::lbfgs_parameter_t lbfgs_params;
        lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
        lbfgs_params.max_iterations = 10000;
        lbfgs_params.max_linesearch = 100;
        int ret = lbfgs::lbfgs_optimize(n_all_poly, dec, &minsnap_value, _evaluate, NULL, _progress, this, &lbfgs_params);

        cout << "lbfgs ret = " << ret << endl;

        for (int i = 0; i < n_all_poly; i++)
        {
            dec_vel(i) = dec[i];
        }
        poly_coef = M.inverse() * dec_vel;
        return_vel.first = poly_coef;
        return_vel.second = dec_vel;
        if (dec != NULL)
        {
            free(dec);
            dec = NULL;
        }

        return return_vel;
    }

    void minsnapOptimization::calMinsnap_polycoef()
    {
        Eigen::VectorXd wps_x(n_seg + 1), wps_y(n_seg + 1), wps_z(n_seg + 1);
        Eigen::Vector3d vaj_x, vaj_y, vaj_z;
        for (int i = 0; i < n_seg + 1; i++)
        {
            wps_x(i) = wps[i](0);
            vaj_x = sta_vaj.col(0);
            wps_y(i) = wps[i](1);
            vaj_y = sta_vaj.col(1);
            wps_z(i) = wps[i](2);
            vaj_z = sta_vaj.col(2);
        }
        init_ts(1);
        getQ();
        getM();

        std::pair<Eigen::VectorXd, Eigen::VectorXd> return_value;
        return_value = minsnapOptServer(wps_x, vaj_x);
        poly_coef_x = return_value.first;
        dec_vel_x = calDecVel(return_value.second);
        lambda_x = lambda;
        return_value = minsnapOptServer(wps_y, vaj_y);
        poly_coef_y = return_value.first;
        dec_vel_y = calDecVel(return_value.second);
        lambda_y = lambda;
        return_value = minsnapOptServer(wps_z, vaj_x);
        poly_coef_z = return_value.first;
        dec_vel_z = calDecVel(return_value.second);
        lambda_z = lambda;
    }

}