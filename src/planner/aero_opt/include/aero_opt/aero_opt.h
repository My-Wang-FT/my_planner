#ifndef _AERO_OPT_H_
#define _AERO_OPT_H_

#include <Eigen/Eigen>
#include <iostream>
#include "aero_opt/lbfgs.hpp"
#include "min_snap/min_snap_closeform.h"

using namespace std;
using std::vector;

namespace my_planner
{
    class aero_optimization :public minsnapCloseform
    {
    private:
     

    public:
        aero_optimization(){};
        ~aero_optimization(){};

        void testfunction();

    };
} // namespace my_planner

#endif