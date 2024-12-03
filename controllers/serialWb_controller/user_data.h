#ifndef _USER_DATA_H_
#define _USER_DATA_H_

#include "PIDmethod.h"
#include "MPC_Calculator.h"
#include "model_fit.h"
#include <qpOASES.hpp>

class UserData_Classdef
{
public:
    UserData_Classdef() { Init(); }
    /**************************轮控制参数**************************/
    /*轮子转向环*/
    float turn_step;
    float lqr_yaw_kp;
    /*轮子距离环*/
    float distance_max;
    /*车体roll轴平衡*/
    float roll_keep_kp;
    float roll_keep_kd;
    /**************************腿控制参数*********************************/
    /*双轮差速内倾*/
    float turn_adaption_kp;
    float turn_adaption_kp2;
    float turn_adaption_kp3;
    /*轮腿高度自适应*/
    float length_keep_kp;
    float length_keep_kd;
    float length_upstep;//让目标高度不要突变
    float length_downstep;
    /*轮腿防劈叉*/
    float j_turn_kp;
    float j_follow_kp;
    float j_follow_kd;
    /*lqr增益参数*/
    Fit_Params leg_lqr_params[4][10];
    /*MPC模型参数*/
    real_t model_A[4 * 6 * 6] = { 1.000000e+00,-1.159163e-15,4.115609e-15,-4.707396e-15,
8.000000e-03,1.288028e-16,-4.573146e-16,5.230722e-16,
1.021743e-03,-2.237107e-02,4.657511e-02,-3.587594e-02,
2.720633e-06,-5.958619e-05,1.240049e-04,-9.549441e-05,
-6.977148e-06,4.798242e-05,-9.989627e-05,7.694823e-05,
-1.859682e-08,1.278009e-07,-2.659667e-07,2.048172e-07,
0,0,0,0,
1.000000e+00,-1.159163e-15,4.115609e-15,-4.707396e-15,
2.563773e-01,-5.609183e+00,1.168961e+01,-9.009973e+00,
1.021743e-03,-2.237107e-02,4.657511e-02,-3.587594e-02,
-1.746376e-03,1.203129e-02,-2.507337e-02,1.932574e-02,
-6.977148e-06,4.798242e-05,-9.989627e-05,7.694823e-05,
0,0,0,0,
0,0,0,0,
1.005667e+00,6.851309e-02,-2.765350e-01,2.990702e-01,
8.015111e-03,1.824448e-04,-7.363721e-04,7.963372e-04,
-1.215522e-05,-1.469498e-04,5.931241e-04,-6.414584e-04,
-3.240985e-08,-3.913091e-07,1.579377e-06,-1.707991e-06,
0,0,0,0,
0,0,0,0,
1.417183e+00,1.718849e+01,-6.938110e+01,7.504503e+01,
1.005667e+00,6.851309e-02,-2.765350e-01,2.990702e-01,
-3.039755e-03,-3.686807e-02,1.488174e-01,-1.609661e-01,
-1.215522e-05,-1.469498e-04,5.931241e-04,-6.414584e-04,
0,0,0,0,
0,0,0,0,
1.101149e-04,-2.410966e-03,5.019475e-03,-3.866408e-03,
2.932023e-07,-6.421597e-06,1.336399e-05,-1.029142e-05,
1.000240e+00,5.171143e-06,-1.076598e-05,8.292836e-06,
8.000639e-03,1.377309e-08,-2.866320e-08,2.207313e-08,
0,0,0,0,
0,0,0,0,
2.763129e-02,-6.045348e-01,1.259858e+00,-9.710578e-01,
1.101149e-04,-2.410966e-03,5.019475e-03,-3.866408e-03,
5.993044e-02,1.296683e-03,-2.702305e-03,2.082848e-03,
1.000240e+00,5.171143e-06,-1.076598e-05,8.292836e-06
    };
    real_t model_B[4 * 6 * 3] = { -2.432609e-05,2.162173e-03,-5.247458e-03,4.521477e-03,
-2.432609e-05,2.162173e-03,-5.247458e-03,4.521477e-03,
-2.961975e-05,-3.951305e-04,1.568590e-03,-1.687714e-03,
-6.154301e-03,5.421852e-01,-1.316890e+00,1.135177e+00,
-6.154301e-03,5.421852e-01,-1.316890e+00,1.135177e+00,
-7.406506e-03,-9.912811e-02,3.935533e-01,-4.234991e-01,
-1.491137e-03,-6.640609e-05,1.081240e-02,-1.518933e-02,
-1.491137e-03,-6.640609e-05,1.081240e-02,-1.518933e-02,
1.043498e-03,-5.345295e-03,1.103051e-02,-8.433576e-03,
-3.732293e-01,-1.969717e-02,2.720860e+00,-3.819405e+00,
-3.732293e-01,-1.969717e-02,2.720860e+00,-3.819405e+00,
2.612891e-01,-1.337984e+00,2.759401e+00,-2.108238e+00,
-2.449938e-05,2.330209e-04,-5.655270e-04,4.872868e-04,
-2.449938e-05,2.330209e-04,-5.655270e-04,4.872868e-04,
-7.050656e-05,-4.258386e-05,1.690495e-04,-1.818876e-04,
-6.132933e-03,5.843450e-02,-1.419290e-01,1.223447e-01,
-6.132933e-03,5.843450e-02,-1.419290e-01,1.223447e-01,
-1.762751e-02,-1.068362e-02,4.241556e-02,-4.564300e-02
    };
    real_t Q[6 * 6] = {
        -1000,0,0,0,0,0,
        0,-50,0,0,0,0,
        0,0,1200,0,0,0,
        0,0,0,6,0,0,
        0,0,0,0,30000,0,
        0,0,0,0,0,60
    };
    real_t R[3 * 3] = {
        1.5,0,0,
        0,1.5,0,
        0,0,0.3
    };
    real_t F[6 * 6];
    real_t lb[1 * 3] = { -6,-6,-40 };
    real_t ub[1 * 3] = { 6, 6, 40 };//bug，当轮子输出限幅较小时，正输出存在偏小无法起立的情况

    void Init()
    {
        /**************************轮控制参数**************************/
        /*轮子转向环*/
        lqr_yaw_kp = 4;
        turn_step = 0.1;
        /**************************腿控制参数**************************/
        /*车体转向内倾*/
        turn_adaption_kp = 0.09;
        turn_adaption_kp2 = 200.;
        turn_adaption_kp3 = 20000.;
        /*车体roll轴自平衡*/
        roll_keep_kp = 1.5;
        roll_keep_kd = 0.1;
        /*轮腿防劈叉*/
        j_turn_kp = 5;
        j_follow_kp = 150;
        j_follow_kd = 4;
        /*高度自适应*/
        length_keep_kp = 1800.;
        length_keep_kd = 110.;
        length_upstep = 0.01;
        length_downstep = 0.01;

        /**************************LQR控制参数*********************************/
        /*参数*/
        leg_lqr_params[0][0].a = -8.580984e-03;
        leg_lqr_params[0][0].b = 8.691125e-03;
        leg_lqr_params[0][0].c = -3.064532e-03;
        leg_lqr_params[0][0].d = -6.771224e-04;
        /*参数*/
        leg_lqr_params[0][1].a = -1.919166e+01;
        leg_lqr_params[0][1].b = 1.943823e+01;
        leg_lqr_params[0][1].c = -6.854543e+00;
        leg_lqr_params[0][1].d = -1.514388e+00;
        /*参数*/
        leg_lqr_params[0][2].a = -4.895911e-03;
        leg_lqr_params[0][2].b = 5.136941e-03;
        leg_lqr_params[0][2].c = -1.624336e-03;
        leg_lqr_params[0][2].d = -9.300758e-04;
        /*参数*/
        leg_lqr_params[0][3].a = -7.741381e+00;
        leg_lqr_params[0][3].b = 8.122572e+00;
        leg_lqr_params[0][3].c = -2.568408e+00;
        leg_lqr_params[0][3].d = -1.470618e+00;
        /*参数*/
        leg_lqr_params[0][4].a = -9.103327e+01;
        leg_lqr_params[0][4].b = 1.148174e+02;
        leg_lqr_params[0][4].c = -4.670219e+01;
        leg_lqr_params[0][4].d = -1.726714e+00;
        /*参数*/
        leg_lqr_params[0][5].a = -2.257595e+00;
        leg_lqr_params[0][5].b = 6.808268e+00;
        leg_lqr_params[0][5].c = -5.630719e+00;
        leg_lqr_params[0][5].d = 7.535236e-02;
        /*参数*/
        leg_lqr_params[0][6].a = -5.311498e+01;
        leg_lqr_params[0][6].b = 5.434890e+01;
        leg_lqr_params[0][6].c = -3.912238e+01;
        leg_lqr_params[0][6].d = -5.456855e-01;
        /*参数*/
        leg_lqr_params[0][7].a = -1.258796e-01;
        leg_lqr_params[0][7].b = -3.281499e+00;
        leg_lqr_params[0][7].c = -2.374681e+00;
        leg_lqr_params[0][7].d = 5.552558e-02;
        /*参数*/
        leg_lqr_params[0][8].a = 6.898404e+02;
        leg_lqr_params[0][8].b = -7.612409e+02;
        leg_lqr_params[0][8].c = 3.089456e+02;
        leg_lqr_params[0][8].d = -5.781468e+01;
        /*参数*/
        leg_lqr_params[0][9].a = 3.699802e+01;
        leg_lqr_params[0][9].b = -4.257956e+01;
        leg_lqr_params[0][9].c = 1.852628e+01;
        leg_lqr_params[0][9].d = -3.944866e+00;
        /*参数*/
        leg_lqr_params[1][0].a = -8.580984e-03;
        leg_lqr_params[1][0].b = 8.691125e-03;
        leg_lqr_params[1][0].c = -3.064532e-03;
        leg_lqr_params[1][0].d = -6.771224e-04;
        /*参数*/
        leg_lqr_params[1][1].a = -1.919166e+01;
        leg_lqr_params[1][1].b = 1.943823e+01;
        leg_lqr_params[1][1].c = -6.854543e+00;
        leg_lqr_params[1][1].d = -1.514388e+00;
        /*参数*/
        leg_lqr_params[1][2].a = 4.895910e-03;
        leg_lqr_params[1][2].b = -5.136940e-03;
        leg_lqr_params[1][2].c = 1.624336e-03;
        leg_lqr_params[1][2].d = 9.300758e-04;
        /*参数*/
        leg_lqr_params[1][3].a = 7.741381e+00;
        leg_lqr_params[1][3].b = -8.122572e+00;
        leg_lqr_params[1][3].c = 2.568408e+00;
        leg_lqr_params[1][3].d = 1.470618e+00;
        /*参数*/
        leg_lqr_params[1][4].a = -5.311498e+01;
        leg_lqr_params[1][4].b = 5.434890e+01;
        leg_lqr_params[1][4].c = -3.912238e+01;
        leg_lqr_params[1][4].d = -5.456855e-01;
        /*参数*/
        leg_lqr_params[1][5].a = -1.258796e-01;
        leg_lqr_params[1][5].b = -3.281499e+00;
        leg_lqr_params[1][5].c = -2.374681e+00;
        leg_lqr_params[1][5].d = 5.552558e-02;
        /*参数*/
        leg_lqr_params[1][6].a = -9.103327e+01;
        leg_lqr_params[1][6].b = 1.148174e+02;
        leg_lqr_params[1][6].c = -4.670219e+01;
        leg_lqr_params[1][6].d = -1.726714e+00;
        /*参数*/
        leg_lqr_params[1][7].a = -2.257595e+00;
        leg_lqr_params[1][7].b = 6.808268e+00;
        leg_lqr_params[1][7].c = -5.630719e+00;
        leg_lqr_params[1][7].d = 7.535236e-02;
        /*参数*/
        leg_lqr_params[1][8].a = 6.898404e+02;
        leg_lqr_params[1][8].b = -7.612409e+02;
        leg_lqr_params[1][8].c = 3.089456e+02;
        leg_lqr_params[1][8].d = -5.781468e+01;
        /*参数*/
        leg_lqr_params[1][9].a = 3.699802e+01;
        leg_lqr_params[1][9].b = -4.257956e+01;
        leg_lqr_params[1][9].c = 1.852628e+01;
        leg_lqr_params[1][9].d = -3.944866e+00;
        /*参数*/
        leg_lqr_params[2][0].a = -3.148654e-02;
        leg_lqr_params[2][0].b = 3.469926e-02;
        leg_lqr_params[2][0].c = -1.400926e-02;
        leg_lqr_params[2][0].d = 2.590956e-03;
        /*参数*/
        leg_lqr_params[2][1].a = -7.041867e+01;
        leg_lqr_params[2][1].b = 7.760391e+01;
        leg_lqr_params[2][1].c = -3.133133e+01;
        leg_lqr_params[2][1].d = 5.794713e+00;
        /*参数*/
        leg_lqr_params[2][2].a = 2.340755e-02;
        leg_lqr_params[2][2].b = -2.755230e-02;
        leg_lqr_params[2][2].c = 1.405272e-02;
        leg_lqr_params[2][2].d = -2.093348e-03;
        /*参数*/
        leg_lqr_params[2][3].a = 3.701049e+01;
        leg_lqr_params[2][3].b = -4.356401e+01;
        leg_lqr_params[2][3].c = 2.222002e+01;
        leg_lqr_params[2][3].d = -3.309977e+00;
        /*参数*/
        leg_lqr_params[2][4].a = 5.077600e+00;
        leg_lqr_params[2][4].b = -1.469901e+01;
        leg_lqr_params[2][4].c = 1.192013e+02;
        leg_lqr_params[2][4].d = 4.557448e+00;
        /*参数*/
        leg_lqr_params[2][5].a = -1.639765e+01;
        leg_lqr_params[2][5].b = 3.264287e+01;
        leg_lqr_params[2][5].c = 9.051318e+00;
        leg_lqr_params[2][5].d = -9.524836e-03;
        /*参数*/
        leg_lqr_params[2][6].a = 2.086439e+01;
        leg_lqr_params[2][6].b = 2.724177e+00;
        leg_lqr_params[2][6].c = -1.232945e+02;
        leg_lqr_params[2][6].d = 5.897838e+00;
        /*参数*/
        leg_lqr_params[2][7].a = 2.381987e+01;
        leg_lqr_params[2][7].b = -3.956027e+01;
        leg_lqr_params[2][7].c = -6.081841e+00;
        leg_lqr_params[2][7].d = 6.370758e-01;
        /*参数*/
        leg_lqr_params[2][8].a = -1.295694e+03;
        leg_lqr_params[2][8].b = 1.309694e+03;
        leg_lqr_params[2][8].c = -4.585257e+02;
        leg_lqr_params[2][8].d = -1.089804e+02;
        /*参数*/
        leg_lqr_params[2][9].a = -8.374758e+01;
        leg_lqr_params[2][9].b = 8.643541e+01;
        leg_lqr_params[2][9].c = -3.140919e+01;
        leg_lqr_params[2][9].d = -2.999203e+00;
        /*参数*/
        leg_lqr_params[3][0].a = -3.148654e-02;
        leg_lqr_params[3][0].b = 3.469926e-02;
        leg_lqr_params[3][0].c = -1.400926e-02;
        leg_lqr_params[3][0].d = 2.590956e-03;
        /*参数*/
        leg_lqr_params[3][1].a = -7.041867e+01;
        leg_lqr_params[3][1].b = 7.760391e+01;
        leg_lqr_params[3][1].c = -3.133133e+01;
        leg_lqr_params[3][1].d = 5.794713e+00;
        /*参数*/
        leg_lqr_params[3][2].a = -2.340755e-02;
        leg_lqr_params[3][2].b = 2.755230e-02;
        leg_lqr_params[3][2].c = -1.405272e-02;
        leg_lqr_params[3][2].d = 2.093348e-03;
        /*参数*/
        leg_lqr_params[3][3].a = -3.701049e+01;
        leg_lqr_params[3][3].b = 4.356401e+01;
        leg_lqr_params[3][3].c = -2.222002e+01;
        leg_lqr_params[3][3].d = 3.309977e+00;
        /*参数*/
        leg_lqr_params[3][4].a = 2.086439e+01;
        leg_lqr_params[3][4].b = 2.724177e+00;
        leg_lqr_params[3][4].c = -1.232945e+02;
        leg_lqr_params[3][4].d = 5.897838e+00;
        /*参数*/
        leg_lqr_params[3][5].a = 2.381987e+01;
        leg_lqr_params[3][5].b = -3.956027e+01;
        leg_lqr_params[3][5].c = -6.081841e+00;
        leg_lqr_params[3][5].d = 6.370758e-01;
        /*参数*/
        leg_lqr_params[3][6].a = 5.077600e+00;
        leg_lqr_params[3][6].b = -1.469901e+01;
        leg_lqr_params[3][6].c = 1.192013e+02;
        leg_lqr_params[3][6].d = 4.557448e+00;
        /*参数*/
        leg_lqr_params[3][7].a = -1.639765e+01;
        leg_lqr_params[3][7].b = 3.264287e+01;
        leg_lqr_params[3][7].c = 9.051318e+00;
        leg_lqr_params[3][7].d = -9.524836e-03;
        /*参数*/
        leg_lqr_params[3][8].a = -1.295694e+03;
        leg_lqr_params[3][8].b = 1.309694e+03;
        leg_lqr_params[3][8].c = -4.585257e+02;
        leg_lqr_params[3][8].d = -1.089804e+02;
        /*参数*/
        leg_lqr_params[3][9].a = -8.374758e+01;
        leg_lqr_params[3][9].b = 8.643541e+01;
        leg_lqr_params[3][9].c = -3.140919e+01;
        leg_lqr_params[3][9].d = -2.999203e+00;


        leg_lqr_params[2][0].d = 5.;
        leg_lqr_params[3][0].d = 5.;

    }
};

#endif
