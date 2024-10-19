/*! @file manipulator_controller.h
 *  @brief Controll methods of manipulator robots
 *
 *  This file contains the Manipulator_Controller_Classdef,
 *  which provide all controllers to controll manipulator robots.
 *  轮腿控制器
 */
#ifndef _MANIPULATOR_CONTROLLER_H_
#define _MANIPULATOR_CONTROLLER_H_

#include "lqrCalculater.h"
#include "manipulator.h"
#include "user_data.h"
#include "Upper_Public.h"
#include "state_data.h"

USING_NAMESPACE_MM
USING_NAMESPACE_QPOASES

#define MPC_CALC MPC_CalculatorClassdef<6, 3, 1, 5>

/*轮控制器参数列表*/
enum wheelCtrl_Enumdef
{
    W_wl_THETA,
    W_wr_THETA,
    W_ll_THETA,
    W_lr_THETA,
    W_b_THETA
};
/*腿控制器参数列表*/
enum jointCtrl_Enumdef
{
    J_wl_THETA,
    J_wr_THETA,
    J_ll_THETA,
    J_lr_THETA,
    J_b_THETA,
    J_TURN,
    J_LENGTHKEEP
};

enum legside_Enumdef
{
    LEFT_JOINT,
    RIGHT_JOINT
};

#ifdef __cplusplus
extern "C"
{
#endif
    class Manipulator_Controller_Classdef
    {
    public:
        Manipulator_Controller_Classdef() {}
        Manipulator_Controller_Classdef(State_Data_Classdef* _sd, Manipulator_Classdef* r_mp, Manipulator_Classdef* l_mp, UserData_Classdef* _user)
        {
            state = _sd;
            user = _user;
            mp[RIGHT_JOINT] = r_mp;
            mp[LEFT_JOINT] = l_mp;
        }
        //参数初始化
        void Init();
        //引用类型加载
        void Load_Reference_Type(State_Data_Classdef* _sd, Manipulator_Classdef* r_mp, Manipulator_Classdef* l_mp, UserData_Classdef* _user);
        //控制器加载
        void Load_Lqr_Controller(lqrCalculater<10, 4>* _lqrCal);
        void Load_Mpc_Controller(MPC_CALC* _mpcCal,modelFit<6,6,3>* _A,modelFit<6,3,3>* _B);
        void Load_Wheel_SubController(PIDmethod* _wSubCtrl);
        void Load_Joint_SubController(PIDmethod* _jSubCtrl);
        //控制器使能
        void Set_Enable_List(bool* _w, bool* _j);
        //控制结算
        void controll_adjust();
        //更新特殊状态
        void target_state2wheel_update();// 从目标位置和目标转角映射到轮子
        void current_state2wheel_update();
    private:
        State_Data_Classdef* state;
        UserData_Classdef* user;
        Manipulator_Classdef* mp[2];

        /*功能动作函数*/
        void jump_adjust();

        /*LQR闭环增益*/
        void lqr_adjust(Manipulator_Classdef* l_mp, Manipulator_Classdef* r_mp, lqrCalculater<10, 4>* lqr_cal);
        void lqr_state_config(Manipulator_Classdef* l_mp, Manipulator_Classdef* r_mp, float* state_target, float* state_current);

        /*mpc控制*/
        void mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B);
        void mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B, real_t ext_uk[3]);
        void mpc_state_config(Manipulator_Classdef* mp, real_t* state_target, real_t* state_current);


        float wheel_turn_adjust();                                 //转向环
        float wheel_feedforward_adjust(Manipulator_Classdef* _mp); //前馈环

        float length_keep_adjust(Manipulator_Classdef* _mp);                                      //轮腿腿长控制
        float joint_turn_adjust(Manipulator_Classdef* _mp_right, Manipulator_Classdef* _mp_left); //转向防劈叉
        float roll_keep_adjust();                                                                 //车体roll轴平衡
        float turn_adaption_adjust();                                                             //车体运动转向roll内倾
        float turn_adaption_adjust2();                                                            //车体运动转向力矩补偿2
        float turn_adaption_adjust3();                                                            //车体运动转向力矩补偿3

        /****************************控制器********************************/
        lqrCalculater<10, 4>* lqrCal; //腿的极性和轮的极性是相反的
        MPC_CALC* mpcCal[2];
        modelFit<6, 6, 3>* model_A;//状态矩阵
        modelFit<6, 3, 3>* model_B;//输出矩阵

        PIDmethod* w_turn_pid;
        PIDmethod* j_turn_pid;

        PIDmethod* j_follow_pid;
        PIDmethod* j_roll_keep_pid;
        PIDmethod* j_length_keep_pid;

        /*创建状态变量数组*/
        float lqr_target[10];
        float lqr_current[10];

        real_t mpc_target[6];
        real_t mpc_current[6];
        /*控制器使能数组*/
        bool wheel_enable_list[5] = { 0 };
        bool joint_enable_list[7] = { 0 };

        /* 其他变量 */
        float wheel2body_comp = 0;
        float wheel2body_comp_dot = 0;
    };
#ifdef __cplusplus
};
#endif

/****************************加载外部引用类型********************************/
void Manipulator_Controller_Classdef::Load_Reference_Type(State_Data_Classdef* _sd, Manipulator_Classdef* r_mp, Manipulator_Classdef* l_mp, UserData_Classdef* _user)
{
    state = _sd;
    user = _user;
    mp[RIGHT_JOINT] = r_mp;
    mp[LEFT_JOINT] = l_mp;
}

/****************************加载控制器********************************/
void Manipulator_Controller_Classdef::Load_Lqr_Controller(lqrCalculater<10, 4>* _lqrCal)
{
    lqrCal = _lqrCal;
}

void Manipulator_Controller_Classdef::Load_Mpc_Controller(MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B)
{
    mpcCal[RIGHT_JOINT] = &_mpcCal[RIGHT_JOINT];
    mpcCal[LEFT_JOINT] = &_mpcCal[LEFT_JOINT];
    model_A = _A;
    model_B = _B;
}

void Manipulator_Controller_Classdef::Load_Wheel_SubController(PIDmethod* _wSubCtrl)
{
    w_turn_pid = &_wSubCtrl[0];
    w_turn_pid->PID_Init(Common, state->dt);
}

void Manipulator_Controller_Classdef::Load_Joint_SubController(PIDmethod* _jSubCtrl)
{
    j_turn_pid = &_jSubCtrl[0];
    j_follow_pid = &_jSubCtrl[1];
    j_roll_keep_pid = &_jSubCtrl[2];
    j_length_keep_pid = &_jSubCtrl[3];
    j_turn_pid->PID_Init(Common, state->dt);
    j_follow_pid->PID_Init(Common, state->dt);
    j_roll_keep_pid->PID_Init(Common, state->dt);
    j_length_keep_pid->PID_Init(Common, state->dt);
}

void Manipulator_Controller_Classdef::Init()
{
    /*数据写入控制器*/
    lqrCal->init(user->leg_lqr_params);
    w_turn_pid->Params_Config(user->lqr_yaw_kp, 0, numeric_limits<float>::max());
    j_roll_keep_pid->Params_Config(PID_Mode::IS_PD, user->roll_keep_kp, user->roll_keep_kd, 0, numeric_limits<float>::max());
    j_length_keep_pid->Params_Config(PID_Mode::IS_PD, user->length_keep_kp, user->length_keep_kd, 0, numeric_limits<float>::max());
    j_turn_pid->Params_Config(user->j_turn_kp, 0, numeric_limits<float>::max());
    j_turn_pid->d_of_current = false;
    j_follow_pid->Params_Config(PID_Mode::IS_PD, user->j_follow_kp, user->j_follow_kd, 0, numeric_limits<float>::max());
    j_follow_pid->d_of_current = false;
    for (int i = 0; i < 7; i++)//初始化使能列表为false
    {
        if (i < 5)
        {
            wheel_enable_list[i] = false;
        }
        joint_enable_list[i] = false;
    }
    // 向模型类输入参数矩阵
    model_A->setFunctions(user->model_A);
    model_B->setFunctions(user->model_B);
}

/**
 * @brief  控制器使能函数
 * @note
 * @param   轮控制使能清单；腿控制使能清单；
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::Set_Enable_List(bool* _w, bool* _j)
{
    for (int i = 0; i < 7; i++)
    {
        if (i < 5)
        {
            if (_w[i] != false && _w[i] != true) {
                wheel_enable_list[i] = false;
            }
            else {
                wheel_enable_list[i] = _w[i];
            }
        }
        if (_j[i] != false && _j[i] != true) {
            joint_enable_list[i] = false;
        }
        else {
            joint_enable_list[i] = _j[i];
        }
    }
}

/**
 * @brief  总控函数
 * @note
 * @param
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::controll_adjust()
{
    float wheel_output[2][5];
    float joint_output[2][7];
    real_t wheel_mpc_out[2];
    real_t joint_mpc_out[2];

    // 使用卡尔曼滤波做速度融合
    this->wheel2body_comp = 0.5 * mp[LEFT_JOINT]->current_joint.pendulum.length * sinf(mp[LEFT_JOINT]->current_joint.pendulum.angle) +
        0.5 * mp[RIGHT_JOINT]->current_joint.pendulum.length * sinf(mp[RIGHT_JOINT]->current_joint.pendulum.angle) +
        mp[LEFT_JOINT]->body.length * sinf(state->current_pos.pitch);
    this->wheel2body_comp_dot = 0.5 * mp[LEFT_JOINT]->current_joint.pendulum.length * mp[LEFT_JOINT]->current_joint.pendulum.dangle * cosf(mp[LEFT_JOINT]->current_joint.pendulum.angle) +
        0.5 * mp[RIGHT_JOINT]->current_joint.pendulum.length * mp[RIGHT_JOINT]->current_joint.pendulum.dangle * cosf(mp[RIGHT_JOINT]->current_joint.pendulum.angle) +
        mp[LEFT_JOINT]->body.length * state->current_av.pitch * cosf(state->current_pos.pitch);
    state->use_estimator(this->wheel2body_comp, this->wheel2body_comp_dot);
    
    for (int i = 0; i < 2; i++)
    {
        mp[i]->body_angle_update(state->current_pos.pitch,state->current_av.pitch); //向腿杆更新车体倾角
        mp[i]->forward_kinematics_cal();
        mp[i]->overall_barycenter_cal();
    }
    /* 状态检测 */
    state->sport_adaption();
    jump_adjust();
    state->tiny_weightlessness_check();
    if (state->flags.leg_length_step)
    {
        state->weightlessness_check(); //跳跃模式下收完腿才开启腾空检测
    }

    state->target_pos.roll = turn_adaption_adjust(); //第一类转向补偿
    float length_error = roll_keep_adjust();         // roll计算
    if (state->flags.weightlessness == false && state->flags.leg_length_step == true)
    {
        mp[RIGHT_JOINT]->target_joint.pendulum.length += length_error;
        mp[LEFT_JOINT]->target_joint.pendulum.length -= length_error;
    }
    for (int i = 0; i < 2; i++)
    {
        /*轮腿控制部分(v是轮杆方向，h是轮杆法向)*/
        if (state->flags.weightlessness) {
            if (mp[i]->target_joint.pendulum.length < 0.22f || mp[i]->target_joint.pendulum.length > 0.4f)
            {
                mp[i]->target_joint.pendulum.length = 0.26f;
            }
        }
        else {}
    }

    /*轮控制*/
    wheel_turn_adjust();
    this->target_state2wheel_update();
    // this->current_state2wheel_update();
    /*lqr解算*/
    lqr_adjust(mp[LEFT_JOINT], mp[RIGHT_JOINT], lqrCal);
    for (int j = 0; j < 2; j++)
    {
        /*轮子控制参数导出*/
        wheel_output[j][W_wl_THETA] = lqrCal->single_out[j][0] + lqrCal->single_out[j][1];
        wheel_output[j][W_wr_THETA] = lqrCal->single_out[j][2] + lqrCal->single_out[j][3];
        wheel_output[j][W_ll_THETA] = lqrCal->single_out[j][4] + lqrCal->single_out[j][5];
        wheel_output[j][W_lr_THETA] = lqrCal->single_out[j][6] + lqrCal->single_out[j][7];
        wheel_output[j][W_b_THETA] = lqrCal->single_out[j][8] + lqrCal->single_out[j][9];
    }
    float turn_fv = 0;
    // turn_fv = turn_adaption_adjust2();//第二类转向补偿
    // turn_fv = turn_adaption_adjust3();//第三类转向补偿
    float theta[2];
    float fv[2];
    float fh[2];
    /*转向+劈叉控制*/
    joint_turn_adjust(mp[RIGHT_JOINT], mp[LEFT_JOINT]);
    for (int k = 0; k < 2; k++)
    {
        //失重或弹跳只留直立环
        if (state->flags.weightlessness || (!state->flags.leg_length_step) || state->flags.tiny_weightlessness)
        {
        }
        else
        {}
        ////real_t lqr_out[3] = { (real_t)lqrCal[k]->total_out[0],(real_t)lqrCal[k]->total_out[0],(real_t)lqrCal[k]->total_out[1] };
        ////mpc_adjust(mp[k], mpcCal[k], model_A, model_B, lqr_out);

        /*mpc_adjust(mp[k], mpcCal[k], model_A, model_B);
        MATRIX out = mpcCal[k]->getOutput();
        wheel_mpc_out[k] = out.getElement(1, 0);
        joint_mpc_out[k] = -out.getElement(2, 0) / mp[k]->current_joint.pendulum.length;*/
        /*获得腿杆控制量*/
        joint_output[k][J_wl_THETA] = (lqrCal->single_out[k+2][0] + lqrCal->single_out[k+2][1]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_wr_THETA] = (lqrCal->single_out[k+2][2] + lqrCal->single_out[k+2][3]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_ll_THETA] = (lqrCal->single_out[k+2][4] + lqrCal->single_out[k+2][5]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_lr_THETA] = (lqrCal->single_out[k+2][6] + lqrCal->single_out[k+2][7]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_b_THETA] = (lqrCal->single_out[k+2][7] + lqrCal->single_out[k+2][8]) / mp[k]->current_joint.pendulum.length;
        joint_output[k][J_TURN] = j_follow_pid->out - j_turn_pid->out;
        //joint_output[k][J_TURN] = j_follow_pid->out;
        joint_output[k][J_LENGTHKEEP] = length_keep_adjust(mp[k]);
        /*腿杆相对于地面的角度*/
        theta[k] = mp[k]->current_joint.pendulum.angle - state->current_pos.pitch;
        /*输出*/
        
        /*根据使能清单关闭相应的控制模块*/
        for (int n = 0; n < 7; n++)
        {
            if (n < 5)
            {
                if (wheel_enable_list[n] == false) {
                    wheel_output[k][n] = 0;
                }
            }
            if (joint_enable_list[n] == false) {
                joint_output[k][n] = 0;
            }
        }
        /*力矩集成输出*/
        if (k == RIGHT_JOINT)
        {
            fv[k] = -(joint_output[k][J_LENGTHKEEP] + turn_fv);
            fh[k] = (joint_output[k][J_wl_THETA] + joint_output[k][J_wr_THETA] + joint_output[k][J_ll_THETA] + joint_output[k][J_lr_THETA] + joint_output[k][J_b_THETA]) - joint_output[k][J_TURN];
            //fh[k] = joint_mpc_out[k] + joint_output[k][J_TURN];
        }
        else
        {
            fv[k] = joint_output[k][J_LENGTHKEEP] - turn_fv;
            fh[k] = -(joint_output[k][J_wl_THETA] + joint_output[k][J_wr_THETA] + joint_output[k][J_ll_THETA] + joint_output[k][J_lr_THETA] + joint_output[k][J_b_THETA]) - joint_output[k][J_TURN];
            //fh[k] = joint_mpc_out[k] - joint_output[k][J_TURN];
        }
        mp[k]->current_joint.Fx = fh[k] * cos(theta[k]) + fv[k] * sin(theta[k]);
        mp[k]->current_joint.Fy = -fh[k] * sin(theta[k]) + fv[k] * cos(theta[k]);
        mp[k]->forward_jacobian();
        //腿部使能
        //统一极性，以车辆前进方向为x轴，沿x轴向下角度增加旋转方向为力矩输出正方向
        if (state->flags.leg_enable)
        {
            if (k == RIGHT_JOINT)
            {
                mp[k]->torque_output.wheel = -(wheel_output[k][W_wl_THETA] + wheel_output[k][W_wr_THETA] + wheel_output[k][W_ll_THETA] + wheel_output[k][W_lr_THETA] + wheel_output[k][W_b_THETA]);
                //mp[k]->torque_output.wheel = wheel_mpc_out[k] /* + wheel_output[k][W_FF]*/ + wheel_output[k][W_TURN];
                mp[k]->torque_output.f_joint = mp[k]->current_joint.f_torque;
                mp[k]->torque_output.b_joint = mp[k]->current_joint.b_torque;
            }
            else
            {
                mp[k]->torque_output.wheel = wheel_output[k][W_wl_THETA] + wheel_output[k][W_wr_THETA] + wheel_output[k][W_ll_THETA] + wheel_output[k][W_lr_THETA] + wheel_output[k][W_b_THETA];
                //mp[k]->torque_output.wheel = -(wheel_mpc_out[k]/* + wheel_output[k][W_FF]*/ - wheel_output[k][W_TURN]);
                mp[k]->torque_output.f_joint = mp[k]->current_joint.f_torque;
                mp[k]->torque_output.b_joint = mp[k]->current_joint.b_torque;
            }
        }
        else
        {
            mp[k]->torque_output.wheel = 0;
            mp[k]->torque_output.f_joint = 0;
            mp[k]->torque_output.b_joint = 0;
        } 
    }
    cout << "ltar:" << mp[0]->target_joint.wheel.dangle << endl;
    cout << "lcur:" << mp[0]->current_joint.wheel.dangle << endl;
    cout << "rtar:" << mp[1]->target_joint.wheel.dangle << endl;
    cout << "rcur:" << mp[1]->current_joint.wheel.dangle << endl;
    /*cout << "wl:" << wheel_output[0][J_wl_THETA] << endl;
    cout << "wr:" << wheel_output[0][J_wr_THETA] << endl;
    cout << "ll:" << wheel_output[0][J_ll_THETA] << endl;
    cout << "lr:" << wheel_output[0][J_lr_THETA] << endl;
    cout << "b :" << wheel_output[0][J_b_THETA]  << endl;*/
    /*cout << "lo:" << state->target_location.y << ", " << state->current_location.y << endl;
    cout << "sp:" << state->target_speed.y << ", " << state->current_speed.y << endl;
    cout << "ya:" << state->target_pos.yaw << ", " << state->current_pos.yaw << endl;
    cout << "lw:" << lqrCal->error[0] << ", " << lqrCal->error[1] << endl;
    cout << "rw:" << lqrCal->error[2] << ", " << lqrCal->error[2] << endl;
    cout << "ll:" << lqrCal->error[4] << ", " << lqrCal->error[5] << endl;
    cout << "rl:" << lqrCal->error[6] << ", " << lqrCal->error[7] << endl;
    cout << "b :" << lqrCal->error[8] << ", " << lqrCal->error[9] << endl;*/
}

/*******************************************LQR控制***********************************************/
/**
 * @brief  lqr总控
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
void Manipulator_Controller_Classdef::lqr_adjust(Manipulator_Classdef* l_mp, Manipulator_Classdef* r_mp, lqrCalculater<10, 4>* lqr_cal)
{
    /*数组打包*/
    lqr_state_config(l_mp, r_mp, lqr_target, lqr_current);
    /*变量写入控制器*/
    lqr_cal->updateData(lqr_target, lqr_current, upper::constrain(0.5*(l_mp->current_joint.pendulum.length+r_mp->current_joint.pendulum.length), 0.13f, 0.4f));
    /*控制器结算*/
    lqr_cal->adjust();
}
/**
 * @brief  lqr状态空间变量设置
 * @note    把状态空间变量打包到给定数组当中
 * @param   目标状态1X6向量，当前状态1X6向量，状态使能1X6向量
 * @return
 * @retval  None
 */
void Manipulator_Controller_Classdef::lqr_state_config(Manipulator_Classdef* l_mp, Manipulator_Classdef* r_mp, float* state_target, float* state_current)
{
    /*state_target[0] = l_mp->target_joint.wheel.angle;
    state_current[0] = l_mp->current_joint.wheel.angle;
    state_target[1] = l_mp->target_joint.wheel.dangle;
    state_current[1] = l_mp->current_joint.wheel.dangle;
    state_target[2] = r_mp->target_joint.wheel.angle;
    state_current[2] = r_mp->current_joint.wheel.angle;
    state_target[3] = r_mp->target_joint.wheel.dangle;
    state_current[3] = r_mp->current_joint.wheel.dangle;*/
    state_target[0] = state->target_location.y - wheel2body_comp;
    state_current[0] = state->current_location.y;
    state_target[1] = state->target_speed.y - wheel2body_comp_dot;
    state_current[1] = state->current_speed.y;
    state_target[2] = state->target_pos.yaw;
    state_current[2] = state->current_pos.yaw;
    state_target[3] = state->target_av.yaw;
    state_current[3] = state->current_av.yaw;
    state_target[4] = l_mp->target_joint.pendulum.angle;
    state_current[4] = l_mp->current_joint.pendulum.angle;
    state_target[5] = 0;
    state_current[5] = l_mp->current_joint.pendulum.dangle;
    state_target[6] = r_mp->target_joint.pendulum.angle;
    state_current[6] = r_mp->current_joint.pendulum.angle;
    state_target[7] = 0;
    state_current[7] = r_mp->current_joint.pendulum.dangle;
    state_target[8] = -0.05;
    state_current[8] = l_mp->body.angle;
    state_target[9] = 0;
    state_current[9] = l_mp->body.dangle;


    for (int n = 0; n < 5; n++)
    {
        if (n < 5)
        {
            if (n == 4)
            {
                if (wheel_enable_list[n] == false) {
                    state_current[2 * n] = state_target[2 * n] = 0;
                    state_current[2 * n + 1] = state_target[2 * n + 1] = 0;
                }
            }
            else
            {
                if (wheel_enable_list[n] == false) {
                    state_current[2 * n] = state_target[2 * n] = 0;
                    state_current[2 * n + 1] = state_target[2 * n + 1] = 0;
                }
            }
        }
        
        /*if (joint_enable_list[n] == false) {
            joint_output[k][n] = 0;
        }*/
    }
}
// mpc总控
void Manipulator_Controller_Classdef::mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B)
{
    mpc_state_config(_mp, mpc_target, mpc_current);
    MATRIX A = model_A->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX B = model_B->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX Q(6);
    MATRIX R(3);
    MATRIX F(6);
    MATRIX lb(1, 3);
    MATRIX ub(1, 3);
    MATRIX X(6, 1);
    MATRIX Y(6, 1);
    Q.setArray(user->Q, sizeof(user->Q) / sizeof(real_t));
    R.setArray(user->R, sizeof(user->R) / sizeof(real_t));
    lb.setArray(user->lb, sizeof(user->lb) / sizeof(real_t));
    ub.setArray(user->ub, sizeof(user->ub) / sizeof(real_t));
    X.setArray(mpc_current, sizeof(mpc_current) / sizeof(real_t));
    Y.setArray(mpc_target, sizeof(mpc_target) / sizeof(real_t));
    F = Q * 2;
    _mpcCal->setConstrain(lb, ub);
    _mpcCal->mpc_update(Y, X, 20, 0.008);
    _mpcCal->mpc_init(A, B, Q, R, F);
    _mpcCal->mpc_solve();
    _mpcCal->compare_storage();
}

//mpc接入外部控制器输出
void Manipulator_Controller_Classdef::mpc_adjust(Manipulator_Classdef* _mp, MPC_CALC* _mpcCal, modelFit<6, 6, 3>* _A, modelFit<6, 3, 3>* _B, real_t ext_uk[3])
{
    mpc_state_config(_mp, mpc_target, mpc_current);
    MATRIX A = model_A->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX B = model_B->modelGenerate(_mp->current_joint.pendulum.length);
    MATRIX Q(6);
    MATRIX R(3);
    MATRIX F(6);
    MATRIX X(6, 1);
    MATRIX Y(6, 1);
    Q.setArray(user->Q, sizeof(user->Q) / sizeof(real_t));
    R.setArray(user->R, sizeof(user->R) / sizeof(real_t));
    X.setArray(mpc_current, sizeof(mpc_current) / sizeof(real_t));
    Y.setArray(mpc_target, sizeof(mpc_target) / sizeof(real_t));
    F = Q;
    _mpcCal->mpc_update(Y, X);
    _mpcCal->mpc_init(A, B, Q, R, F);
    _mpcCal->mpc_solve(ext_uk);
}

// mpc状态空间变量设置
void Manipulator_Controller_Classdef::mpc_state_config(Manipulator_Classdef* mp, real_t* state_target, real_t* state_current)
{
    state_target[0] = state->target_location.y;
    state_current[0] = state->current_location.y;
    if (mp->target_joint.pendulum.length > 0.25)
    {
        state_target[1] = 0.5 * state->target_speed.y;
    }
    else
    {
        state_target[1] = state->target_speed.y;
    }
    state_current[1] = state->current_speed.y;
    state_target[2] = mp->target_joint.pendulum.angle;
    state_current[2] = mp->current_joint.pendulum.angle;
    state_target[3] = 0;
    state_current[3] = mp->current_joint.pendulum.dangle;
    state_target[4] = state->target_pos.pitch;
    state_current[4] = state->current_pos.pitch;
    state_target[5] = 0;
    state_current[5] = state->current_av.pitch;

    for (int n = 0; n < 6; n++)
    {
        if (wheel_enable_list[n] == false) {
            state_current[n] = state_target[n] = 0;
        }
        /*if (joint_enable_list[n] == false) {
            joint_output[k][n] = 0;
        }*/
    }
}

/*******************************************轮控制***********************************************/

/**
 * @brief  lqr转向环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheel_turn_adjust()
{
    static float last_target = 0;
    if (state->target_av.yaw >= last_target)
    {
        if (abs(state->target_av.yaw - last_target) > user->turn_step)
        {
            last_target += user->turn_step;
        }
        else
        {
            last_target = state->target_av.yaw;
        }
    }
    else if (state->target_av.yaw < last_target)
    {
        if (abs(state->target_av.yaw - last_target) > user->turn_step)
        {
            last_target -= user->turn_step;
        }
        else
        {
            last_target = state->target_av.yaw;
        }
    }
    else
    {
    }
    w_turn_pid->target = last_target;
    w_turn_pid->current = state->current_av.yaw;
    w_turn_pid->Adjust(0);
    return w_turn_pid->out;
}

/**
 * @brief  lqr前馈环
 * @note
 * @param
 * @return  力矩输出
 * @retval  None
 */
float Manipulator_Controller_Classdef::wheel_feedforward_adjust(Manipulator_Classdef* _mp)
{
    /*float error = 0 - _mp->overall.angle;*/
    float error = 0 - state->current_pos.pitch;
    float debug_feedforward = -_mp->overall.mass * 9.8f * _mp->overall.length;
    /*float debug_feedforward = -0.5*_mp.body.mass * 9.8 * (_mp.current_joint.pendulum.length+_mp.body.length);*/
    /*float debug_feedforward = -0.5 * _mp.body.mass * 9.8 * _mp->overall.length;*/
    /*float debug_feedforward = -_mp->overall.mass * 9.8 * (_mp.current_joint.pendulum.length + _mp.body.length);*/
    if (error > 0)
    {
        return debug_feedforward * sin(abs(error));
    }
    else
    {
        return -debug_feedforward * sin(abs(error));
    }
}

/*********************************************腿控制****************************************************/

/**
 * @brief  车体转向内倾
 * @note    用角速度，速度，腿长三个变量拟合增益
 * @param
 * @return  roll轴角度目标值
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        float out;
        if (state->current_speed.y >= 0)
        {
            out = -user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.6f / 3.f);
        }
        else if (state->current_speed.y < 0)
        {
            out = user->turn_adaption_kp * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.6f / 3.f);
        }
        return out;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体转向内倾2
 * @note    用角速度，速度，腿长三个变量拟合增益
 * @param
 * @return  两轮杆方向力矩差分
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust2()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        float out;
        if (state->current_speed.y >= 0)
        {
            out = -user->turn_adaption_kp2 * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.8f / 3.f);
        }
        else if (state->current_speed.y < 0)
        {
            out = user->turn_adaption_kp2 * (state->current_av.yaw / abs(state->current_av.yaw)) * (0.4f + abs(state->current_av.yaw) * 1.1f / 1.5f) * pow((mp[RIGHT_JOINT]->current_joint.pendulum.length + mp[LEFT_JOINT]->current_joint.pendulum.length), 2) * (0.7f + abs(state->current_speed.y) * 0.8f / 3.f);
        }
        return out;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体转向内倾3
 * @note    用roll倾角拟合增益
 * @param
 * @return  两轮杆方向力矩差分
 * @retval  None
 */
float Manipulator_Controller_Classdef::turn_adaption_adjust3()
{
    if (state->flags.sport_flag && abs(state->current_av.yaw) > 0.2f)
    {
        // return -(current_pos.roll- target_pos.roll) * turn_adaption_kp3*(0.4 + abs(current_av.yaw) * 1.1 / 1.5) * pow((current_right_joint.pendulum.length + current_left_joint.pendulum.length), 2) * (0.7 + abs(current_speed.y) * 0.8 / 3.);
        return -(state->current_pos.roll - state->target_pos.roll) * user->turn_adaption_kp3;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  车体roll轴平衡
 * @note
 * @param
 * @return  高度双腿差分值
 * @retval  None
 */
float Manipulator_Controller_Classdef::roll_keep_adjust()
{
    j_roll_keep_pid->target = state->current_pos.roll;
    j_roll_keep_pid->current = state->target_pos.roll;
    j_roll_keep_pid->Adjust(0, state->current_av.roll);
    return j_roll_keep_pid->out;
}

/**
 * @brief  轮腿高度自适应
 * @note
 * @param
 * @return  得出某个轮腿在y方向上的力
 * @retval  None
 */
float Manipulator_Controller_Classdef::length_keep_adjust(Manipulator_Classdef* _mp)
{
    float length_target = upper::constrain(_mp->target_joint.pendulum.length, 0.13, 0.4);
    float length_kd;
    float length_kp;
    float length_keep_feedforward = -0.5 * mp[RIGHT_JOINT]->body.mass * 9.8;
    //length_keep_feedforward = 0;
    if (state->flags.leg_length_step) //轮腿高度的非阶跃响应
    {
        if (length_target > _mp->target_joint.pendulum.last_length)
        {
            _mp->target_joint.pendulum.last_length += user->length_upstep;
        }
        else if (length_target < _mp->target_joint.pendulum.last_length)
        {
            _mp->target_joint.pendulum.last_length -= user->length_downstep;
        }
        else
        {
        }
        length_kd = user->length_keep_kd;
        length_kp = user->length_keep_kp;
    }
    else //阶跃响应
    {
        _mp->target_joint.pendulum.last_length = length_target;
        if (state->flags.kd_inhibition)
        {
            length_kd = user->length_keep_kd / 2.f;
            length_kp = user->length_keep_kp * 7.f / 6.f;
        }
        else
        {
            length_kd = user->length_keep_kd;
            length_kp = user->length_keep_kp;
        }
    }
    j_length_keep_pid->kp = length_kp;
    j_length_keep_pid->kd = length_kd;
    j_length_keep_pid->target = -_mp->target_joint.pendulum.last_length;
    j_length_keep_pid->current = -_mp->current_joint.pendulum.length;
    j_length_keep_pid->Adjust(0, _mp->current_joint.pendulum.dlength);
    return length_keep_feedforward + j_length_keep_pid->out;
}

/**
 * @brief  关节转向环
 * @note    用于避免两腿开叉
 * @param
 * @return  末端力
 * @retval  None
 */
float Manipulator_Controller_Classdef::joint_turn_adjust(Manipulator_Classdef* _mp_right, Manipulator_Classdef* _mp_left)
{
    j_turn_pid->target = state->target_av.yaw;
    j_turn_pid->current = state->current_av.yaw;
    j_turn_pid->Adjust(0);
    j_follow_pid->target = _mp_right->current_joint.pendulum.angle;
    j_follow_pid->current = _mp_left->current_joint.pendulum.angle;
    j_follow_pid->Adjust(0);
    return j_follow_pid->out - j_turn_pid->out;
}

/*******************************功能动作函数**************************************/
/**
 * @brief  车体弹跳函数
 * @note
 * @param
 * @return  末端力
 * @retval  None
 */
void Manipulator_Controller_Classdef::jump_adjust()
{
    static int count = 0;
    static bool is_build = false;
    /*cout << right_mp->current_joint.pendulum.length << endl;
    cout << left_mp->current_joint.pendulum.length << endl;*/
    if (state->flags.jump_flag)
    {
        count += 120;
        if (mp[RIGHT_JOINT]->current_joint.pendulum.length > 0.22f || mp[LEFT_JOINT]->current_joint.pendulum.length > 0.22f)
        {
            is_build = true;
        }
        else
        {
            is_build = false;
        }
        state->flags.jump_flag = false;
    }
    else
    {
        count--;
    }
    count = upper::constrain(count, 0, 120);
    if (is_build)
    {
        if (count % 120 > 90)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.2f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.2f;
            state->flags.kd_inhibition = false;
        }
        else if (count % 120 < 90 && count % 120 > 70)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.2f + 0.3f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.2f + 0.3f;
            state->flags.kd_inhibition = true;
        }
        else if (count % 120 < 70 && count % 120 > 40)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length = 0.2f - 0.1f;
            mp[LEFT_JOINT]->target_joint.pendulum.length = 0.2f - 0.1f;
            state->flags.kd_inhibition = true;
        }
        else
        {
            state->flags.kd_inhibition = false;
        }
        if (count > 40)
        {
            state->flags.leg_length_step = false;
        }
        else
        {
            state->flags.leg_length_step = true;
            is_build = false;
        }
    }
    else
    {
        if (count % 120 > 100)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length += 0.3f;
            mp[LEFT_JOINT]->target_joint.pendulum.length += 0.3f;
            state->flags.kd_inhibition = true;
        }
        else if (count % 120 < 100 && count % 120 > 70)
        {
            mp[RIGHT_JOINT]->target_joint.pendulum.length -= 0.1f;
            mp[LEFT_JOINT]->target_joint.pendulum.length -= 0.1f;
            state->flags.kd_inhibition = true;
        }
        else
        {
            state->flags.kd_inhibition = false;
        }
        if (count > 70)
        {
            state->flags.leg_length_step = false;
        }
        else
        {
            state->flags.leg_length_step = true;
            is_build = false;
        }
    }
}

void Manipulator_Controller_Classdef::target_state2wheel_update()
{
    double R = 0.07;
    double D = 0.54;
    double hl = mp[LEFT_JOINT]->target_joint.pendulum.length;
    double hr = mp[RIGHT_JOINT]->target_joint.pendulum.length;
    double thetall = mp[LEFT_JOINT]->target_joint.pendulum.angle;
    double thetalr = mp[RIGHT_JOINT]->target_joint.pendulum.angle;
    double dthetall = mp[LEFT_JOINT]->target_joint.pendulum.dangle;
    double dthetalr = mp[RIGHT_JOINT]->target_joint.pendulum.dangle;
    state->target_pos.yaw = state->current_pos.yaw;
    //state->target_location.y = state->current_location.y;
    /*this->mp[LEFT_JOINT]->target_joint.wheel.angle = (state->target_location.y - 0.5 * D * (state->target_pos.yaw + hl / D * sinf(thetall) - hr / D * sinf(thetalr))) / R;
    this->mp[RIGHT_JOINT]->target_joint.wheel.angle = (state->target_location.y + 0.5 * D * (state->target_pos.yaw + hl / D * sinf(thetall) - hr / D * sinf(thetalr))) / R;*/
    this->mp[LEFT_JOINT]->target_joint.wheel.dangle = (state->target_speed.y - 0.5 * D * (state->target_av.yaw + hl / D * dthetall * cosf(thetall) - hr / D * dthetalr * cosf(thetalr))) / R;
    this->mp[RIGHT_JOINT]->target_joint.wheel.dangle = (state->target_speed.y + 0.5 * D * (state->target_av.yaw + hl / D * dthetall * cosf(thetall) - hr / D * dthetalr * cosf(thetalr))) / R;
    //this->mp[LEFT_JOINT]->target_joint.wheel.angle = (state->target_location.y - 0.5 * D * (state->target_pos.yaw)) / R;
    //this->mp[RIGHT_JOINT]->target_joint.wheel.angle = (state->target_location.y + 0.5 * D * (state->target_pos.yaw)) / R;
    this->mp[LEFT_JOINT]->target_joint.wheel.angle = 0;
    this->mp[RIGHT_JOINT]->target_joint.wheel.angle = 0;
    /*this->mp[LEFT_JOINT]->target_joint.wheel.dangle = (state->target_speed.y - 0.5 * D * (state->target_av.yaw)) / R;
    this->mp[RIGHT_JOINT]->target_joint.wheel.dangle = (state->target_speed.y + 0.5 * D * (state->target_av.yaw)) / R;*/
}

void Manipulator_Controller_Classdef::current_state2wheel_update()
{
    double R = 0.07;
    double D = 0.54;
    double hl = mp[LEFT_JOINT]->current_joint.pendulum.length;
    double hr = mp[RIGHT_JOINT]->current_joint.pendulum.length;
    double thetall = mp[LEFT_JOINT]->current_joint.pendulum.angle;
    double thetalr = mp[RIGHT_JOINT]->current_joint.pendulum.angle;
    double dthetall = mp[LEFT_JOINT]->current_joint.pendulum.dangle;
    double dthetalr = mp[RIGHT_JOINT]->current_joint.pendulum.dangle;
    /*this->mp[LEFT_JOINT]->current_joint.wheel.angle = (state->current_location.y - 0.5 * D * (state->current_pos.yaw + hl / D * sinf(thetall) - hr / D * sinf(thetalr))) / R;
    this->mp[RIGHT_JOINT]->current_joint.wheel.angle = (state->current_location.y + 0.5 * D * (state->current_pos.yaw + hl / D * sinf(thetall) - hr / D * sinf(thetalr))) / R;
    this->mp[LEFT_JOINT]->current_joint.wheel.dangle = (state->current_speed.y - 0.5 * D * (state->current_av.yaw + hl / D * dthetall * cosf(thetall) - hr / D * dthetalr * cosf(thetalr))) / R;
    this->mp[RIGHT_JOINT]->current_joint.wheel.dangle = (state->current_speed.y + 0.5 * D * (state->current_av.yaw + hl / D * dthetall * cosf(thetall) - hr / D * dthetalr * cosf(thetalr))) / R;*/
    //this->mp[LEFT_JOINT]->current_joint.wheel.angle = (state->current_location.y - 0.5 * D * (state->current_pos.yaw)) / R;
    //this->mp[RIGHT_JOINT]->current_joint.wheel.angle = (state->current_location.y + 0.5 * D * (state->current_pos.yaw)) / R;
    this->mp[LEFT_JOINT]->current_joint.wheel.angle = 0;
    this->mp[RIGHT_JOINT]->current_joint.wheel.angle = 0;
    this->mp[LEFT_JOINT]->current_joint.wheel.dangle = (state->current_speed.y - 0.5 * D * (state->current_av.yaw)) / R;
    this->mp[RIGHT_JOINT]->current_joint.wheel.dangle = (state->current_speed.y + 0.5 * D * (state->current_av.yaw)) / R;
}

#endif
