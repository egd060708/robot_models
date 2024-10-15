// File:          parallelWb_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Camera.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Display.hpp>
#include <string>
#include <fstream>
#include "state_data.h"
#include "manipulator.h"
#include "manipulator_controller.h"
#include "dataDisplay.h"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

#define INFINITY std::numeric_limits<double>::infinity()

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  /*自定义算法类*/
  PIDmethod wheel_subController;
  PIDmethod joint_subController[4];
  lqrCalculater<10, 4> lqr_calculate[2];
  State_Data_Classdef infantry_state((float)timeStep / 1000.);
  UserData_Classdef user_params;
  Manipulator_Classdef right_manipulator(infantry_state.dt);
  Manipulator_Classdef left_manipulator(infantry_state.dt);
  Manipulator_Controller_Classdef controller(&infantry_state, &right_manipulator, &left_manipulator, &user_params);
  MPC_CALC mpcCal[2];
  modelFit<6, 6, 3> modelA;
  modelFit<6, 3, 3> modelB;
  right_manipulator.Init(RF_JOINT_OFFSET, RB_JOINT_OFFSET, F_JOINT_MAX, F_JOINT_MIN, B_JOINT_MAX, B_JOINT_MIN);
  left_manipulator.Init(LF_JOINT_OFFSET, LB_JOINT_OFFSET, F_JOINT_MAX, F_JOINT_MIN, B_JOINT_MAX, B_JOINT_MIN);
  controller.Load_Lqr_Controller(lqr_calculate);
  controller.Load_Wheel_SubController(&wheel_subController);
  controller.Load_Joint_SubController(joint_subController);
  controller.Load_Mpc_Controller(mpcCal, &modelA, &modelB);
  controller.Init();//控制器最后初始化
  bool w_en[6] = { true,true,true,true,true,true };
  bool j_en[6] = { true,true,true,true,true,true };
  
  /*****************************************************************************************/

  Keyboard* keyboard = robot->getKeyboard();
  Gyro* gyro_sensor = robot->getGyro("gyro");
  InertialUnit* eular_sensor = robot->getInertialUnit("inertial unit");
  Accelerometer* acc_sensor = robot->getAccelerometer("accelerometer");
  Motor* rf_motor = robot->getMotor("rf_motor");
  Motor* rb_motor = robot->getMotor("rb_motor");
  Motor* lf_motor = robot->getMotor("lf_motor");
  Motor* lb_motor = robot->getMotor("lb_motor");
  Motor* right_wheel_motor = robot->getMotor("right_wheel_motor");
  Motor* left_wheel_motor = robot->getMotor("left_wheel_motor");
  PositionSensor* rf_ps = robot->getPositionSensor("rf_ps");
  PositionSensor* rb_ps = robot->getPositionSensor("rb_ps");
  PositionSensor* lf_ps = robot->getPositionSensor("lf_ps");
  PositionSensor* lb_ps = robot->getPositionSensor("lb_ps");
  PositionSensor* right_wheel_ps = robot->getPositionSensor("right_wheel_ps");
  PositionSensor* left_wheel_ps = robot->getPositionSensor("left_wheel_ps");
  Camera* camera = robot->getCamera("camera");
  Display* draw = robot->getDisplay("display");

  dataDisplay<6> dataDisp(draw);
  // Asuwave_Channel asu_test;

  keyboard->enable(timeStep);
  /*camera->enable(timeStep);*/
  gyro_sensor->enable(timeStep);
  eular_sensor->enable(timeStep);
  acc_sensor->enable(timeStep);
  rf_ps->enable(timeStep);
  rb_ps->enable(timeStep);
  lf_ps->enable(timeStep);
  lb_ps->enable(timeStep);
  rf_motor->enableTorqueFeedback(timeStep);
  rf_motor->enableForceFeedback(timeStep);
  rb_motor->enableTorqueFeedback(timeStep);
  lf_motor->enableTorqueFeedback(timeStep);
  lb_motor->enableTorqueFeedback(timeStep);
  right_wheel_ps->enable(timeStep);
  left_wheel_ps->enable(timeStep);
  right_wheel_motor->setPosition(INFINITY);
  left_wheel_motor->setPosition(INFINITY);
  right_wheel_motor->setVelocity(0);
  left_wheel_motor->setVelocity(0);
  lf_motor->setPosition(INFINITY);
  lf_motor->setVelocity(0);
  lb_motor->setPosition(INFINITY);
  lb_motor->setVelocity(0);
  rf_motor->setPosition(INFINITY);
  rf_motor->setVelocity(0);
  rb_motor->setPosition(INFINITY);
  rb_motor->setVelocity(0);

  float target_length = 0.15;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
      //Read the keyboard
      static float target_yspeed = 0;
      static float last_target_yspeed = 0;
      static float target_zspeed = 0;
      static float last_target_zspeed = 0;
      bool y_speed_get = false;
      bool z_speed_get = false;
      float y_speed_max = 2.0;

      int key = keyboard->getKey();
      while (key > 0)
      {
          switch (key)
          {
          case 'W': if (infantry_state.flags.leap_flag) { target_yspeed += 0.15; y_speed_max = 4.5; }
                  else { target_yspeed += 0.05; y_speed_max = 2.0; }
                  y_speed_get = true; break;
          case keyboard->SHIFT + 'W': if (infantry_state.flags.leap_flag) { target_yspeed += 0.2; y_speed_max = 5.0; }
                                    else { target_yspeed += 0.1; y_speed_max = 3.0; }
                                    y_speed_get = true; break;
          case 'S': target_yspeed -= 0.04, y_speed_max = 2.0, y_speed_get = true; break;
          case keyboard->SHIFT + 'S': target_yspeed -= 0.08, y_speed_max = 3.0, y_speed_get = true; break;
          default:  break;
          }
          switch (key)
          {
          case 'A': target_zspeed = 1.5, z_speed_get = true; break;
          case keyboard->SHIFT + 'A': target_zspeed = 2.5, z_speed_get = true; break;
          case 'D': target_zspeed = -1.5, z_speed_get = true; break;
          case keyboard->SHIFT + 'D': target_zspeed = -2.5, z_speed_get = true; break;
          default:  break;
          }
          switch (key)
          {
          case 'R': infantry_state.flags.leap_flag = true; break;
          case 'E': infantry_state.flags.leap_flag = false; break;
          case 'G': infantry_state.flags.slope_flag = true; break;
          case 'F': infantry_state.flags.slope_flag = false; break;
          case 'O': infantry_state.flags.leg_enable = true; break;
          case 'P': infantry_state.flags.leg_enable = false; break;
          case ' ': infantry_state.flags.jump_flag = true; break;
          case keyboard->SHIFT + ' ':infantry_state.flags.jump_flag = true; break;
          case 'Z': target_length = 0.15f; break;
          case 'X': target_length = 0.20f; break;
          case 'C': target_length = 0.26f; break;
          case 'V': target_length = 0.32f; break;
          case keyboard->SHIFT + 'Z': target_length = 0.15f; break;
          case keyboard->SHIFT + 'X': target_length = 0.20f; break;
          case keyboard->SHIFT + 'C': target_length = 0.26f; break;
          case keyboard->SHIFT + 'V': target_length = 0.32f; break;
          default:break;
          }
          key = keyboard->getKey();
      }

      if (!y_speed_get)
      {
          target_yspeed = 0;
      }
      if (!z_speed_get)
      {
          target_zspeed = 0;
      }
      target_yspeed = upper::constrain(target_yspeed, -y_speed_max, y_speed_max);

      right_manipulator.target_joint.pendulum.length = target_length;
      left_manipulator.target_joint.pendulum.length = target_length;

      infantry_state.timeStamp_update((float)timeStep / 1000.);
      infantry_state.target_update(target_yspeed, target_zspeed);
      // Read the sensors:
        // Process sensor data here.
        /*轮子部分*/
      float current_left_ps = left_wheel_ps->getValue();
      static float last_clps = 0;
      float current_lspeed = (current_left_ps - last_clps) / timeStep * 1000 * 0.07;
      float current_right_ps = right_wheel_ps->getValue();
      static float last_crps = 0;
      float current_rspeed = (current_right_ps - last_crps) / timeStep * 1000 * 0.07;
      float current_distance = 0.5 * (current_left_ps - current_right_ps) * 0.07;//获取当前距离
      float current_speed = 0.5 * (current_lspeed - current_rspeed);//获取当前速度
      last_clps = current_left_ps;
      last_crps = current_right_ps;
      
      infantry_state.current_location_update(0, current_distance, 0);
      infantry_state.current_speed_update(0, current_speed, 0);
      /*姿态部分*/
      const double* gyro = gyro_sensor->getValues();
      const double* eular = eular_sensor->getRollPitchYaw();
      const double* acc = acc_sensor->getValues();
      infantry_state.current_pos_update(eular[1], eular[2], eular[0]);
      infantry_state.current_av_update(gyro[1], gyro[2], gyro[0]);
      infantry_state.current_acc_update(acc[0], acc[1], acc[2]);
      /*轮腿部分*/
      float rf_alpha = rf_ps->getValue();
      float rb_alpha = rb_ps->getValue();
      float lf_alpha = lf_ps->getValue();
      float lb_alpha = lb_ps->getValue();
      right_manipulator.current_joint_update(-rf_alpha, -rb_alpha);
      right_manipulator.current_wheel_update(current_left_ps, current_lspeed);
      //right_manipulator.body_angle_update(infantry_state.current_pos.pitch,infantry_state.current_av.pitch);
      left_manipulator.current_joint_update(lf_alpha, lb_alpha);
      left_manipulator.current_joint_update(current_right_ps, current_rspeed);
      //left_manipulator.body_angle_update(infantry_state.current_pos.pitch,infantry_state.current_av.pitch);

      /*控制解算*/
      controller.Set_Enable_List(w_en, j_en);
      controller.controll_adjust();

      /*控制量下发*/
      right_wheel_motor->setTorque(-upper::constrain(right_manipulator.torque_output.wheel, -5., 5.));
      left_wheel_motor->setTorque(-upper::constrain(left_manipulator.torque_output.wheel, -5., 5.));
      rf_motor->setTorque(-upper::constrain(right_manipulator.torque_output.f_joint, -17.5, 17.5));
      rb_motor->setTorque(-upper::constrain(right_manipulator.torque_output.b_joint, -17.5, 17.5));
      lf_motor->setTorque(-upper::constrain(left_manipulator.torque_output.f_joint, -17.5, 17.5));
      lb_motor->setTorque(-upper::constrain(left_manipulator.torque_output.b_joint, -17.5, 17.5));

      //std::cout << upper::constrain(right_manipulator.torque_output.f_joint, -17.5, 17.5)<< " " << rf_motor->getTorqueFeedback() << std::endl;

      /*cout << right_manipulator.current_joint.f_angle << "\t" << right_manipulator.current_joint.b_angle << endl;
      cout << right_manipulator.torque_output.f_joint << "\t" << right_manipulator.torque_output.b_joint << endl;*/

      // MPC对比测试
      /*static int count = 0;
      if (infantry_state.flags.leg_enable == true)
      {
          if (count < 2500)
          {
              MATRIX tmp = mpcCal[0].getCompareState();
              for (int i = 0; i < 12; i++)
              {
                  x_compare[i][count] = tmp.getElement(i, 0);
              }
          }
          count++;
      }*/

      //写入数据到绘图视窗并输出
      float _data[6] = { mpcCal[0].getCompareState().getElement(0,0),mpcCal[0].getCompareState().getElement(1,0),mpcCal[0].getCompareState().getElement(2,0),mpcCal[0].getCompareState().getElement(3,0),mpcCal[0].getCompareState().getElement(4,0) ,mpcCal[0].getCompareState().getElement(5,0) };
      dataDisp.sendCtrl(_data);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
