// File:          parallelWheelBipe_controller.cpp
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

// All the webots classes are defined in the "webots" namespace
using namespace webots;

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
  
  keyboard->enable(timeStep);
  camera->enable(timeStep);
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

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    /*控制量下发*/
  right_wheel_motor->setTorque(0);
  left_wheel_motor->setTorque(0);
  rf_motor->setTorque(0);
  rb_motor->setTorque(0);
  lf_motor->setTorque(0);
  lb_motor->setTorque(0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
