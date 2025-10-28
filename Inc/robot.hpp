#ifndef _ROBOT_HPP_
#define _ROBOT_HPP_

#include<pid.hpp>
#include<threads.hpp>
#include<parameters.hpp>

#include<array>
#include<queue>

#include<franka/duration.h>
#include<franka/exception.h>
#include<franka/model.h>
#include<franka/robot.h>

namespace CSIR{
namespace Robot{


// typedef MessageQue<std::array<double, DOF> > messageQ;
typedef std::array<double, DOF> arrayDOF;

/**
 * @brief initialization of robot
 * 
 * @param robot 
 */
void initialize(franka::Robot& robot);

/**
 * @brief infinite control loop for robot
 * 
 * @param robot 
 * @param recieved_angle_queue 
 */
void robot_control(franka::Robot& robot, CommandQueue& recieved_angle_queue);

/**
 * @brief update the reaultime operating postions queue for each joints,
 * the queue will be totally cleaned before add new values to it
 * 
 * @param desire_angle_queues 
 * @param previous_setting_angles 
 * @param recieved_angle_queue 
 */
void update_desire_velocity(std::queue<double> desire_angle_queues[], const arrayDOF& previous_setting_angles, CommandQueue& recieved_angle_queue);

/**
 * @brief generate joint velocities by pid optimizers
 * 
 * @param robot_state 
 * @param desire_angle_queues  queues storing the realtime operating position for each joints
 * @param pid_optimizers 
 * @param previous_setting_angles record the latest angle configuration 
 * @param previous_execution_velocities
 * @return franka::JointVelocities 
 */
franka::JointVelocities operate_pid(const franka::RobotState& robot_state, std::queue<double> desire_angle_queues[], PID pid_optimizers[], arrayDOF& previous_setting_angles);

};
};

#endif