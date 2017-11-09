/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/**
 * @file /include/op3_test_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef OP3_TEST_GUI_QNODE_HPP_
#define OP3_TEST_GUI_QNODE_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#ifndef Q_MOC_RUN

#include <map>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <string>
#include <boost/thread.hpp>
#include <QThread>
#include <QStringListModel>

#include <Eigen/Dense>

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <yaml-cpp/yaml.h>

#include "op3_wholebody_module_msgs/JointPose.h"
#include "op3_wholebody_module_msgs/KinematicsPose.h"
#include "op3_wholebody_module_msgs/FootStepCommand.h"
#include "op3_wholebody_module_msgs/WalkingParam.h"

#include "op3_wholebody_module_msgs/GetKinematicsPose.h"
#include "op3_wholebody_module_msgs/GetJointPose.h"

#endif

#define DEG2RAD   (M_PI / 180.0)
#define RAD2DEG   (180.0 / M_PI)

#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace op3_test_gui
{

/*****************************************************************************
 ** Class
 *****************************************************************************/

class QNode: public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  virtual ~QNode();
  bool init();
  void run();

  /*********************
   ** Logging
   **********************/
  enum LogLevel
  {
    Debug, Info, Warn, Error, Fatal
  };

  QStringListModel* loggingModel()
  {
    return &logging_model_;
  }
  void log(const LogLevel &level, const std::string &msg, std::string sender = "GUI");
  //    void statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg);

  void sendIniPoseMsg(std_msgs::String msg);
  void sendResetBodyMsg( std_msgs::Bool msg );

  void sendSetModeMsg();

  void sendJointPoseMsg(op3_wholebody_module_msgs::JointPose msg);
  void sendKinematicsPoseMsg(op3_wholebody_module_msgs::KinematicsPose msg);

  void sendFootStepCommandMsg(op3_wholebody_module_msgs::FootStepCommand msg);
  void sendWalkingParamMsg(op3_wholebody_module_msgs::WalkingParam msg);

  void sendBodyOffsetMsg(geometry_msgs::Pose msg);
  void sendWholebodyBalanceMsg(std_msgs::String msg);
  void sendFootDistanceMsg(std_msgs::Float64 msg);

//  void sendJointGainMsg(op3_wholebody_module_msgs::JointGain msg);

  //    void sendJointPoseMsg(manipulator_h_base_module_msgs::JointPose msg);
  //    void sendKinematicsPoseMsg(manipulator_h_base_module_msgs::KinematicsPose msg);

public Q_SLOTS:
  void getJointPose();
  void getKinematicsPose(std::string group_name);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateCurrentJointPose(op3_wholebody_module_msgs::JointPose);
  void updateCurrentKinematicsPose(op3_wholebody_module_msgs::KinematicsPose);

private:
  int     init_argc_;
  char**  init_argv_;

  ros::Publisher      chatter_publisher_;
  QStringListModel    logging_model_;

  ros::Publisher      set_ctrl_module_pub_;

  ros::Publisher      init_pose_msg_pub_;
  ros::Publisher      reset_body_msg_pub_;
  ros::Publisher      set_mode_msg_pub_;

  ros::Publisher      joint_pose_msg_pub_;
  ros::Publisher      kinematics_pose_msg_pub_;

  ros::Publisher      foot_step_command_pub_;
  ros::Publisher      walking_param_pub_;

  ros::Publisher      body_offset_pub_;
  ros::Publisher      foot_distance_pub_;
  ros::Publisher      wholebody_balance_pub_;

//  ros::Publisher      joint_gain_msg_pub_;

  ros::ServiceClient  get_joint_pose_client_;
  ros::ServiceClient  get_kinematics_pose_client_;

  ros::Subscriber     status_msg_sub_;

};

}  // namespace op3_test_gui

#endif /* op3_test_gui_QNODE_HPP_ */
