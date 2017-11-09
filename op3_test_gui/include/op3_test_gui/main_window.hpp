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
 * @file /include/op3_test_gui/main_window.hpp
 *
 * @brief Qt based gui for op3_test_gui.
 *
 * @date November 2010
 **/
#ifndef OP3_TEST_GUI_MAIN_WINDOW_H
#define OP3_TEST_GUI_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace op3_test_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(int argc, char** argv, QWidget *parent = 0);
  ~MainWindow();

  void closeEvent(QCloseEvent *event); // Overloaded function

  /******************************************
    ** Transformation
    *******************************************/
  Eigen::MatrixXd rotationX( double angle );
  Eigen::MatrixXd rotationY( double angle );
  Eigen::MatrixXd rotationZ( double angle );

  Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
  Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );

  Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
  Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );

  Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
  Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );

  void parseIniPoseData(const std::string &path);

public Q_SLOTS:
  /******************************************
     ** Auto-connections (connectSlotsByName())
     *******************************************/
  void on_actionAbout_triggered();

  void on_set_mode_button_clicked( bool check );

  void on_joint_send_box_clicked( bool check );
  void on_joint_get_box_clicked( bool check );

  void on_wholebody_send_button_clicked( bool check );
  void on_wholebody_get_button_clicked( bool check );
  void on_wb_init_pose_button_clicked( bool check );

  void on_walking_forward_button_clicked( bool check );
  void on_walking_backward_button_clicked( bool check );
  void on_walking_stop_button_clicked( bool check );
  void on_walking_turn_l_button_clicked( bool check );
  void on_walking_turn_r_button_clicked( bool check );
  void on_walking_right_button_clicked( bool check );
  void on_walking_left_button_clicked( bool check );

  void on_set_walking_param_button_clicked( bool check );
  void on_send_body_offset_button_clicked( bool check );
  void on_send_foot_distance_button_clicked( bool check );

  void on_init_pose_button_clicked( bool check );
  void on_zero_pose_button_clicked( bool check );

  void on_balance_on_button_clicked( bool check );
  void on_balance_off_button_clicked( bool check );

  void on_get_gain_button_clicked( bool check );
  void on_set_gain_button_clicked( bool check );
  void on_zero_gain_button_clicked( bool check );
  void on_default_gain_button_clicked( bool check );

  /******************************************
     ** Manual connections
     *******************************************/
  void updateLoggingView(); // no idea why this can't connect automatically

  void updateCurrJointPoseSpinbox( op3_wholebody_module_msgs::JointPose msg );
  void updateCurrKinematicsPoseSpinbox( op3_wholebody_module_msgs::KinematicsPose msg );

private:
  Ui::MainWindowDesign ui;
  QNode qnode;

  std::vector<std::string> joint_name;
  QList<QAbstractSpinBox *> joint_box;
  QList<QAbstractSpinBox *> ff_p_gain_box;
  QList<QAbstractSpinBox *> ff_d_gain_box;
  QList<QAbstractSpinBox *> fb_p_gain_box;
  QList<QAbstractSpinBox *> fb_d_gain_box;
};

}  // namespace op3_test_gui

#endif // op3_test_gui_MAIN_WINDOW_H
