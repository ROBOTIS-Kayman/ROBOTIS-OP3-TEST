/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "op3_test_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_test_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  joint_name.push_back("r_leg_hip_y");
  joint_name.push_back("r_leg_hip_r");
  joint_name.push_back("r_leg_hip_p");
  joint_name.push_back("r_leg_kn_p");
  joint_name.push_back("r_leg_an_p");
  joint_name.push_back("r_leg_an_r");

  joint_name.push_back("l_leg_hip_y");
  joint_name.push_back("l_leg_hip_r");
  joint_name.push_back("l_leg_hip_p");
  joint_name.push_back("l_leg_kn_p");
  joint_name.push_back("l_leg_an_p");
  joint_name.push_back("l_leg_an_r");

  //  joint_name.push_back("OP3_right_shoulder_pitch_joint");
  //  joint_name.push_back("OP3_right_shoulder_roll_joint");
  //  joint_name.push_back("OP3_right_elbow_joint");

  //  joint_name.push_back("OP3_left_shoulder_pitch_joint");
  //  joint_name.push_back("OP3_left_shoulder_roll_joint");
  //  joint_name.push_back("OP3_left_elbow_joint");

  //  joint_name.push_back("OP3_head_pan_joint");
  //  joint_name.push_back("OP3_head_tilt_joint");

  /*********************
    ** Logging
    **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  joint_box.append( ui.rl1_box );
  joint_box.append( ui.rl2_box );
  joint_box.append( ui.rl3_box );
  joint_box.append( ui.rl4_box );
  joint_box.append( ui.rl5_box );
  joint_box.append( ui.rl6_box );

  joint_box.append( ui.ll1_box );
  joint_box.append( ui.ll2_box );
  joint_box.append( ui.ll3_box );
  joint_box.append( ui.ll4_box );
  joint_box.append( ui.ll5_box );
  joint_box.append( ui.ll6_box );

  //  joint_box.append( ui.ra1_box );
  //  joint_box.append( ui.ra2_box );
  //  joint_box.append( ui.ra3_box );

  //  joint_box.append( ui.la1_box );
  //  joint_box.append( ui.la2_box );
  //  joint_box.append( ui.la3_box );

  //  joint_box.append( ui.h1_box );
  //  joint_box.append( ui.h2_box );

  ff_p_gain_box.append( ui.hip_yaw_ff_p_gain_box );
  ff_p_gain_box.append( ui.hip_roll_ff_p_gain_box );
  ff_p_gain_box.append( ui.hip_pitch_ff_p_gain_box );
  ff_p_gain_box.append( ui.knee_ff_p_gain_box );
  ff_p_gain_box.append( ui.ankle_pitch_ff_p_gain_box );
  ff_p_gain_box.append( ui.ankle_roll_ff_p_gain_box );
  ff_p_gain_box.append( ui.sh_pitch_ff_p_gain_box );
  ff_p_gain_box.append( ui.sh_roll_ff_p_gain_box );
  ff_p_gain_box.append( ui.elbow_ff_p_gain_box );
  ff_p_gain_box.append( ui.head_pan_ff_p_gain_box );
  ff_p_gain_box.append( ui.head_tilt_ff_p_gain_box );

  ff_d_gain_box.append( ui.hip_yaw_ff_d_gain_box );
  ff_d_gain_box.append( ui.hip_roll_ff_d_gain_box );
  ff_d_gain_box.append( ui.hip_pitch_ff_d_gain_box );
  ff_d_gain_box.append( ui.knee_ff_d_gain_box );
  ff_d_gain_box.append( ui.ankle_pitch_ff_d_gain_box );
  ff_d_gain_box.append( ui.ankle_roll_ff_d_gain_box );
  ff_d_gain_box.append( ui.sh_pitch_ff_d_gain_box );
  ff_d_gain_box.append( ui.sh_roll_ff_d_gain_box );
  ff_d_gain_box.append( ui.elbow_ff_d_gain_box );
  ff_d_gain_box.append( ui.head_pan_ff_d_gain_box );
  ff_d_gain_box.append( ui.head_tilt_ff_d_gain_box );

  fb_p_gain_box.append( ui.hip_yaw_fb_p_gain_box );
  fb_p_gain_box.append( ui.hip_roll_fb_p_gain_box );
  fb_p_gain_box.append( ui.hip_pitch_fb_p_gain_box );
  fb_p_gain_box.append( ui.knee_fb_p_gain_box );
  fb_p_gain_box.append( ui.ankle_pitch_fb_p_gain_box );
  fb_p_gain_box.append( ui.ankle_roll_fb_p_gain_box );
  fb_p_gain_box.append( ui.sh_pitch_fb_p_gain_box );
  fb_p_gain_box.append( ui.sh_roll_fb_p_gain_box );
  fb_p_gain_box.append( ui.elbow_fb_p_gain_box );
  fb_p_gain_box.append( ui.head_pan_fb_p_gain_box );
  fb_p_gain_box.append( ui.head_tilt_fb_p_gain_box );

  fb_d_gain_box.append( ui.hip_yaw_fb_d_gain_box );
  fb_d_gain_box.append( ui.hip_roll_fb_d_gain_box );
  fb_d_gain_box.append( ui.hip_pitch_fb_d_gain_box );
  fb_d_gain_box.append( ui.knee_fb_d_gain_box );
  fb_d_gain_box.append( ui.ankle_pitch_fb_d_gain_box );
  fb_d_gain_box.append( ui.ankle_roll_fb_d_gain_box );
  fb_d_gain_box.append( ui.sh_pitch_fb_d_gain_box );
  fb_d_gain_box.append( ui.sh_roll_fb_d_gain_box );
  fb_d_gain_box.append( ui.elbow_fb_d_gain_box );
  fb_d_gain_box.append( ui.head_pan_fb_d_gain_box );
  fb_d_gain_box.append( ui.head_tilt_fb_d_gain_box );

  /****************************
    ** Connect
    ****************************/

  qRegisterMetaType<op3_wholebody_module_msgs::JointPose>("op3_wholebody_module_msgs::JointPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentJointPose(op3_wholebody_module_msgs::JointPose)), this, SLOT(updateCurrJointPoseSpinbox(op3_wholebody_module_msgs::JointPose)));

  qRegisterMetaType<op3_wholebody_module_msgs::KinematicsPose>("op3_wholebody_module_msgs::KinematicsPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentKinematicsPose(op3_wholebody_module_msgs::KinematicsPose)), this, SLOT(updateCurrKinematicsPoseSpinbox(op3_wholebody_module_msgs::KinematicsPose)));

  /*********************
    ** Auto Start
    **********************/
  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

//void MainWindow::on_curr_joint_button_clicked( bool check )
//{
//  qnode.getJointPose( joint_name );
//}

void MainWindow::on_balance_on_button_clicked( bool check )
{
  std_msgs::String msg;
  msg.data = "balance_on";

  qnode.sendWholebodyBalanceMsg(msg);
}

void MainWindow::on_balance_off_button_clicked( bool check )
{
  std_msgs::String msg;
  msg.data = "balance_off";

  qnode.sendWholebodyBalanceMsg(msg);
}

void MainWindow::on_set_mode_button_clicked( bool check )
{
  qnode.sendSetModeMsg();
}

void MainWindow::on_joint_get_box_clicked( bool check )
{
  qnode.getJointPose();
}

void MainWindow::on_joint_send_box_clicked( bool check )
{
  op3_wholebody_module_msgs::JointPose msg;
  msg.mov_time = ui.joint_mov_time_box->value();

  for ( int _id = 0; _id < joint_box.size(); _id++ )
  {
    msg.pose.name.push_back( joint_name[ _id ] );
    msg.pose.position.push_back( ((QDoubleSpinBox *) joint_box[ _id ])->value() * M_PI / 180.0 );
  }

  qnode.sendJointPoseMsg( msg );
}

//void MainWindow::on_curr_pos_button_clicked( bool check )
//{

//}

//void MainWindow::on_des_pos_button_clicked( bool check )
//{
//  op3_wholebody_module_msgs::KinematicsPose msg;

//  msg.mov_time = ui.mov_time_spinbox->value();
//  msg.pose.position.x = ui.pos_x_spinbox->value();
//  msg.pose.position.y = ui.pos_y_spinbox->value();
//  msg.pose.position.z = ui.pos_z_spinbox->value();

//  double roll = ui.ori_roll_spinbox->value() * M_PI / 180.0;
//  double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
//  double yaw = ui.ori_yaw_spinbox->value() * M_PI / 180.0;

//  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

//  msg.pose.orientation.x = QR.x();
//  msg.pose.orientation.y = QR.y();
//  msg.pose.orientation.z = QR.z();
//  msg.pose.orientation.w = QR.w();

//  qnode.sendKinematicsPoseMsg( msg );
//}

void MainWindow::on_init_pose_button_clicked( bool check )
{
  std::string ini_pose_path = ros::package::getPath("op3_test_gui") + "/config/init_pose.yaml";
  parseIniPoseData(ini_pose_path);

  std_msgs::Bool msg;
  msg.data = true;

  qnode.sendResetBodyMsg(msg);
}

void MainWindow::on_zero_pose_button_clicked( bool check )
{
  std::string ini_pose_path = ros::package::getPath("op3_test_gui") + "/config/zero_pose.yaml";
  parseIniPoseData(ini_pose_path);
}

void MainWindow::on_wholebody_send_button_clicked( bool check )
{
  op3_wholebody_module_msgs::KinematicsPose msg;

  msg.name = ui.wholebody_group_name_box->currentText().toStdString();

  msg.mov_time = ui.wholebody_mov_time_box->value();

  msg.pose.position.x = ui.goal_x_box->value();
  msg.pose.position.y = ui.goal_y_box->value();
  msg.pose.position.z = ui.goal_z_box->value();

  double roll = ui.goal_roll_box->value() * DEG2RAD;
  double pitch = ui.goal_pitch_box->value() * DEG2RAD;
  double yaw = ui.goal_yaw_box->value() * DEG2RAD;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_wholebody_get_button_clicked( bool check )
{
  std::string group_name = ui.wholebody_group_name_box->currentText().toStdString();
  qnode.getKinematicsPose(group_name);
}

void MainWindow::on_wb_init_pose_button_clicked( bool check )
{
  op3_wholebody_module_msgs::KinematicsPose msg;

  msg.name = "body";

  msg.mov_time = 1.0;

  msg.pose.position.x = 0.0;
  msg.pose.position.y = 0.0;
  msg.pose.position.z = 0.727;

  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;

  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_walking_forward_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = ui.walking_start_leg_box->currentText().toStdString();

  msg.command = "forward";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_walking_backward_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = ui.walking_start_leg_box->currentText().toStdString();

  msg.command = "backward";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_walking_stop_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = ui.walking_start_leg_box->currentText().toStdString();

  msg.command = "stop";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_walking_turn_l_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = "";

  msg.command = "turn_left";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_walking_turn_r_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = "";

  msg.command = "turn_right";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_walking_right_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = "";

  msg.command = "right";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_walking_left_button_clicked( bool check )
{
  op3_wholebody_module_msgs::FootStepCommand msg;

  msg.step_time = ui.walking_step_time_box->value();
  msg.step_num = ui.walking_step_num_box->value();
  msg.step_length = ui.walking_step_length_box->value();
  msg.side_length = ui.walking_side_length_box->value();
  msg.step_angle = ui.walking_step_angle_box->value()*DEG2RAD;
  msg.start_leg = "";

  msg.command = "left";

  qnode.sendFootStepCommandMsg(msg);
}

void MainWindow::on_set_walking_param_button_clicked( bool check )
{
  op3_wholebody_module_msgs::WalkingParam msg;

  msg.dsp_ratio = ui.dsp_ratio_box->value();
  msg.lipm_height = ui.lipm_height_box->value();
  msg.foot_height_max = ui.foot_height_max_box->value();
  msg.zmp_offset_x = ui.zmp_offset_x_box->value();
  msg.zmp_offset_y = ui.zmp_offset_y_box->value();

  qnode.sendWalkingParamMsg(msg);
}

void MainWindow::on_send_body_offset_button_clicked( bool check )
{
  geometry_msgs::Pose msg;
  msg.position.x = ui.body_offset_x_box->value();
  msg.position.y = ui.body_offset_y_box->value();
  msg.position.z = ui.body_offset_z_box->value();

  qnode.sendBodyOffsetMsg(msg);
}

void MainWindow::on_send_foot_distance_button_clicked( bool check )
{
  std_msgs::Float64 msg;
  msg.data = ui.foot_distance_box->value();

  qnode.sendFootDistanceMsg(msg);
}

void MainWindow::updateCurrJointPoseSpinbox( op3_wholebody_module_msgs::JointPose msg )
{
  for ( int i=0; i<msg.pose.name.size(); i++ )
    ((QDoubleSpinBox *) joint_box[i])->setValue( msg.pose.position[i] * RAD2DEG );
}

void MainWindow::updateCurrKinematicsPoseSpinbox( op3_wholebody_module_msgs::KinematicsPose msg )
{
  ui.goal_x_box->setValue( msg.pose.position.x );
  ui.goal_y_box->setValue( msg.pose.position.y );
  ui.goal_z_box->setValue( msg.pose.position.z );

  Eigen::Quaterniond QR( msg.pose.orientation.w , msg.pose.orientation.x , msg.pose.orientation.y , msg.pose.orientation.z );
  Eigen::MatrixXd rpy = quaternion2rpy( QR );

  double roll = rpy.coeff( 0 , 0 ) * 180.0 / M_PI;
  double pitch = rpy.coeff( 1 , 0 ) * 180.0 / M_PI;
  double yaw = rpy.coeff( 2, 0 ) * 180.0 /M_PI;

  ui.goal_roll_box->setValue( roll );
  ui.goal_pitch_box->setValue( pitch );
  ui.goal_yaw_box->setValue( yaw );
}

void MainWindow::on_get_gain_button_clicked( bool check )
{

}

void MainWindow::on_set_gain_button_clicked( bool check )
{
  //  op3_wholebody_module_msgs::JointGain msg;

  //  int max_leg_id = 6;

  //  for (int i=0; i<max_leg_id; i++)
  //  {
  //    msg.joint_name.push_back( joint_name[i] );
  //    msg.ff_p_gain.push_back( ((QDoubleSpinBox *) ff_p_gain_box[i])->value() );
  //    msg.ff_d_gain.push_back( ((QDoubleSpinBox *) ff_d_gain_box[i])->value() );
  //    msg.fb_p_gain.push_back( ((QDoubleSpinBox *) fb_p_gain_box[i])->value() );
  //    msg.fb_d_gain.push_back( ((QDoubleSpinBox *) fb_d_gain_box[i])->value() );
  //  }

  //  for (int i=max_leg_id; i<max_leg_id+max_leg_id; i++)
  //  {
  //    msg.joint_name.push_back( joint_name[i] );
  //    msg.ff_p_gain.push_back( ((QDoubleSpinBox *) ff_p_gain_box[i - max_leg_id] )->value() );
  //    msg.ff_d_gain.push_back( ((QDoubleSpinBox *) ff_d_gain_box[i - max_leg_id] )->value() );
  //    msg.fb_p_gain.push_back( ((QDoubleSpinBox *) fb_p_gain_box[i - max_leg_id] )->value() );
  //    msg.fb_d_gain.push_back( ((QDoubleSpinBox *) fb_d_gain_box[i - max_leg_id] )->value() );
  //  }

  //  qnode.sendJointGainMsg( msg );
}

void MainWindow::on_zero_gain_button_clicked( bool check )
{
  //  for (int i=0; i<ff_p_gain_box.size(); i++)
  //  {
  //    ((QDoubleSpinBox *) ff_p_gain_box[i])->setValue( 0.0 );
  //    ((QDoubleSpinBox *) ff_d_gain_box[i])->setValue( 0.0 );
  //    ((QDoubleSpinBox *) fb_p_gain_box[i])->setValue( 0.0 );
  //    ((QDoubleSpinBox *) fb_d_gain_box[i])->setValue( 0.0 );
  //  }
}

void MainWindow::on_default_gain_button_clicked( bool check )
{
  //  for (int i=0; i<ff_p_gain_box.size(); i++)
  //  {
  //    ((QDoubleSpinBox *) ff_p_gain_box[i])->setValue( 1.0 );
  //    ((QDoubleSpinBox *) ff_d_gain_box[i])->setValue( 0.0 );
  //    ((QDoubleSpinBox *) fb_p_gain_box[i])->setValue( 5.0 );
  //    ((QDoubleSpinBox *) fb_d_gain_box[i])->setValue( 0.05 );
  //  }

  //  ((QDoubleSpinBox *) fb_p_gain_box[0])->setValue( 25.0 );
  //  ((QDoubleSpinBox *) fb_d_gain_box[0])->setValue( 0.5 );

  //  ((QDoubleSpinBox *) fb_p_gain_box[1])->setValue( 20.0 );
  //  ((QDoubleSpinBox *) fb_d_gain_box[1])->setValue( 0.2 );

  //  ((QDoubleSpinBox *) fb_p_gain_box[2])->setValue( 15.0 );
  //  ((QDoubleSpinBox *) fb_d_gain_box[2])->setValue( 0.35 );

  //  ((QDoubleSpinBox *) fb_p_gain_box[3])->setValue( 10.0 );

  //  ((QDoubleSpinBox *) fb_p_gain_box[4])->setValue( 3.0 );
  //  ((QDoubleSpinBox *) fb_d_gain_box[4])->setValue( 0.01 );

  //  ((QDoubleSpinBox *) fb_p_gain_box[5])->setValue( 3.0 );
  //  ((QDoubleSpinBox *) fb_d_gain_box[5])->setValue( 0.01 );
}

Eigen::MatrixXd MainWindow::rotationX( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationY( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
      0.0, 1.0, 	       0.0,
      -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationZ( double angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotation2rpy( Eigen::MatrixXd rotation )
{
  Eigen::MatrixXd _rpy = Eigen::MatrixXd::Zero( 3 , 1 );

  _rpy.coeffRef( 0 , 0 ) = atan2( rotation.coeff( 2 , 1 ), rotation.coeff( 2 , 2 ) );
  _rpy.coeffRef( 1 , 0 ) = atan2( -rotation.coeff( 2 , 0 ), sqrt( pow( rotation.coeff( 2 , 1 ) , 2 ) + pow( rotation.coeff( 2 , 2 ) , 2 ) ) );
  _rpy.coeffRef( 2 , 0 ) = atan2 ( rotation.coeff( 1 , 0 ) , rotation.coeff( 0 , 0 ) );

  return _rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );

  return _rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd MainWindow::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

  return _rotation;
}

void MainWindow::parseIniPoseData(const std::string &path)
{
  YAML::Node doc;
  try
  {
    // load yaml
    doc = YAML::LoadFile(path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  op3_wholebody_module_msgs::JointPose msg;

  // parse movement time
  double mov_time = doc["mov_time"].as<double>();
  msg.mov_time = mov_time;

  // parse target pose
  YAML::Node tar_pose_node = doc["tar_pose"];
  for (YAML::iterator it = tar_pose_node.begin(); it != tar_pose_node.end(); ++it)
  {
    std::string joint_name = it->first.as<std::string>();
    double value = it->second.as<double>();

    msg.pose.name.push_back(joint_name);
    msg.pose.position.push_back(value * DEG2RAD);
  }

  qnode.sendJointPoseMsg( msg );
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Robotis</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace op3_test_gui

