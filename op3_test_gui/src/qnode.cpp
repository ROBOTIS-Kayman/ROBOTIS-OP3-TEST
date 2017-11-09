/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "op3_test_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op3_test_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc_(argc),
  init_argv_(argv)
{}

QNode::~QNode() {
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc_,init_argv_,"op3_test_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  set_ctrl_module_pub_ = n.advertise<std_msgs::String>("/robotis/enable_ctrl_module", 1);

  init_pose_msg_pub_ = n.advertise<std_msgs::String>("/robotis/init_pose", 0);
  reset_body_msg_pub_ = n.advertise<std_msgs::Bool>("/robotis/reset_body", 0);

  joint_pose_msg_pub_ = n.advertise<op3_wholebody_module_msgs::JointPose>("/robotis/goal_joint_pose", 0);
  kinematics_pose_msg_pub_ = n.advertise<op3_wholebody_module_msgs::KinematicsPose>("/robotis/goal_kinematics_pose", 0);

  foot_step_command_pub_ = n.advertise<op3_wholebody_module_msgs::FootStepCommand>("/robotis/foot_step_command", 0);
  walking_param_pub_ = n.advertise<op3_wholebody_module_msgs::WalkingParam>("/robotis/walking_param", 0);

  body_offset_pub_ = n.advertise<geometry_msgs::Pose>("/robotis/wholebody/body_offset", 0);
  foot_distance_pub_ = n.advertise<std_msgs::Float64>("/robotis/wholebody/foot_distance", 0);
  wholebody_balance_pub_ = n.advertise<std_msgs::String>("/robotis/wholebody_balance_msg", 0);

//  joint_gain_msg_pub_ = n.advertise<op3_wholebody_module_msgs::JointGain>("/robotis/joint_gain", 0);

  get_joint_pose_client_ = n.serviceClient<op3_wholebody_module_msgs::GetJointPose>("/robotis/get_joint_pose", 0);
  get_kinematics_pose_client_ = n.serviceClient<op3_wholebody_module_msgs::GetKinematicsPose>("/robotis/get_kinematics_pose", 0);

  //  status_msg_sub_ = n.subscribe("/robotis/status", 10, &QNode::statusMsgCallback, this);

  start();
  return true;
}

void QNode::run() {

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendSetModeMsg()
{
  std_msgs::String str_msg;
  str_msg.data = "wholebody_module";

  set_ctrl_module_pub_.publish(str_msg);

  return;
}

void QNode::sendWholebodyBalanceMsg(std_msgs::String msg)
{
  wholebody_balance_pub_.publish( msg );

  log( Info , "Wholebody Balance Msg" );
}

void QNode::sendJointPoseMsg(op3_wholebody_module_msgs::JointPose msg)
{
  joint_pose_msg_pub_.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

void QNode::sendKinematicsPoseMsg( op3_wholebody_module_msgs::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  log( Info , "Send Kinematics Pose Msg" );
}

void QNode::sendFootStepCommandMsg(op3_wholebody_module_msgs::FootStepCommand msg)
{
  foot_step_command_pub_.publish(msg);

  log( Info , "Send Foot Step Command Msg" );
}

void QNode::sendIniPoseMsg( std_msgs::String msg )
{
  init_pose_msg_pub_.publish ( msg );
  log( Info , "Go to Initial Pose" );
}

void QNode::sendResetBodyMsg( std_msgs::Bool msg )
{
  reset_body_msg_pub_.publish( msg );
  log( Info , "Reset Body Pose" );
}

void QNode::sendWalkingParamMsg(op3_wholebody_module_msgs::WalkingParam msg)
{
  walking_param_pub_.publish(msg);
  log( Info, "Set Walking Parameter");
}

void QNode::sendBodyOffsetMsg(geometry_msgs::Pose msg)
{
  body_offset_pub_.publish(msg);
  log( Info, "Send Body Offset");
}

void QNode::sendFootDistanceMsg(std_msgs::Float64 msg)
{
  foot_distance_pub_.publish(msg);
  log( Info, "Send Foot Distance");
}


//void QNode::sendJointGainMsg(op3_wholebody_module_msgs::JointGain msg)
//{
//  joint_gain_msg_pub_.publish(msg);
//  log( Info, "Set Joint Gain");
//}

void QNode::getJointPose()
{
  log( Info , "Get Current Joint Pose" );

  op3_wholebody_module_msgs::GetJointPose get_joint_pose;

  // request

  // response
  if ( get_joint_pose_client_.call ( get_joint_pose ) )
  {
    op3_wholebody_module_msgs::JointPose joint_pose;

    for ( int i = 0; i < get_joint_pose.response.pose.pose.name.size(); i++ )
    {
      joint_pose.pose.name.push_back( get_joint_pose.response.pose.pose.name[i] );
      joint_pose.pose.position.push_back( get_joint_pose.response.pose.pose.position[i] );
    }

    Q_EMIT updateCurrentJointPose( joint_pose );
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNode::getKinematicsPose ( std::string group_name )
{
  log( Info , "Get Current Kinematics Pose" );

  op3_wholebody_module_msgs::GetKinematicsPose get_kinematics_pose;

  // request
  get_kinematics_pose.request.name = group_name;

  // response
  if ( get_kinematics_pose_client_.call( get_kinematics_pose ) )
  {
    op3_wholebody_module_msgs::KinematicsPose kinematcis_pose;

    kinematcis_pose.name = get_kinematics_pose.request.name;
    kinematcis_pose.pose = get_kinematics_pose.response.pose.pose;

    Q_EMIT updateCurrentKinematicsPose( kinematcis_pose );
  }
  else
    log(Error, "fail to get kinematcis pose.");
}

}  // namespace op3_test_gui
