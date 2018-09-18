//
// Created by William on 17/09/18.
//

#include "cwru_davinci_skills_and_demo/davinci_skills_management_base.h"
#include <cwru_davinci_control/psm_controller.h>

double current_q1_;
double current_q2_;


void jointStateCb(const sensor_msgs::JointState jointState) {
  current_q1_ = jointState.position.at(6);
  current_q2_ = jointState.position.at(2);
}



void cutAndMove(psm_controller &psm, double q2, double time) {

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory.points.clear();
  joint_trajectory.joint_names.clear();
  joint_trajectory.header.stamp = ros::Time::now();

  joint_trajectory_point.positions.resize(7);
  joint_trajectory_point.positions.clear();

  std::cout << "current_q1_: " << current_q1_ << std::endl;

  joint_trajectory_point.positions.push_back(current_q1_);
  joint_trajectory_point.positions.push_back(q2);
  for (int i = 2; i < 7; i++) {
    joint_trajectory_point.positions.push_back(0);
  }

  joint_trajectory_point.time_from_start = ros::Duration(time);

  joint_trajectory.points.push_back(joint_trajectory_point);

  psm.move_psm(joint_trajectory);

}

void goToHomePose(psm_controller &psm) {

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory.points.clear();
  joint_trajectory.joint_names.clear();
  joint_trajectory.header.stamp = ros::Time::now();

  joint_trajectory_point.positions.resize(7);
  joint_trajectory_point.positions.clear();

  for (int i = 0; i < 7; i++) {
    joint_trajectory_point.positions.push_back(0);
  }
  joint_trajectory_point.time_from_start = ros::Duration(3);

  joint_trajectory.points.push_back(joint_trajectory_point);

  psm.move_psm(joint_trajectory);

  ros::Duration(3).sleep();

}



int main(int argc, char **argv) {



  ros::init(argc, argv, "test_controller_main");
  ros::NodeHandle nh;
  psm_controller psm1(1, nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Subscriber sub = nh.subscribe("/dvrk/PSM1/joint_states", 1000, jointStateCb);

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;

  joint_trajectory.points.clear();
  joint_trajectory.joint_names.clear();
  joint_trajectory.header.stamp = ros::Time::now();

  joint_trajectory_point.positions.resize(7);
  joint_trajectory_point.positions.clear();

  joint_trajectory_point.positions.push_back(1.0);

  for (int i = 1; i < 7; i++) {
    joint_trajectory_point.positions.push_back(0);
  }

  joint_trajectory_point.time_from_start = ros::Duration(10);
  joint_trajectory.points.push_back(joint_trajectory_point);

  ROS_WARN("Commencing test now.");

  goToHomePose(psm1);

  ros::Duration(0.5).sleep();

  psm1.move_psm(joint_trajectory_point);

  ros::Duration(2).sleep();

  cutAndMove(psm1, 0.5, 3);

  ros::Duration(5).sleep();
  std::cout << "current_q1_: " << current_q1_ << std::endl;
  std::cout << "current_q2_: " << current_q2_ << std::endl;

  ros::waitForShutdown();

  return 0; // Should never reach here
}