//
// Created by William on 17/09/18.
//

#include "cwru_davinci_skills_and_demo/davinci_skills_management_base.h"
#include <cwru_davinci_control/psm_controller.h>
#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */


double current_q1_;
double current_q2_;
double current_q1_pms2_;
double current_q2_pms2_;
const double kPace(0.1);

boost::mutex mtx_;

double final_q1_target_(1.0);

double countdown_(0);

void goToHomePose(psm_controller &psm);

void jointStateCbPsmOne(const sensor_msgs::JointState jointState) {
  current_q1_ = jointState.position.at(6);
  current_q2_ = jointState.position.at(2);
}

void jointStateCbPsmTwo(const sensor_msgs::JointState jointState) {
  current_q1_pms2_ = jointState.position.at(6);
  current_q2_pms2_ = jointState.position.at(2);
}
void addJointTwoOffset(psm_controller &psm) {

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;

  /*
   * Clean up the messages
   */
  joint_trajectory.points.clear();
  joint_trajectory.joint_names.clear();
  joint_trajectory.header.stamp = ros::Time::now();



  double next_q1;
  double time_from_start(0);

  double q2(0.1);
  double q3(0.15);

  next_q1 = current_q1_ + kPace;
  time_from_start += 1;

  int remaining_step = (final_q1_target_ - next_q1)/kPace;

  for (int n = 0; n < remaining_step; n++) {

    joint_trajectory_point.positions.resize(7);
    joint_trajectory_point.positions.clear();
    joint_trajectory_point.positions.push_back(next_q1);
    joint_trajectory_point.positions.push_back(q2);
    joint_trajectory_point.positions.push_back(q3);
    for (int i = 3; i < 7; i++) {
      joint_trajectory_point.positions.push_back(0);
    }

    joint_trajectory_point.time_from_start = ros::Duration(time_from_start);
    joint_trajectory.points.push_back(joint_trajectory_point);

    next_q1 +=  kPace;
    time_from_start += 1;

  }


  ROS_WARN("Adding Joint 2 offset now!");
  psm.move_psm(joint_trajectory);

  ros::Duration(time_from_start-1).sleep();
  ROS_WARN("Execution Complete! (thread sleep over)");


}


void runPlanningThread(psm_controller &psm) {

  int randnum;
  double randtime;
  /* initialize random seed: */
  srand (time(NULL));
  randnum = rand() % 100;

  randtime = ((double)randnum/120.0)*countdown_;

  ROS_INFO("Random time generated. Joint 2 offset will be added at that point.");
  std::cout << "randtime: " << randtime << std::endl;

  ros::Duration(randtime).sleep();

  ROS_INFO("Current Joint 1: %f", current_q1_);

  addJointTwoOffset(psm);

  /* Keep the thread alive */
  while (ros::ok()) {

  }


}


int main(int argc, char **argv) {

  ros::init(argc, argv, "dynamic_trajectory_swtich_main");
  ros::NodeHandle nh;
  psm_controller psm1(1, nh);
  psm_controller psm2(2, nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::Subscriber sub_1 = nh.subscribe("/dvrk/PSM1/joint_states", 1000, jointStateCbPsmOne);

  ros::Subscriber sub_2 = nh.subscribe("/dvrk/PSM2/joint_states", 1000, jointStateCbPsmTwo);

  trajectory_msgs::JointTrajectoryPoint joint_trajectory_point;
  trajectory_msgs::JointTrajectory joint_trajectory;

  /*
   * Clean up the messages
   */
  joint_trajectory.points.clear();
  joint_trajectory.joint_names.clear();
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory_point.positions.resize(7);

  double q1(-1.0);
  double time_from_start(6);

  double steps;

  steps = (final_q1_target_ - q1)/kPace;

  for (int n = 0; n < steps; n++) {
    joint_trajectory_point.positions.resize(7);
    joint_trajectory_point.positions.clear();
    joint_trajectory_point.positions.push_back(q1);

    for (int i = 1; i < 7; i++) {
      joint_trajectory_point.positions.push_back(0);
    }

    joint_trajectory_point.time_from_start = ros::Duration(time_from_start);
    joint_trajectory.points.push_back(joint_trajectory_point);

    time_from_start += 1;
    q1 += kPace;

  }

  countdown_ = time_from_start - 1;

  ROS_WARN("Commencing test now.");
  goToHomePose(psm1);
  ros::Duration(0.5).sleep();

  psm1.move_psm(joint_trajectory);

  /* Use a boost ref otherwise the thread try to make a copy of it. */
  boost::thread thread_1(runPlanningThread, boost::ref(psm1));
  boost::thread thread_2(runPlanningThread, boost::ref(psm2));


  ros::Duration(countdown_).sleep();
  ROS_WARN("Execution Complete! (main sleep over)");


  ros::waitForShutdown();

  thread_1.join();
  thread_2.join();

  return 0; // Should never reach here
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