//
// Created by William on 27/08/18.
//

#include "cwru_davinci_skills_and_demo/DavinciSkillsCommunicationsActionClient.h"


/*
 * To be called in a thread after the client created.
 */
void cancelGoalIn5Sec(DavinciSkillsCommunicationsActionClient *client) {

  ros::Duration(5).sleep();
  client -> testCancelGoals();

}

int main (int argc, char **argv)
{

  ros::init(argc, argv, "action_client_test_node");
  ros::NodeHandle nh;

  DavinciSkillsCommunicationsActionClient *davinci_skills_communications_action_client =
      new DavinciSkillsCommunicationsActionClient(nh);


//  while (ros::ok()) {

    ROS_INFO("sending 1");
    davinci_skills_communications_action_client -> testActionServer();
    ros::Duration(1).sleep();

    ROS_INFO("sending 2");
    davinci_skills_communications_action_client -> testActionServer();
    ros::Duration(20).sleep();

    ROS_INFO("sending 3");
    davinci_skills_communications_action_client -> testActionServer();
    ros::Duration(1).sleep();

//  }



//  ros::Duration(5).sleep();
//  davinci_skills_communications_action_client-> testCancelGoals();
//
//  ros::Duration(1).sleep();
//  davinci_skills_communications_action_client -> testActionServer();
//
//  ros::Duration(1).sleep();
//  davinci_skills_communications_action_client -> testActionServer();
//
//  ros::Duration(1).sleep();
//  davinci_skills_communications_action_client -> testActionServer();
//
//  ros::Duration(1).sleep();
//  davinci_skills_communications_action_client -> testActionServer();
//
//
//  ROS_WARN("About to cancel goal from another thread.");
//  boost::thread testCancel(&DavinciSkillsCommunicationsActionClient::testCancelGoals, davinci_skills_communications_action_client);

//  boost::thread testCancelLocalFun(&cancelGoalIn5Sec, davinci_skills_communications_action_client);
//
//  ros::Duration(1).sleep();
//
//  davinci_skills_communications_action_client -> testActionServer();

//  testCancel.join();

  return 0;
}

