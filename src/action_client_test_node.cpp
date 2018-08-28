//
// Created by William on 27/08/18.
//

#include "cwru_davinci_skills_and_demo/DavinciSkillsCommunicationsActionClient.h"


int main (int argc, char **argv)
{

  ros::init(argc, argv, "action_client_test_node");
  ros::NodeHandle nh;

  DavinciSkillsCommunicationsActionClient davinci_skills_communications_action_client(nh);

  davinci_skills_communications_action_client.testActionServer();

  return 0;
}

