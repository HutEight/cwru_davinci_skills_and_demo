//
// Created by william on 27/08/18.
//

#include "cwru_davinci_skills_and_demo/davinci_skills_management_base.h"
#include "cwru_davinci_skills_and_demo/cwru_davinci_skills_communications_node.h"
#include "cwru_davinci_skills_and_demo/DavinciSkillsCommunicationsManager.h"

using namespace cwru_davinci_skills_communications;

int main(int argc, char **argv) {


  ros::init(argc, argv, "cwru_davinci_skills_communications_node");
  ros::NodeHandle nh;

  DavinciSkillsCommunicationsManager davinci_skills_communications_manager(nh, "cwru_davinci_skills_communications_node", false);

//  while (ros::ok())
//  {
////    ros::spin();
//  }


  return 0; // Should never reach here
}