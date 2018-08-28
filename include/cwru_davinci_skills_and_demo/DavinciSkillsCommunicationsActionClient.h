//
// Created by william on 27/08/18.
//

#ifndef CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSCOMMUNICATIONSACTIONCLIENT_H
#define CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSCOMMUNICATIONSACTIONCLIENT_H

#include "DavinciSkillsCommunicationsManager.h"


class DavinciSkillsCommunicationsActionClient {

 public:
  DavinciSkillsCommunicationsActionClient(ros::NodeHandle& nodeHandle,
                                          string name = "/cwru_davinci_skills_communications_node");

  void sendGoal(cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoal goal);

  void testActionServer();


 private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<cwru_davinci_skills_and_demo::CwruDavinciSkillControlAction> action_client_;
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const cwru_davinci_skills_and_demo::CwruDavinciSkillControlResultConstPtr& result);
  void activeCb();
  void feedbackCb(const cwru_davinci_skills_and_demo::CwruDavinciSkillControlFeedbackConstPtr& feedback);


};

#endif //CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSCOMMUNICATIONSACTIONCLIENT_H
