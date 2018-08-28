//
// Created by william on 27/08/18.
//

#include "cwru_davinci_skills_and_demo/DavinciSkillsCommunicationsActionClient.h"


DavinciSkillsCommunicationsActionClient::DavinciSkillsCommunicationsActionClient(ros::NodeHandle& nodeHandle,
                                                                                 string name) :
    nh_(nodeHandle), action_client_(name, true) {

  ROS_INFO("Skills Communication Action Client Startup.");
  action_client_.waitForServer();

}


void DavinciSkillsCommunicationsActionClient::sendGoal(cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoal goal) {

  action_client_.sendGoal(goal, boost::bind(&DavinciSkillsCommunicationsActionClient::doneCb, this, _1, _2),
               boost::bind(&DavinciSkillsCommunicationsActionClient::activeCb, this),
               boost::bind(&DavinciSkillsCommunicationsActionClient::feedbackCb, this, _1));


}



void DavinciSkillsCommunicationsActionClient::doneCb(const actionlib::SimpleClientGoalState& state,
                                                     const cwru_davinci_skills_and_demo::CwruDavinciSkillControlResultConstPtr& result) {

  bool temp = result->success;

  if (temp)
  {
    std::cout << "    \e[1m\e[32mGoal Complete!\e[0m" << std::endl;
  } else {
    std::cout << "    \e[31mGoal Execution Failed." << std::endl;
  }

}


void DavinciSkillsCommunicationsActionClient::activeCb() {



}


void DavinciSkillsCommunicationsActionClient::feedbackCb(const cwru_davinci_skills_and_demo::CwruDavinciSkillControlFeedbackConstPtr& feedback) {



}


void DavinciSkillsCommunicationsActionClient::testActionServer() {

  bool finished_before_timeout;
  cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoal goal;

  goal.skill_type = cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoal::TEST_DEMO_ONE;
  goal.request_skill_planning = true;

  sendGoal(goal);
  finished_before_timeout = action_client_.waitForResult(ros::Duration(5));

  if (!finished_before_timeout) {
    ROS_WARN("Timeout! Server did not respond.");
  }

}


