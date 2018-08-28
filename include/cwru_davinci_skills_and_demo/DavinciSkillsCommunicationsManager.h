//
// Created by William on 27/08/18.
//



#ifndef CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSCOMMUNICATIONSMANAGER_H
#define CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSCOMMUNICATIONSMANAGER_H

#include <actionlib/server/simple_action_server.h>
#include "cwru_davinci_skills_and_demo/davinci_skills_management_base.h"
#include "cwru_davinci_skills_and_demo/DavinciSkillsAuxiliary.h"


using namespace cwru_davinci_skills;
using namespace cwru_davinci_skills_communications;
using std::string;

class DavinciSkillsCommunicationsManager {
 public:

  explicit DavinciSkillsCommunicationsManager(ros::NodeHandle nodeHandle, string name, bool debug);
  ~DavinciSkillsCommunicationsManager(void){};

  inline void setDebugMode(bool debug) {

      debug_mode_toggle_ = debug;

      if (debug) {
        ROS_WARN("Debug mode enabled.");
      } else {
        ROS_INFO("Debug mode disabled.");
      }

  }

  /*
   * Obtains the basic transform information from ROS.
   */
  bool initialiseTransforms();

  /*
   * Sets up publishers and listeners, and initialise controllers to PSMs.
   */
  void initialiseCommunications();


  /*
   * Defines the common operation zone with the PSMs transform.
   */
  bool defineCommonOperationZone();

  /*
   * A function for multi-threading.
   * Handles backgroud information exchange and updates.
   */
  void manageInfromation();
  void runInformationManagementThread();

  void startActionServer();

  void runRobotController();

  void runActiveTasks();

  void detachAllThreads();


 private:

  boost::mutex mtx_;

  ros::Timer exe_timer_;
  ros::NodeHandle nh_;

  bool debug_mode_toggle_;

  /* Action Server Related */
  actionlib::SimpleActionServer<cwru_davinci_skills_and_demo::CwruDavinciSkillControlAction> action_server_;
  cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoal action_goal_;
  cwru_davinci_skills_and_demo::CwruDavinciSkillControlFeedback action_feedback_;
  cwru_davinci_skills_and_demo::CwruDavinciSkillControlResult action_result_;
  void actionServerExecuteCallback(const cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoalConstPtr &goal);
  bool isPreempt_;
  bool goalComplete_;

  /* Kinematics */
  davinci_kinematics::Inverse dvrk_inverse_;


  /* Multi-threading */
  boost::thread thread_information_manager_;
  ros::AsyncSpinner spinner_;

  BasicEnvTransforms basic_env_transforms_;

//  DavinciSkillsAuxiliary davinci_skills_auxiliary_;


};

#endif //CWRU_DAVINCI_SKILLS_AND_DEMO_DAVINCISKILLSCOMMUNICATIONSMANAGER_H
