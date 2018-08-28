//
// Created by William on 27/08/18.
//

#include "cwru_davinci_skills_and_demo/DavinciSkillsCommunicationsManager.h"


DavinciSkillsCommunicationsManager::DavinciSkillsCommunicationsManager(ros::NodeHandle nodeHandle, string name, bool debug) :
    nh_(nodeHandle),
    action_server_(nh_, name,  boost::bind(&DavinciSkillsCommunicationsManager::actionServerExecuteCallback, this, _1), false),
    spinner_(5),
    debug_mode_toggle_(debug) {

  ROS_INFO("Initialising da Vinci Skills Communications Manager MK I.");

  initialiseTransforms();

  initialiseCommunications();

  spinner_.start();


//  runActionServerThread();



  // TODO should be wrapped into functions instead of calling like this? And the threads are not necessarily class members.
  boost::thread thread_information_manager_(&DavinciSkillsCommunicationsManager::manageInfromation, this);
//  boost::thread thread_active_skill_(&DavinciSkillsCommunicationsManager::runActiveSkill, this);
  boost::thread thread_robot_controller_(&DavinciSkillsCommunicationsManager::runRobotController, this);
//  boost::thread thread_functional_test_(&DavinciSkillsCommunicationsManager::runTrialFunctions, this);


  boost::thread thread_action_server_(&DavinciSkillsCommunicationsManager::runActionServer, this);
  ROS_WARN("RN DEBUG 2");


  ros::waitForShutdown();
}


bool DavinciSkillsCommunicationsManager::initialiseTransforms() {

  tf::TransformListener tfListener;
  tf::StampedTransform tfResult_one, tfResult_two, tfResult_three;
  DavinciSkillsAuxiliary davinci_skills_auxiliary;
  bool tf_acquired = false;
  int n_tries = 0;

  ROS_INFO("Attempting to get Camera PSMs transforms");

  while (!tf_acquired){
    if (n_tries > 5) break;
    tf_acquired = true;
    try {
      tfListener.lookupTransform("left_camera_link",
                                 "PSM1psm_base_link",
                                 ros::Time(0),
                                 tfResult_one);

      tfListener.lookupTransform("left_camera_link",
                                 "PSM2psm_base_link",
                                 ros::Time(0),
                                 tfResult_two);

      tfListener.lookupTransform("PSM1psm_base_link",
                                 "PSM2psm_base_link",
                                 ros::Time(0),
                                 tfResult_three);

    } catch (tf::TransformException &exception) {
      ROS_WARN("%s", exception.what());
      tf_acquired = false;
      ros::Duration(0.5).sleep();
      ros::spinOnce();
      n_tries++;
    }
  }

  if (!tf_acquired) {

    ROS_ERROR("Could NOT acquire Camera PSMs transforms transform.");
    return false;

  } else {

    ROS_INFO("Transform Info acquired.");

    boost::lock_guard<boost::mutex> guard(mtx_);

    basic_env_transforms_.psm_one_affine_wrt_lt_camera = davinci_skills_auxiliary.convertTFToEigen(tfResult_one);
    basic_env_transforms_.psm_two_affine_wrt_lt_camera = davinci_skills_auxiliary.convertTFToEigen(tfResult_two);
    basic_env_transforms_.psm_two_affine_wrt_psm_one = davinci_skills_auxiliary.convertTFToEigen(tfResult_three);
    basic_env_transforms_.psm_one_affine_wrt_psm_two = basic_env_transforms_.psm_two_affine_wrt_psm_one.inverse();

    if (debug_mode_toggle_) {
      ROS_INFO("Basic Transforms:");
      basic_env_transforms_.printAllTransforms();
    }

    return true;

  }


}


void DavinciSkillsCommunicationsManager::initialiseCommunications() {

}


void DavinciSkillsCommunicationsManager::manageInfromation() {

}


void DavinciSkillsCommunicationsManager::runActionServer() {
  ROS_INFO("Commnucations Action Server Stand By.");
  action_server_.start();
  ROS_INFO("Commnucations Action Server Ready.");
}

void DavinciSkillsCommunicationsManager::runActionServerThread() {
//  boost::thread thread_action_server_(&DavinciSkillsCommunicationsManager::runActionServer, this);
//  thread_action_server_.detach();
//
//  ROS_WARN("RN DEBUG");

}


void DavinciSkillsCommunicationsManager::runRobotController() {

}


void DavinciSkillsCommunicationsManager::runActiveTasks() {


}



/*
 * It will first switch cases to what types of tasks requested by the Client.
 */
void DavinciSkillsCommunicationsManager::actionServerExecuteCallback(const cwru_davinci_skills_and_demo::CwruDavinciSkillControlGoalConstPtr &goal) {

  ROS_INFO("New Goal Received!");





}

